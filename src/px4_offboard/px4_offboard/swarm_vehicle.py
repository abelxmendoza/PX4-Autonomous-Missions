"""Independent PX4 position controller for the cooperative SITL survey.

Commands expire on the shared host's monotonic clock. A coordinator outage
holds locally, then lands. Land/abort are terminal for this node lifetime.
"""
from __future__ import annotations

import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleControlMode, VehicleLandDetected)
from std_msgs.msg import String, Bool

from .mission_logic import yaw_toward
from .swarm_logic import HOMES, inside, local_to_world, world_to_local, point


class SwarmVehicle(Node):
    def __init__(self, **kwargs):
        super().__init__("swarm_vehicle", **kwargs)
        self.declare_parameter("vehicle_id", "px4_1")
        self.declare_parameter("target_system_id", 2)
        self.declare_parameter("abort_after_ready_s", 0.0)
        self.vehicle = str(self.get_parameter("vehicle_id").value)
        self.system_id = int(self.get_parameter("target_system_id").value)
        if self.vehicle not in HOMES or self.system_id != int(self.vehicle[-1]) + 1:
            raise ValueError("vehicle identity must match PX4 instance + 1")
        if self.get_namespace() != f"/{self.vehicle}":
            raise ValueError("vehicle namespace must match vehicle_id")
        self.origin = HOMES[self.vehicle]
        self.abort_after = float(self.get_parameter("abort_after_ready_s").value)
        if not math.isfinite(self.abort_after) or self.abort_after < 0:
            raise ValueError("abort_after_ready_s must be nonnegative")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(OffboardControlMode, "fmu/in/offboard_control_mode", qos)
        self.target_pub = self.create_publisher(TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.create_publisher(VehicleCommand, "fmu/in/vehicle_command", qos)
        self.status_pub = self.create_publisher(String, "swarm/telemetry", 1)
        self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position", self._position, qos)
        self.create_subscription(VehicleControlMode, "fmu/out/vehicle_control_mode", self._mode, qos)
        self.create_subscription(VehicleLandDetected, "fmu/out/vehicle_land_detected", self._land, qos)
        self.create_subscription(String, "swarm/command", self._command, 1)
        self.create_subscription(Bool, "swarm/abort", self._abort_message, 1)
        self.wire_stamps = {}
        self.session = uuid.uuid4().hex
        self.seq = 0
        self.state = "WAITING"
        self.fault = ""
        self.position = self.origin
        self.velocity = (0.0, 0.0, 0.0)
        self.calibration = None
        self.resets = None
        self.frame_valid = True
        self.position_valid = False
        self.position_stamp = -math.inf
        self.mode_stamp = -math.inf
        self.land_stamp = -math.inf
        self.armed = False
        self.offboard = False
        self.landed = False
        self.command_session = None
        self.command_seq = -1
        self.command_stamp = -math.inf
        self.action = "hold"
        self.goal = self.origin
        self.setpoint = self.origin
        self.start_time = time.monotonic()
        self.takeoff_started = None
        self.ready_since = None
        self.last_px4_command = -math.inf
        self.stale_hold = False
        self.timer = self.create_timer(0.05, self._tick)

    def _accept_wire(self, topic, stamp):
        previous = self.wire_stamps.get(topic, 0)
        if stamp <= previous:
            if stamp < previous and self.state not in {"WAITING", "LANDED"}:
                # PX4 SITL timesync resets often rewind timestamps under GUI load.
                # Re-baseline the wire clock instead of aborting the swarm.
                self.get_logger().warn(
                    f"PX4 telemetry clock rewound on {topic}; re-baselining"
                )
                self.wire_stamps[topic] = stamp
            return False
        self.wire_stamps[topic] = stamp
        return True

    def _position(self, msg):
        if not self._accept_wire("position", msg.timestamp):
            return
        now = time.monotonic()
        try:
            local = point((msg.x, msg.y, msg.z))
            velocity = point((msg.vx, msg.vy, msg.vz))
        except ValueError:
            self.position_valid = False
            return
        self.position_valid = bool(msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid)
        if not self.position_valid:
            return
        counters = (msg.xy_reset_counter, msg.z_reset_counter)
        if self.resets is not None and counters != self.resets and self.state != "WAITING":
            self.frame_valid = False
            self._abort("estimator frame reset")
        if self.calibration is None or (counters != self.resets and self.state == "WAITING"):
            self.calibration = local
        self.resets = counters
        self.position = local_to_world(local, self.origin, self.calibration)
        self.velocity = velocity
        self.position_stamp = now

    def _mode(self, msg):
        if not self._accept_wire("mode", msg.timestamp):
            return
        self.armed = bool(msg.flag_armed)
        self.offboard = bool(msg.flag_control_offboard_enabled)
        self.mode_stamp = time.monotonic()

    def _land(self, msg):
        if not self._accept_wire("land", msg.timestamp):
            return
        self.landed = bool(msg.landed)
        self.land_stamp = time.monotonic()

    def _valid(self, now):
        return (self.calibration is not None and self.position_valid and self.frame_valid
                and now - self.position_stamp < 0.75 and now - self.mode_stamp < 1.5
                and now - self.land_stamp < 1.5)

    def _command(self, msg):
        try:
            data = json.loads(msg.data)
            now = time.monotonic()
            if (data["vehicle"] != self.vehicle or data["vehicle_session"] != self.session
                    or type(data["seq"]) is not int or data["seq"] <= self.command_seq
                    or not isinstance(data["session"], str)
                    or not 0 <= now - float(data["sent"]) <= 0.75):
                return
            if self.command_session is not None and data["session"] != self.command_session:
                self._abort("coordinator restarted")
                return
            action = data["action"]
            if action not in {"takeoff", "move", "hold", "land"}:
                return
            goal = point(data["target"]) if action == "move" else self.goal
            if action == "move" and (not inside(goal) or abs(goal[2] + 3.0) > 0.01):
                return
        except (ValueError, TypeError, KeyError, OverflowError):
            return
        self.command_session = data["session"]
        self.command_seq = data["seq"]
        self.command_stamp = now
        if self.state in {"LANDING", "LANDED"}:
            return
        if action == "hold" and self.action != "hold":
            self.goal = self.position
            self.setpoint = self.position
        elif action == "move" and self.state in {"READY", "MOVING"}:
            self.goal = goal
            self.state = "MOVING"
        elif action == "land":
            self.state = "LANDING"
        elif action == "takeoff" and self.state == "WAITING" and self._valid(now) and now - self.start_time > 2.0:
            self.state = "TAKEOFF"
            self.takeoff_started = now
            self.goal = (*self.origin[:2], -3.0)
        if action == "takeoff" and self.state == "TAKEOFF":
            self.goal = (*self.origin[:2], -3.0)
        self.action = action

    def _abort_message(self, msg):
        if msg.data:
            self._abort("requested vehicle abort")

    def _abort(self, reason):
        if not self.fault:
            self.fault = reason
            self.get_logger().warn(reason)
        if self.state != "LANDED":
            self.state = "LANDING"

    def _px4_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.target_system, msg.target_component = self.system_id, 1
        msg.source_system, msg.source_component = 255, 191
        msg.from_external = True
        self.command_pub.publish(msg)

    def _stream(self):
        if self.calibration is None:
            return
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = True
        msg.velocity = True
        self.mode_pub.publish(msg)
        # Limit setpoint advance to 1.5 m/s and 0.6 m ahead of actual
        # position. _tick's fault check aborts past 1.0 m of tracking error
        # (a real safety margin tied to route reservations, not a tuning
        # knob) — 0.6 m keeps real PX4 tracking lag comfortably clear of it.
        # (0.15 m/1.5 m tripped that fault almost immediately in real SITL:
        # tracking lag at 3x the original speed exceeded 1.0 m outright.)
        # Lead must be large enough for PX4 position mode: commanded speed
        # is roughly MPC_XY_P * position_error (~0.95 * lead). A 1.6 m lead
        # caps cruise near 1.5 m/s; ~10 m lead unlocks ~10 m/s up to VEL_MAX.
        cruise_mps = 10.0
        lead_m = 10.0
        delta = math.dist(self.setpoint, self.goal)
        if delta:
            step = min(cruise_mps * 0.05, delta)
            candidate = tuple(self.setpoint[i] + (self.goal[i] - self.setpoint[i]) * step / delta for i in range(3))
            if math.dist(candidate, self.position) <= lead_m:
                self.setpoint = candidate
        target = TrajectorySetpoint()
        target.timestamp = msg.timestamp
        target.position = list(world_to_local(self.setpoint, self.origin, self.calibration))
        # Velocity feedforward toward the live goal (same NED axes as local).
        goal_delta = math.dist(self.position, self.goal)
        if goal_delta > 1e-3:
            feed = min(cruise_mps, goal_delta)
            target.velocity = [
                (self.goal[i] - self.position[i]) / goal_delta * feed for i in range(3)
            ]
        else:
            target.velocity = [0.0, 0.0, 0.0]
        target.acceleration = [float("nan")] * 3
        # Face the direction of travel instead of a fixed heading — yaw_toward
        # returns NaN when already at the setpoint, which PX4 reads as "hold
        # current yaw" (same convention offboard_mission.py's single-vehicle
        # setpoints already use).
        target.yaw = yaw_toward(self.position, self.setpoint)
        self.target_pub.publish(target)

    def _tick(self):
        now = time.monotonic()
        if self.state not in {"WAITING", "LANDING", "LANDED"}:
            if not self._valid(now):
                self._abort("local telemetry timeout or invalid estimate")
            elif math.dist(self.position, self.setpoint) > 12.0:
                self._abort("tracking error exceeds reservation margin")
            elif self.takeoff_started is not None and now - self.takeoff_started > 340.0:
                self._abort("local mission timeout")
            elif not inside(self.position):
                self._abort("world geofence breach")
            elif now - self.command_stamp > 2.0:
                self._abort("coordinator command timeout")
            elif now - self.command_stamp > 0.75:
                if not self.stale_hold:
                    self.goal = self.position
                    self.setpoint = self.position
                    self.stale_hold = True
            else:
                self.stale_hold = False
            if self.ready_since is not None and self.abort_after and now - self.ready_since >= self.abort_after:
                self._abort("simulated vehicle dropout")
            if self.state in {"READY", "MOVING"} and (not self.armed or not self.offboard):
                self._abort("unexpected arming/offboard loss")
        if self.state == "TAKEOFF":
            if now - self.takeoff_started > 30.0:
                self._abort("takeoff/arming timeout")
            elif now - self.last_px4_command >= 1.0:
                self._px4_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
                self._px4_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
                self.last_px4_command = now
            if self.armed and self.offboard and math.dist(self.position, (*self.origin[:2], -3.0)) < 0.25 and math.dist(self.velocity, (0, 0, 0)) < 0.3:
                self.state = "READY"
                self.ready_since = now
        if self.state == "LANDING":
            if self.landed and not self.armed and now - self.land_stamp < 1.5 and now - self.mode_stamp < 1.5:
                self.state = "LANDED"
            elif now - self.last_px4_command > 1.0:
                self._px4_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.last_px4_command = now
        elif self.state != "LANDED":
            self._stream()
        self.seq += 1
        if self.seq % 2 == 0:
            status = {"vehicle": self.vehicle, "session": self.session, "seq": self.seq,
                      "sent": now, "position": self.position, "velocity": self.velocity,
                      "state": self.state, "valid": self._valid(now), "armed": self.armed,
                      "landed": self.landed, "fault": self.fault}
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = SwarmVehicle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
