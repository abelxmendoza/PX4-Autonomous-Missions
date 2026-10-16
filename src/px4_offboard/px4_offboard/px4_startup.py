"""Process-specific PX4 SITL startup checks.

A log file that already contains the success line is not evidence that the
process we just launched is ready. The success line counts only when the log
was written at or after the launch time and the recorded process is alive.
"""

from __future__ import annotations

SUCCESS_LINE = "Startup script returned successfully"

NOT_STARTED = "NOT_STARTED"
STARTING = "STARTING"
CONNECTED = "CONNECTED"
READY = "READY"
FAILED = "FAILED"


def _fresh(log_mtime_s: float | None, launched_at_s: float) -> bool:
    return log_mtime_s is not None and log_mtime_s >= launched_at_s - 1.0


def process_gate(
    *,
    pid_alive: bool,
    log_text: str,
    log_mtime_s: float | None,
    launched_at_s: float,
) -> str:
    """State of the PX4 process we launched, before MAVSDK is considered.

    READY means that process is alive and its log, written after launch,
    contains the PX4 startup success line.
    """
    fresh = _fresh(log_mtime_s, launched_at_s)
    success = SUCCESS_LINE in log_text and fresh
    if not pid_alive:
        if success or (log_text.strip() and fresh):
            return FAILED
        return NOT_STARTED
    if not success:
        return STARTING
    return READY


def assess(
    *,
    pid_alive: bool,
    log_text: str,
    log_mtime_s: float | None,
    launched_at_s: float,
    mavlink_connected: bool,
    telemetry_fresh: bool,
) -> str:
    """Full startup state, including the MAVSDK link and a fresh telemetry sample."""
    phase = process_gate(
        pid_alive=pid_alive,
        log_text=log_text,
        log_mtime_s=log_mtime_s,
        launched_at_s=launched_at_s,
    )
    if phase in (NOT_STARTED, STARTING, FAILED):
        return phase
    if not mavlink_connected:
        return STARTING
    if not telemetry_fresh:
        return CONNECTED
    return READY
