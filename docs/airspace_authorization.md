# Airspace authorization

Airspace is a release gate on the seeding supervisor, after altitude and weather and before traffic and simulated communications. The dispenser opens only when every required gate allows it. A chart that contains no blocking volume is still not an approval to fly, and `operational_approval` stays false.

The local NED fence in `mission_logic.Fence` remains the vehicle keep-in. This check is separate: it asks whether the release point, expressed as latitude, longitude, and AMSL, sits inside a charted volume.

## What the chart has to say

Each dataset names its source, provenance (`simulated`, `recorded`, or `live`), clock, observation time, receipt time, and the area it covers. Coverage is a center and a radius. A dataset with no coverage, or a release point outside that radius, is denied. An empty volume list inside a covering chart is the only "clear" result, and it is labeled as a simulation check.

Volumes are inclusive latitude/longitude boxes with an AMSL floor and ceiling:

| Kind | Release inside the box |
|---|---|
| Prohibited | Denied. A simulation authorization token does not waive it. |
| Restricted | Denied. |
| TFR | Denied while the effective window contains the evaluation time. A TFR with no window is denied rather than treated as inactive. |
| Controlled | Denied unless the policy carries an `authorization_id`. That id is a recorded simulation token, not an FAA grant. |

The release position is the profile's north and east, offset from the policy origin (simulation default 47 N, 122 W). The release altitude is the same AMSL used by the weather gate: local-origin AMSL plus release height. A volume whose ceiling is 100 m does not block a 4,572 m release. A shelf from 4,000 m to 5,000 m does.

`latest` reads a cache. `StaticAirspaceProvider` and `SimulatedAirspaceProvider` do not use the network. There is no live FAA chart client in this repository.

## Abort

If the release point becomes blocked during dispense, that step closes the valve, records the reason code, keeps earlier samples, and uses the existing return and abort. A later clear chart does not open the valve again.

Altitude is reported first, then weather, then airspace. A low estimator altitude still aborts for altitude when the chart is clear.

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/px4_offboard/test/test_airspace_authorization.py \
  src/px4_offboard/test/test_weather_authorization.py \
  src/px4_offboard/test/test_seeding_supervisor.py \
  src/px4_offboard/test/test_mission_orchestrator.py -q
```
