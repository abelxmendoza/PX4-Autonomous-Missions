# Weather authorization

Weather is a release constraint on the seeding supervisor. A passing check is a software decision about the observation in hand. It is not an approval to fly, and it does not certify that the air is safe.

The seeding controller still flies climb, hold, dispense, and return. `SeedingSupervisor` opens the dispenser only when the altitude frame, the weather authorizer, and the airspace authorizer all allow the release. An altitude denial is kept ahead of weather, and both are kept ahead of airspace. See [airspace authorization](airspace_authorization.md).

## Providers

| Provider | Role |
|---|---|
| `StaticWeatherProvider` | One fixed observation. Used by deterministic tests. |
| `SimulatedWeatherProvider` | Switches observation or failure as mission time advances. |
| `CachedWeatherProvider` | Holds the last stored fetch. `latest` does not contact the network. |
| `fetch_awc_metar` | Optional HTTP GET of one NOAA Aviation Weather Center METAR. The supervisor never calls it. |

`latest(now_s)` is the only read on the control path. It returns a cache. Live retrieval stays in `fetch_awc_metar`, which tests exercise with an injected transport rather than a socket.

A METAR parsed from the documented AWC JSON schema is a **surface** observation on the **Unix** clock. Wind arrives in knots and is stored in metres per second (`1852/3600`). Temperature stays degrees Celsius. Sea-level pressure, or the altimeter setting if sea-level pressure is absent, arrives in hectopascals and is stored in pascals. Station elevation is metres. `obsTime` is Unix seconds. Relative humidity is left missing because that product does not report it. A fixture defaults to provenance `recorded`. Only the live client marks a report `live`. Simulation observations are provenance `simulated`.

## Units and time

Internal units are metres per second, degrees Celsius, pascals, metres, and seconds. Missing values stay `None`. They are not replaced with zero, and a failed observation is not replaced with favorable weather.

Mission steps use elapsed mission time. A Unix observation is not subtracted from that clock. It is usable only when `WeatherPolicy.unix_origin_s` names the Unix instant of mission time zero. Otherwise the reason is `clock_domain_mismatch`.

Age is evaluation time minus observation time on the mission clock. An observation newer than `max_future_s` is `future_observation`. An observation older than `max_age_s` is `stale_observation`. Both the observation time and the receipt time have to be present.

## Authorization rules

The first failing check wins:

1. Provider failure or no observation.
2. Clock domain.
3. Missing, future, or stale timestamps.
4. Non-finite or physically implausible numbers.
5. Missing coordinates, or a station farther than `radius_m` from the mission area.
6. Release-point AMSL. That altitude is the local origin's AMSL (`vehicle_local_position.ref_alt`) plus the release height. The vehicle's current GPS altitude is not used while it is still climbing.
7. Applicability. A surface observation covers the release only when the release AMSL is within `surface_layer_m` of the station elevation and at or below `surface_product_ceiling_amsl_m` (simulation default 300 m). A 15,000 ft (4572 m) release is above that ceiling. An `altitude` observation covers the release only inside its own altitude span. A forecast product is `unsupported_product`.
8. Required measurements: sustained wind, gust, and temperature. Pressure and humidity are required only when a limit is configured for them.
9. Simulation limits. The defaults are 12 m/s sustained, 18 m/s gust, and −20 °C to 40 °C. Those numbers are scenario knobs, not a flight manual.

`operational_approval` in the evidence is always false.

## Abort

If weather becomes unauthorized during dispense, that supervisor step closes the valve, records the reason code, keeps earlier release samples, and uses the mission's existing return and abort. A later favorable observation does not open the valve again. The mission reason is already set.

## Configuration

```python
from px4_offboard.weather import WeatherAuthorizer, simulation_observation, simulation_policy
from px4_offboard.weather_provider import StaticWeatherProvider

authorizer = WeatherAuthorizer(
    StaticWeatherProvider(simulation_observation()),
    simulation_policy(),  # area 47 N, 122 W; radius 50 km; surface ceiling 300 m AMSL
)
```

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/px4_offboard/test/test_weather_authorization.py \
  src/px4_offboard/test/test_weather_provider.py \
  src/px4_offboard/test/test_weather_scenarios.py \
  src/px4_offboard/test/test_altitude_reference.py \
  src/px4_offboard/test/test_seeding_mission.py \
  src/px4_offboard/test/test_seeding_supervisor.py \
  src/px4_offboard/test/test_precision_land.py \
  src/px4_offboard/test/test_mission_orchestrator.py -q
```

The local run of that command was 79 passed. No PX4 SITL flight was run for the weather gate. The live AWC endpoint was not called.

## Limitations

- No ROS 2 weather topic is subscribed. The control path is `SeedingSupervisor.step`.
- Surface METARs are not treated as a profile of the air at release altitude.
- Gust left out of a report is missing, so the release is denied rather than treated as calm.
- `fetch_awc_metar` against aviationweather.gov has not been executed from this workspace.
