// Pure, THREE-dependent-but-side-effect-free helpers shared between the
// flight replay page (loaded as a plain <script>, defining globals) and its
// test suite (imported as a CommonJS module). Kept in one place so the
// tested code is the exact code the page runs — not a parallel
// reimplementation that could silently drift out of sync.
//
// In the browser, THREE is a global provided by the CDN <script> tag loaded
// before this file. In tests, it's the real `three` npm package assigned to
// globalThis.THREE by the test setup — same library, not a stub.

// world <- NED :  x=east, y=altitude(=-down), z=-north
function toWorld(n, e, d) {
  return new THREE.Vector3(e, -d, -n);
}

function parseCsv(text) {
  const lines = text.trim().split(/\r?\n/);
  const header = lines[0].split(',');
  const out = [];
  let skipped = 0;
  for (let i = 1; i < lines.length; i++) {
    const cols = lines[i].split(',');
    if (cols.length < header.length) { skipped++; continue; }
    const o = {};
    header.forEach((h, idx) => { o[h] = cols[idx]; });
    o.north = parseFloat(o.north); o.east = parseFloat(o.east); o.down = parseFloat(o.down);
    // Core position fields must be real numbers — a malformed row here would
    // otherwise inject NaN into the 3D scene (drone/trail silently render
    // off-screen or frozen with no visible error). Skip it instead.
    if (!Number.isFinite(o.north) || !Number.isFinite(o.east) || !Number.isFinite(o.down)) {
      skipped++;
      continue;
    }
    o.tgt_n = parseFloat(o.tgt_n); o.tgt_e = parseFloat(o.tgt_e); o.tgt_d = parseFloat(o.tgt_d);
    o.wp_index = parseInt(o.wp_index, 10);
    o.geocage = o.geocage === '1'; o.geofence = o.geofence === '1';
    // Real PX4 attitude/velocity — absent (NaN) on older recordings made
    // before offboard_mission.py logged them; callers fall back accordingly.
    o.roll_deg = parseFloat(o.roll_deg); o.pitch_deg = parseFloat(o.pitch_deg);
    o.yaw_deg = parseFloat(o.yaw_deg);
    o.vn = parseFloat(o.vn); o.ve = parseFloat(o.ve); o.vd = parseFloat(o.vd);
    o.sensor_fresh = o.sensor_fresh === '1';
    o.lidar_front_m = parseFloat(o.lidar_front_m);
    o.lidar_left_m = parseFloat(o.lidar_left_m);
    o.lidar_right_m = parseFloat(o.lidar_right_m);
    o.mapped_clearance_m = parseFloat(o.mapped_clearance_m);
    o.nominal_n = parseFloat(o.nominal_n); o.nominal_e = parseFloat(o.nominal_e);
    o.nominal_d = parseFloat(o.nominal_d);
    // Mission-executive telemetry — present in the schema but not yet
    // rendered anywhere in the viewer; parsed here so numeric HUD fields
    // built on these later don't inherit a string-vs-number bug.
    o.battery_frac = parseFloat(o.battery_frac);
    o.link_quality = parseFloat(o.link_quality);
    o.propellant_s = parseFloat(o.propellant_s);
    // GPS-denied Pass 1 scaffolding columns (absent on legacy logs).
    o.in_gps_denied_zone = o.in_gps_denied_zone === '1';
    o.gps_injected_deny = o.gps_injected_deny === '1';
    o.gps_xy_valid = o.gps_xy_valid === undefined || o.gps_xy_valid === ''
      ? null
      : o.gps_xy_valid === '1';
    o.dead_reckoning = o.dead_reckoning === '1';
    o.eph_m = parseFloat(o.eph_m);
    o.loc_source = o.loc_source || '';
    o.loc_event = o.loc_event || '';
    out.push(o);
  }
  if (skipped > 0) {
    console.warn(`parseCsv: skipped ${skipped} malformed row(s)`);
  }
  return out;
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { toWorld, parseCsv };
}
