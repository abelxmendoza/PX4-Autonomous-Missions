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

// Visual-only pads in worlds/obstacle_world.sdf. Launch is at spawn;
// landing is the far end of the course (NED north = 50 m).
const COURSE_PADS = {
  launch:  { east: 0, north: 0,  radius: 2.5 },
  landing: { east: 0, north: 50, radius: 2.5 },
};

// Mirrors worlds/obstacle_world.sdf gps_denied_zone and
// localization_logic.DEFAULT_GPS_DENIED_ZONE / offboard_mission.yaml.
const GPS_DENIED_ZONE = {
  eastMin: -8.0,
  eastMax: 8.0,
  northMin: 15.5,
  northMax: 31.5,
  altMin: 0.0,
  altMax: 12.0,
};

const COURSE_SURFACE = { east: 0, north: 25, sizeE: 30, sizeN: 60 };

const COURSE_BEACONS = [
  { east: -16, north: 10 }, { east: 16, north: 10 },
  { east: -16, north: 25 }, { east: 16, north: 25 },
  { east: -16, north: 40 }, { east: 16, north: 40 },
];

// Painted boxes + roof slabs from worlds/obstacle_world.sdf (diffuse RGB 0–1).
const COURSE_OBSTACLES = [
  { name: 'OB1', east: -6, north: 10, sizeE: 3, sizeN: 3, h: 11.5,
    color: [0.85, 0.15, 0.15],
    roof: { sizeE: 3.18, sizeN: 3.18, h: 0.12, color: [0.08, 0.08, 0.08] } },
  { name: 'OB2', east: 10, north: 10, sizeE: 3, sizeN: 3, h: 6,
    color: [0.90, 0.50, 0.05],
    roof: { sizeE: 4.18, sizeN: 3.18, h: 0.12, color: [0.98, 0.98, 0.98] } },
  { name: 'OB3', east: -8, north: 24, sizeE: 5, sizeN: 3, h: 4,
    color: [0.15, 0.65, 0.15],
    roof: { sizeE: 2.18, sizeN: 2.18, h: 0.12, color: [0.98, 0.98, 0.98] } },
  { name: 'OB4', east: 6, north: 24, sizeE: 3, sizeN: 2, h: 5,
    color: [0.15, 0.15, 0.85],
    roof: { sizeE: 2.18, sizeN: 2.18, h: 0.12, color: [0.98, 0.98, 0.98] } },
  { name: 'OB5', east: 0, north: 38, sizeE: 5, sizeN: 3, h: 11.5,
    color: [0.65, 0.00, 0.75],
    roof: { sizeE: 5.18, sizeN: 3.18, h: 0.12, color: [0.08, 0.08, 0.08] } },
];

function rgb01ToHex(rgb) {
  const to = (x) => Math.max(0, Math.min(255, Math.round(x * 255)));
  return (to(rgb[0]) << 16) | (to(rgb[1]) << 8) | to(rgb[2]);
}

function gpsDeniedWorldBox(zone) {
  const z = zone || GPS_DENIED_ZONE;
  return {
    east: (z.eastMin + z.eastMax) / 2,
    north: (z.northMin + z.northMax) / 2,
    sizeE: z.eastMax - z.eastMin,
    sizeN: z.northMax - z.northMin,
    height: z.altMax - z.altMin,
  };
}

function gpsDeniedLabelPos(zone) {
  const box = gpsDeniedWorldBox(zone);
  return { east: box.east, north: box.north, alt: box.height + 1.6 };
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
    // GPS-denied and estimator-fusion evidence (absent on legacy logs).
    o.in_gps_denied_zone = o.in_gps_denied_zone === '1';
    o.gps_injected_deny = o.gps_injected_deny === '1';
    o.gps_xy_valid = o.gps_xy_valid === undefined || o.gps_xy_valid === ''
      ? null
      : o.gps_xy_valid === '1';
    o.dead_reckoning = o.dead_reckoning === '1';
    o.eph_m = parseFloat(o.eph_m);
    o.loc_source = o.loc_source || '';
    o.loc_event = o.loc_event || '';
    for (const key of ['raw_gps_healthy', 'vio_stream_healthy', 'ev_pos_fused',
      'ev_vel_fused', 'gnss_pos_fused', 'gnss_vel_fused', 'gps_failure_active']) {
      o[key] = o[key] === '1';
    }
    out.push(o);
  }
  if (skipped > 0) {
    console.warn(`parseCsv: skipped ${skipped} malformed row(s)`);
  }
  return out;
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    toWorld, parseCsv, COURSE_PADS, GPS_DENIED_ZONE,
    COURSE_SURFACE, COURSE_BEACONS, COURSE_OBSTACLES, rgb01ToHex,
    gpsDeniedWorldBox, gpsDeniedLabelPos,
  };
}
