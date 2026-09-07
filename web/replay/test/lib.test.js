import { describe, it, expect } from 'vitest';
import { toWorld, parseCsv, COURSE_PADS, GPS_DENIED_ZONE, gpsDeniedWorldBox, gpsDeniedLabelPos } from '../lib.js';

const FULL_HEADER =
  'time,state,north,east,down,tgt_n,tgt_e,tgt_d,obstacle,obstacle_source,sensor_fresh,' +
  'lidar_front_m,lidar_left_m,lidar_right_m,mapped_clearance_m,nominal_n,nominal_e,nominal_d,' +
  'wp_index,geocage,geofence,inside,caged,roll_deg,pitch_deg,yaw_deg,vn,ve,vd,' +
  'executive_mode,battery_frac,link_quality,propellant_s,' +
  'in_gps_denied_zone,gps_xy_valid,gps_injected_deny,loc_source,loc_event,dead_reckoning,eph_m,' +
  'raw_gps_healthy,vio_stream_healthy,ev_pos_fused,ev_vel_fused,gnss_pos_fused,gnss_vel_fused,gps_failure_active';

function fullRow(overrides = {}) {
  const base = {
    time: '10:00:00.0', state: 'MOVE', north: '5.0', east: '-3.0', down: '-4.0',
    tgt_n: '10.0', tgt_e: '0.0', tgt_d: '-4.0', obstacle: 'front',
    obstacle_source: 'sensor_only', sensor_fresh: '1',
    lidar_front_m: '2.5', lidar_left_m: '-1.0', lidar_right_m: '6.0',
    mapped_clearance_m: '1.8', nominal_n: '10.0', nominal_e: '0.0', nominal_d: '-4.0',
    wp_index: '2', geocage: '1', geofence: '1', inside: '1', caged: '0',
    roll_deg: '1.2', pitch_deg: '3.4', yaw_deg: '-56.7',
    vn: '0.5', ve: '-0.2', vd: '0.1',
    executive_mode: 'NOMINAL', battery_frac: '0.9', link_quality: '1.0', propellant_s: '150.0',
    in_gps_denied_zone: '0', gps_xy_valid: '1', gps_injected_deny: '0',
    loc_source: 'GPS', loc_event: '', dead_reckoning: '0', eph_m: '0.8',
    raw_gps_healthy: '1', vio_stream_healthy: '1', ev_pos_fused: '0', ev_vel_fused: '0',
    gnss_pos_fused: '1', gnss_vel_fused: '1', gps_failure_active: '0',
  };
  return Object.assign(base, overrides);
}

function csv(rows) {
  const header = FULL_HEADER.split(',');
  const lines = [FULL_HEADER, ...rows.map(r => header.map(h => r[h]).join(','))];
  return lines.join('\n');
}

describe('toWorld', () => {
  it('maps NED to the world frame (x=east, y=altitude, z=-north)', () => {
    const v = toWorld(10, 3, -5); // north=10, east=3, down=-5 (5m altitude)
    expect(v.x).toBe(3);
    expect(v.y).toBe(5);
    expect(v.z).toBe(-10);
  });

  it('parses operational GPS-denied fusion evidence', () => {
    const rows = parseCsv(csv([fullRow({
      raw_gps_healthy: '0', gps_failure_active: '1', vio_stream_healthy: '1',
      ev_pos_fused: '1', ev_vel_fused: '1', gnss_pos_fused: '0', gnss_vel_fused: '0',
    })]));
    expect(rows[0].gps_failure_active).toBe(true);
    expect(rows[0].raw_gps_healthy).toBe(false);
    expect(rows[0].vio_stream_healthy).toBe(true);
    expect(rows[0].ev_pos_fused).toBe(true);
    expect(rows[0].gnss_pos_fused).toBe(false);
  });

  it('places the GPS-denied prism over the mid-course corridor', () => {
    expect(GPS_DENIED_ZONE.northMin).toBe(15.5);
    expect(GPS_DENIED_ZONE.northMax).toBe(31.5);
    expect(GPS_DENIED_ZONE.eastMin).toBe(-8);
    expect(GPS_DENIED_ZONE.eastMax).toBe(8);
    const box = gpsDeniedWorldBox();
    expect(box.north).toBeCloseTo(23.5);
    expect(box.east).toBeCloseTo(0);
    expect(box.sizeE).toBeCloseTo(16);
    expect(box.sizeN).toBeCloseTo(16);
    expect(box.height).toBeCloseTo(12);
    const world = toWorld(box.north, box.east, -box.height / 2);
    expect(world.x).toBeCloseTo(0);
    expect(world.y).toBeCloseTo(6);
    expect(world.z).toBeCloseTo(-23.5);
    const label = gpsDeniedLabelPos();
    expect(label.north).toBeCloseTo(23.5);
    expect(label.alt).toBeGreaterThan(box.height);
  });

  it('places the landing pad 50 m north of the launch pad', () => {
    expect(COURSE_PADS.launch.north).toBe(0);
    expect(COURSE_PADS.landing.north).toBe(50);
    const launch = toWorld(COURSE_PADS.launch.north, COURSE_PADS.launch.east, 0);
    const land = toWorld(COURSE_PADS.landing.north, COURSE_PADS.landing.east, 0);
    expect(launch.z).toBeCloseTo(0);
    expect(land.z).toBeCloseTo(-50);
    expect(land.x).toBeCloseTo(0);
  });

  it('places the origin at the world origin', () => {
    const v = toWorld(0, 0, 0);
    // toBeCloseTo (not toBe) because -down of 0 is JS's -0, which is
    // numerically equal to 0 but fails Object.is-based equality.
    expect(v.x).toBeCloseTo(0); expect(v.y).toBeCloseTo(0); expect(v.z).toBeCloseTo(0);
  });

  it('returns a real THREE.Vector3 (has vector methods, not a plain object)', () => {
    const v = toWorld(1, 1, -1);
    expect(typeof v.clone).toBe('function');
    expect(v.clone().equals(v)).toBe(true);
  });
});

describe('parseCsv — well-formed input', () => {
  it('parses a full modern row with every column', () => {
    const rows = parseCsv(csv([fullRow()]));
    expect(rows).toHaveLength(1);
    const r = rows[0];
    expect(r.north).toBe(5.0);
    expect(r.east).toBe(-3.0);
    expect(r.down).toBe(-4.0);
    expect(r.wp_index).toBe(2);
    expect(r.geocage).toBe(true);
    expect(r.geofence).toBe(true);
    expect(r.sensor_fresh).toBe(true);
    expect(r.yaw_deg).toBeCloseTo(-56.7);
  });

  it('parses mission-executive telemetry as numbers, not strings', () => {
    const rows = parseCsv(csv([fullRow({ battery_frac: '0.42', link_quality: '0.87', propellant_s: '63.5' })]));
    expect(rows[0].battery_frac).toBeCloseTo(0.42);
    expect(rows[0].link_quality).toBeCloseTo(0.87);
    expect(rows[0].propellant_s).toBeCloseTo(63.5);
  });

  it('parses GPS-denied Pass 1 scaffolding columns', () => {
    const rows = parseCsv(csv([fullRow({
      in_gps_denied_zone: '1',
      gps_injected_deny: '1',
      gps_xy_valid: '0',
      loc_source: 'GPS_DENIED_INJECTED',
      loc_event: 'LOC_FAILSAFE',
      eph_m: '3.2',
    })]));
    expect(rows[0].in_gps_denied_zone).toBe(true);
    expect(rows[0].gps_injected_deny).toBe(true);
    expect(rows[0].gps_xy_valid).toBe(false);
    expect(rows[0].loc_source).toBe('GPS_DENIED_INJECTED');
    expect(rows[0].loc_event).toBe('LOC_FAILSAFE');
    expect(rows[0].eph_m).toBeCloseTo(3.2);
  });

  it('parses geocage/geofence booleans strictly from "1"', () => {
    const rows = parseCsv(csv([fullRow({ geocage: '0', geofence: '1' })]));
    expect(rows[0].geocage).toBe(false);
    expect(rows[0].geofence).toBe(true);
  });

  it('handles a single data row', () => {
    const rows = parseCsv(csv([fullRow()]));
    expect(rows).toHaveLength(1);
  });

  it('handles zero data rows (header only)', () => {
    const rows = parseCsv(FULL_HEADER + '\n');
    expect(rows).toEqual([]);
  });

  it('handles CRLF line endings without corrupting the last column', () => {
    const text = csv([fullRow()]).split('\n').join('\r\n');
    const rows = parseCsv(text);
    expect(rows).toHaveLength(1);
    expect(rows[0].propellant_s).toBe(150.0);
  });
});

describe('parseCsv — backward compatibility with older schemas', () => {
  it('leaves attitude/lidar fields as NaN when those columns are absent (legacy log)', () => {
    const legacyHeader = 'time,state,north,east,down,tgt_n,tgt_e,tgt_d,obstacle,wp_index,geocage,geofence,inside,caged';
    const legacyRow = '10:00:00,MOVE,5.0,-3.0,-4.0,10.0,0.0,-4.0,,1,1,1,1,0';
    const rows = parseCsv(legacyHeader + '\n' + legacyRow);
    expect(rows).toHaveLength(1);
    expect(rows[0].north).toBe(5.0);
    expect(Number.isFinite(rows[0].yaw_deg)).toBe(false);
    expect(Number.isFinite(rows[0].lidar_front_m)).toBe(false);
  });
});

describe('parseCsv — malformed input is rejected, not silently corrupted', () => {
  it('skips a row with fewer columns than the header', () => {
    const text = FULL_HEADER + '\n' + '10:00:00,MOVE,5.0'; // truncated row
    const rows = parseCsv(text);
    expect(rows).toEqual([]);
  });

  it('skips a row whose north/east/down field is empty (would otherwise be NaN)', () => {
    const rows = parseCsv(csv([fullRow({ north: '' }), fullRow({ east: 'garbage' })]));
    expect(rows).toEqual([]);
  });

  it('keeps valid rows and drops only the malformed ones in a mixed file', () => {
    const rows = parseCsv(csv([fullRow({ north: '1.0' }), fullRow({ north: '' }), fullRow({ north: '3.0' })]));
    expect(rows.map(r => r.north)).toEqual([1.0, 3.0]);
  });

  it('does not throw on a completely empty string', () => {
    expect(() => parseCsv('')).not.toThrow();
  });
});
