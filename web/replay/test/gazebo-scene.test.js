import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import world from '../gazebo-world.js';
import { buildGazeboEnvironment } from '../gazebo-scene.js';

describe('Gazebo environment parity', () => {
  it('renders every scenery visual from the exported SDF', () => {
    const scene = new THREE.Scene();
    const root = buildGazeboEnvironment(THREE, scene, world);
    const expected = world.models.filter(m => !m.name.startsWith('obstacle_') && m.name !== 'gps_denied_zone');
    let meshes = 0;
    root.traverse(object => { if (object.isMesh) meshes++; });
    expect(meshes).toBe(expected.reduce((n, m) => n + m.links.reduce((k, l) => k + l.visuals.length, 0), 0));
    expect(scene.background.r).toBeCloseTo(world.background[0]);
    expect(scene.fog.near).toBe(world.fog.start);
    expect(root.getObjectByName('landscape_landmarks')).toBeTruthy();
  });
  it('converts SDF ENU markings into replay Y-up coordinates', () => {
    const scene = new THREE.Scene();
    const root = buildGazeboEnvironment(THREE, scene, world);
    scene.updateMatrixWorld(true);
    const position = root.getObjectByName('course_surface').getWorldPosition(new THREE.Vector3());
    expect(position.x).toBeCloseTo(0);
    expect(position.y).toBeCloseTo(0.012);
    expect(position.z).toBeCloseTo(-25);
  });
});
