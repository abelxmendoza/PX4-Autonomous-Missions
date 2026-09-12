// Render the SDF visual hierarchy in ENU, then rotate it into Three's Y-up frame.
function buildGazeboEnvironment(THREE, scene, data) {
  const rgb = values => new THREE.Color().setRGB(...values.slice(0, 3));
  scene.background = rgb(data.background);
  scene.fog = new THREE.Fog(rgb(data.fog.color), data.fog.start, data.fog.end);
  scene.add(new THREE.AmbientLight(rgb(data.ambient), 0.8));
  const sun = new THREE.DirectionalLight(0xffffff, 0.85);
  sun.position.set(50, 90, 10); // Opposite SDF sun direction (-0.5, 0.1, -0.9).
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, {left: -70, right: 70, top: 70, bottom: -70, far: 220});
  sun.shadow.bias = -0.0005;
  scene.add(sun);
  const root = new THREE.Group();
  root.name = 'gazebo-environment';
  root.rotation.x = -Math.PI / 2;
  function pose(object, values) {
    object.position.set(...values.slice(0, 3));
    object.rotation.set(values[3], values[4], values[5], 'ZYX');
  }
  for (const model of data.models) {
    // Obstacles retain recording-specific dimensions; the interactive denied
    // volume is rendered by the replay's existing telemetry layer.
    if (model.name.startsWith('obstacle_') || model.name === 'gps_denied_zone') continue;
    const group = new THREE.Group();
    group.name = model.name;
    pose(group, model.pose);
    for (const link of model.links) {
      const parent = new THREE.Group();
      pose(parent, link.pose);
      for (const visual of link.visuals) {
        const spec = visual.geometry;
        let geometry;
        if (spec.type === 'box') geometry = new THREE.BoxGeometry(...spec.size);
        if (spec.type === 'sphere') geometry = new THREE.SphereGeometry(spec.radius[0], 24, 16);
        if (spec.type === 'cylinder') {
          geometry = new THREE.CylinderGeometry(spec.radius[0], spec.radius[0], spec.length[0], 48);
          geometry.rotateX(Math.PI / 2);
        }
        if (spec.type === 'plane') {
          geometry = new THREE.PlaneGeometry(...spec.size);
          const normal = new THREE.Vector3(...spec.normal).normalize();
          geometry.applyMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(
            new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal)));
        }
        const opacity = visual.color[3] * (1 - visual.transparency);
        const material = new THREE.MeshStandardMaterial({
          color: rgb(visual.color), emissive: rgb(visual.emissive), roughness: 0.85,
          transparent: opacity < 1, opacity, side: THREE.DoubleSide,
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.name = visual.name;
        pose(mesh, visual.pose);
        mesh.castShadow = visual.castShadows;
        mesh.receiveShadow = true;
        parent.add(mesh);
      }
      group.add(parent);
    }
    root.add(group);
  }
  scene.add(root);
  return root;
}
if (typeof module !== 'undefined') module.exports = { buildGazeboEnvironment };
