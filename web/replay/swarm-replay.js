// Swarm evidence remains in its recorded shared-world frame; never offset a
// flight to make it appear to have launched from a different home.
function parseSwarmReplay(text) {
  const data = JSON.parse(text);
  if (data.schema !== 1 || !Array.isArray(data.frames) || data.frames.length < 2) throw new Error('Invalid swarm recording');
  let previous = -1;
  for (const f of data.frames) {
    if (!Number.isFinite(f.time) || f.time <= previous) throw new Error('Invalid swarm timeline');
    previous = f.time;
    for (const id of ['px4_1', 'px4_2']) {
      const v = f.vehicles[id];
      if (!v || v.position.length !== 3 || !v.position.every(Number.isFinite)) throw new Error('Missing vehicle position');
    }
  }
  return data;
}

function createSwarmReplay(THREE, scene, model, data, resolution) {
  const models = [model, model.clone(true)];
  for (const child of [...models[1].children]) if (!child.isMesh) models[1].remove(child);
  scene.add(models[1]);
  const colors = [0x69f7cd, 0xff5da2];
  const lines = [], counts = [], labels = [];
  const materials = [];
  models[1].traverse(obj => {
    if (obj.isMesh) {
      obj.material = obj.material.clone();
      obj.material.color.setHex(colors[1]);
      materials.push(obj.material);
    }
  });
  for (let k = 0; k < 2; k++) {
    const id = `px4_${k + 1}`, vertices = [], frameCounts = [];
    const label = makeLabelSprite(`DRONE ${k + 1}`);
    scene.add(label); labels.push(label);
    for (let i = 0; i < data.frames.length; i++) {
      const v = data.frames[i].vehicles[id];
      const prev = data.frames[Math.max(0, i - 1)].vehicles[id];
      if (i && v.valid && prev.valid) {
        for (const p of [prev.position, v.position]) vertices.push(p[1], -p[2], -p[0]);
      }
      frameCounts.push(vertices.length / 6);
    }
    const geometry = new THREE.LineSegmentsGeometry();
    geometry.setPositions(vertices.length ? vertices : [0, 0, 0, 0, 0, 0]);
    const material = new THREE.LineMaterial({color: colors[k], linewidth: 5, resolution});
    const line = new THREE.LineSegments2(geometry, material);
    scene.add(line); lines.push(line); counts.push(frameCounts);
  }
  return {
    materials: lines.map(l => l.material),
    update(index, cameraMode) {
      const f = data.frames[index];
      models.forEach((m, k) => {
        const v = f.vehicles[`px4_${k + 1}`], p = v.position;
        m.position.set(p[1], -p[2], -p[0]);
        const prev = data.frames[Math.max(0, index - 1)].vehicles[`px4_${k + 1}`].position;
        if (Math.hypot(p[0] - prev[0], p[1] - prev[1]) > .001) m.rotation.set(0, Math.atan2(p[1] - prev[1], -(p[0] - prev[0])), 0);
        m.visible = v.valid && !(k === 0 && cameraMode);
        labels[k].visible = v.valid;
        labels[k].position.copy(m.position).add(new THREE.Vector3(0, 1, 0));
        lines[k].geometry.instanceCount = counts[k][index];
      });
    },
    dispose() {
      scene.remove(models[1]); materials.forEach(m => m.dispose());
      for (const l of lines) { scene.remove(l); l.geometry.dispose(); l.material.dispose(); }
      for (const l of labels) { scene.remove(l); l.material.map.dispose(); l.material.dispose(); }
    }
  };
}
if (typeof module !== 'undefined') module.exports = {parseSwarmReplay};
