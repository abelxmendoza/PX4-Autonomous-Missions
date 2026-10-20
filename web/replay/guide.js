// Guided tour for a recorded replay. Beats are times in the recording, not a
// separate animation. The page calls update() with seconds since the first sample.

function beatIndex(beats, flightSeconds) {
  let idx = -1;
  for (let i = 0; i < beats.length; i++) {
    if (beats[i].at <= flightSeconds + 1e-6) idx = i;
    else break;
  }
  return idx;
}

function searchGuideBeats(trace) {
  const t0 = trace.frames[0].t;
  const rel = t => t - t0;
  const conf = [...trace.confirmations].sort((a, b) => a.t - b.t);
  const beats = [{
    at: 0,
    title: 'A planned sweep',
    text: 'One drone flies fixed lanes. The downward camera confirms the markers. The world file is the score, not the guidance.',
  }];
  if (conf.length) {
    beats.push({
      at: rel(conf[0].t),
      title: 'The first marker is confirmed',
      text: `Target #${conf[0].id} is confirmed from the camera. The ring is that estimate.`,
    });
  }
  if (conf.length > 2) {
    const mid = conf[Math.floor(conf.length / 2)];
    beats.push({
      at: rel(mid.t),
      title: `${Math.floor(conf.length / 2) + 1} of ${conf.length}`,
      text: 'The sweep is still going. Each confirmation is a camera fix. Ground truth stays hidden unless you turn it on.',
    });
  }
  if (conf.length > 1) {
    const last = conf[conf.length - 1];
    beats.push({
      at: rel(last.t),
      title: `All ${conf.length} targets are confirmed`,
      text: 'The last marker in this recording has been confirmed.',
    });
  }
  const end = rel(trace.frames[trace.frames.length - 1].t);
  const lastAt = beats[beats.length - 1].at;
  if (end - lastAt > 2) {
    beats.push({
      at: end,
      title: 'The sweep is over',
      text: `The recording ends with ${conf.length} confirmed targets.`,
    });
  }
  return beats;
}

function seedingGuideBeats(trace) {
  const t0 = trace.frames[0].t;
  const kg = trace.release.amount_kg;
  const err = trace.touchdown_horizontal_error_m;
  const marker = trace.marker_id;
  const touch = Number.isFinite(err) && marker != null
    ? ` Touchdown is ${err.toFixed(3)} m from marker ${marker}, using PX4's position. The marker survey did not guide the landing.`
    : '';
  const beats = [{at: 0, title: 'Taking off',
    text: 'PX4 handles takeoff. The simulated dispenser stays closed until the mission reaches its release point and every gate allows it.'}];
  for (const s of trace.states) {
    const at = Math.max(0, s.t - t0);
    if (s.to === 'preflight' || s.to === 'takeoff') continue;
    if (s.to === 'transit_to_release') {
      beats.push({at, title: 'Climbing to the release point', text: 'The aircraft is airborne and the orchestrator is flying it to the release station. The valve is closed.'});
    } else if (s.to === 'hold_for_release') {
      beats.push({at, title: 'Holding for the release decision', text: 'Seed opens only while the aircraft is holding, and only if altitude, weather, airspace, traffic, and simulated communications all allow it.'});
    } else if (s.to === 'dispensing') {
      beats.push({at, title: 'The valve is open', text: `Every gate allowed the release. This recording dispenses ${kg.toFixed(3)} kg, then closes the valve.`});
    } else if (s.to === 'return_to_recovery') {
      beats.push(s.reason === 'release complete'
        ? {at, title: 'Release finished', text: 'The valve is closed. The aircraft turns back toward the recovery site.'}
        : {at, title: 'Release denied', text: `The release stops here: ${s.reason}. The valve stays closed and the aircraft turns back. These inputs are simulated.`});
    } else if (s.to === 'precision_landing') {
      beats.push({at, title: 'Handing off to the marker', text: 'Staging conditions are met. The precision-landing controller owns the setpoints. The 5 s marker timeout stays as it is.'});
    } else if (s.to === 'landed') {
      beats.push({at, title: 'Landed', text: 'The land detector reports the aircraft down after pad contact.' + touch});
    } else if (s.to === 'aborted') {
      beats.push({at, title: 'Recovered', text: 'The release was denied and the aircraft still landed.' + touch});
    } else if (s.to === 'failed') {
      const why = s.reason || (trace.outcomes && trace.outcomes.recovery_reason_code) || 'the landing failed';
      beats.push({at, title: 'Recovery failed', text: `The landing stops: ${why}. The aircraft holds, and the dispenser stays closed.`});
    }
  }
  return beats;
}

function slamGuideBeats(trace) {
  const samples = trace.samples.drifting_odometry;
  const t0 = samples[0][0];
  const odom = s => Math.hypot(s[4] - s[1], s[5] - s[2]);
  const drift = trace.report.drift_injected;
  const run = trace.report.runs.drifting_odometry;
  const scalePct = Math.round((drift.scale - 1) * 100);
  const beats = [{
    at: 0,
    title: 'Three paths, one flight',
    text: `Grey is Gazebo's pose, used only to score. Red is dead reckoning with ${scalePct}% scale error and ${drift.yaw_deg_per_s}°/s of heading drift injected. Mint is SLAM, which never reads the grey path.`,
  }];
  for (const thresh of [5, 10]) {
    const hit = samples.find(s => odom(s) >= thresh);
    if (hit) {
      beats.push({
        at: hit[0] - t0,
        title: `Dead reckoning is ${thresh} m off`,
        text: 'The red path has walked away from the aircraft. SLAM is still placing each scan on the map it has built.',
      });
    }
  }
  const end = samples[samples.length - 1];
  beats.push({
    at: end[0] - t0,
    title: 'The map is finished',
    text: `Dead reckoning ends ${odom(end).toFixed(1)} m off. After one alignment, the SLAM path averages ${(run.slam_ate_aligned_mean_m * 100).toFixed(0)} cm. The amber and purple routes are planned on the finished costmap after the flight.`,
  });
  return beats;
}

// parent is document.body for a viewport caption, or the replay figure for one
// that stays on the drawing. onReplay restarts playback; onHide runs when the
// viewer leaves the tour.
function mountGuide(parent, {onReplay, onHide} = {}) {
  const floating = parent === document.body;
  const panel = document.createElement('div');
  panel.className = 'guide-panel' + (floating ? ' guide-float' : '');
  panel.hidden = true;
  panel.innerHTML = '<div class="guide-eyebrow"><span class="guide-progress">Guide</span>'
    + '<button type="button" class="guide-exit">Fly it yourself</button></div>'
    + '<h2 class="guide-title"></h2><p class="guide-text"></p>';
  const resume = document.createElement('button');
  resume.type = 'button';
  resume.className = 'guide-resume' + (floating ? ' guide-float' : '');
  resume.hidden = true;
  resume.textContent = 'Turn the guide back on';
  parent.appendChild(panel);
  parent.appendChild(resume);
  let beats = [];
  let active = false;
  let last = -2;
  function paint(flightSeconds) {
    const idx = beatIndex(beats, flightSeconds);
    if (idx < 0 || idx === last) return;
    last = idx;
    panel.querySelector('.guide-progress').textContent = `Guide · ${idx + 1} of ${beats.length}`;
    panel.querySelector('.guide-title').textContent = beats[idx].title;
    panel.querySelector('.guide-text').textContent = beats[idx].text;
  }
  function update(flightSeconds) {
    if (active) paint(flightSeconds);
  }
  function show(replay) {
    active = true;
    last = -2;
    panel.hidden = false;
    resume.hidden = true;
    if (replay && onReplay) onReplay();
  }
  function hide() {
    active = false;
    panel.hidden = true;
    resume.hidden = false;
    if (onHide) onHide();
  }
  panel.querySelector('.guide-exit').addEventListener('click', hide);
  resume.addEventListener('click', () => show(false));
  return {
    update, show, hide,
    setBeats(next) { beats = next || []; last = -2; },
    get active() { return active; },
  };
}

if (typeof module !== 'undefined') {
  module.exports = {beatIndex, searchGuideBeats, seedingGuideBeats, slamGuideBeats, mountGuide};
}
