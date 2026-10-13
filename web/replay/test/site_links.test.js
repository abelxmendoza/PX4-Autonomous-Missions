// Every page's internal links and images must point at files that exist in the site,
// so a deploy cannot ship a page with a dead link or a missing screenshot.
import { describe, it, expect } from 'vitest';
import { readFileSync, existsSync, statSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const SITE = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const PAGES = ['index.html', 'demo/index.html', 'writeup/index.html', 'search/index.html', 'search-demo/index.html', 'slam/index.html', 'seeding/index.html'];

function localRefs(html) {
  const refs = [];
  for (const m of html.matchAll(/\s(?:href|src)="([^"]+)"/g)) {
    const ref = m[1];
    if (/^(https?:|mailto:|tel:|#|data:|javascript:)/.test(ref) || ref.includes('${')) continue;
    refs.push(ref.split('#')[0].split('?')[0]);
  }
  return refs.filter(Boolean);
}

function resolveRef(page, ref) {
  const base = ref.startsWith('/') ? join(SITE, ref) : resolve(dirname(join(SITE, page)), ref);
  if (existsSync(base) && statSync(base).isDirectory()) return join(base, 'index.html');
  return base;
}

describe('site links', () => {
  for (const page of PAGES) {
    it(`${page}: every internal link and image exists`, () => {
      const missing = localRefs(readFileSync(join(SITE, page), 'utf8'))
        .filter(ref => !existsSync(resolveRef(page, ref)));
      expect(missing).toEqual([]);
    });
  }

  it('the landing page links to the search page and the search page links back', () => {
    expect(localRefs(readFileSync(join(SITE, 'index.html'), 'utf8'))).toContain('/search/');
    expect(localRefs(readFileSync(join(SITE, 'search/index.html'), 'utf8'))).toContain('/');
  });

  it('every page links to both demos (other than itself)', () => {
    for (const page of PAGES) {
      const refs = localRefs(readFileSync(join(SITE, page), 'utf8'));
      if (page !== 'demo/index.html') expect(refs, page).toContain('/demo/');
      if (page !== 'search-demo/index.html') expect(refs, page).toContain('/search-demo/');
    }
  });

  it('the search demo replays the committed trace and draws real marker textures', () => {
    const html = readFileSync(join(SITE, 'search-demo/index.html'), 'utf8');
    expect(html).toContain('../data/search_trace.json');
    expect(html).toContain('../search-replay.js');
    const truth = JSON.parse(readFileSync(join(SITE, 'data/search_targets.json'), 'utf8'));
    for (const t of truth.targets) expect(existsSync(join(SITE, `search-demo/markers/aruco_${t.id}.png`))).toBe(true);
  });

  it('every page uses the same site tabs and marks exactly one current page', () => {
    const tabs = ['href="/demo/"', 'href="/search-demo/"', 'href="/search/"', 'href="/slam/"', 'href="/seeding/"'];
    const here = {
      'index.html': 'href="/" aria-current="page"',
      'demo/index.html': 'href="/demo/" aria-current="page"',
      'search-demo/index.html': 'href="/search-demo/" aria-current="page"',
      'search/index.html': 'href="/search/" aria-current="page"',
      'slam/index.html': 'href="/slam/" aria-current="page"',
      'seeding/index.html': 'href="/seeding/" aria-current="page"',
    };
    for (const page of PAGES) {
      const html = readFileSync(join(SITE, page), 'utf8');
      const nav = html.match(/<nav class="site-nav"[\s\S]*?<\/nav>/);
      expect(nav, page).not.toBeNull();
      expect(nav[0], page).not.toContain('href="/writeup/"');
      expect(html, page).not.toContain('BUG-021');
      for (const tab of tabs) expect(nav[0], page).toContain(tab);
      expect(nav[0], page).toMatch(/href="\/"(?:\s|>)/);
      if (page === 'writeup/index.html') {
        expect(nav[0].match(/aria-current="page"/g), page).toBeNull();
        continue;
      }
      expect(nav[0].match(/aria-current="page"/g), page).toHaveLength(1);
      expect(nav[0], page).toContain(here[page]);
    }
  });

  it('the seeding page loads the completed flight and the later campaign from committed evidence', () => {
    const html = readFileSync(join(SITE, 'seeding/index.html'), 'utf8');
    expect(html).toContain('../data/seeding_flight.json');
    expect(html).toContain('../data/seeding_campaign.json');
    const flight = JSON.parse(readFileSync(join(SITE, 'data/seeding_flight.json'), 'utf8'));
    const report = JSON.parse(readFileSync(join(SITE, '../../evidence/integrated_seeding/sitl_report.json'), 'utf8'));
    expect(flight.mission_result).toBe(report.mission_result);
    expect(flight.touchdown_horizontal_error_m).toBe(report.touchdown_horizontal_error_m);
    expect(flight.release_sample_count).toBe(report.release_sample_count);
    expect(flight.release_amount_kg).toBe(report.release_amount_kg);
    expect(flight.marker_id).toBe(report.marker_id);
    expect(flight.phases.map(p => p.destination)).toEqual(
      report.orchestrator_evidence.mission_state_history.map(h => h.destination));
    const campaign = JSON.parse(readFileSync(join(SITE, 'data/seeding_campaign.json'), 'utf8'));
    const summary = JSON.parse(readFileSync(join(SITE, '../../evidence/mission_verification/campaign_summary.json'), 'utf8'));
    expect(campaign).toEqual(summary);
    expect(campaign.campaign_outcome.passed).toBe(false);
    expect(campaign.runs.find(r => r.scenario_id === 'nominal').expectation_met).toBe(false);
  });

  it('the swarm demo opens with the guide and a mission picker for every recording', () => {
    const html = readFileSync(join(SITE, 'demo/index.html'), 'utf8');
    expect(html).not.toContain('landingHero');
    expect(html).not.toContain('Explore Freely');
    expect(html).toContain('class="mission-menu"');
    expect(html).toContain('id="missionStart"');
    expect(html).toContain('id="storyPanel" class="story-panel" hidden');
    expect(html).toContain('class="menu-open"');
    for (const id of ['golden', 'swarm', 'full', 'quick', 'clean', 'course', 'stereo']) {
      expect(html).toContain(`id:'${id}'`);
    }
    expect(html).toContain('let storyModeActive = true');
  });

  it('the search page loads its numbers from the committed data, not hard-coded text', () => {
    const html = readFileSync(join(SITE, 'search/index.html'), 'utf8');
    expect(html).toContain('../data/search_report.json');
    expect(html).toContain('../data/search_targets.json');
  });
});
