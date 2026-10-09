// Every page's internal links and images must point at files that exist in the site,
// so a deploy cannot ship a page with a dead link or a missing screenshot.
import { describe, it, expect } from 'vitest';
import { readFileSync, existsSync, statSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const SITE = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const PAGES = ['index.html', 'demo/index.html', 'writeup/index.html', 'search/index.html', 'search-demo/index.html'];

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
    for (const page of ['index.html', 'demo/index.html', 'writeup/index.html', 'search/index.html', 'search-demo/index.html']) {
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

  it('the search page loads its numbers from the committed data, not hard-coded text', () => {
    const html = readFileSync(join(SITE, 'search/index.html'), 'utf8');
    expect(html).toContain('../data/search_report.json');
    expect(html).toContain('../data/search_targets.json');
  });
});
