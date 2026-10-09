// Every page's internal links and images must point at files that exist in the site,
// so a deploy cannot ship a page with a dead link or a missing screenshot.
import { describe, it, expect } from 'vitest';
import { readFileSync, existsSync, statSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const SITE = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const PAGES = ['index.html', 'demo/index.html', 'writeup/index.html', 'search/index.html'];

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

  it('the search page loads its numbers from the committed data, not hard-coded text', () => {
    const html = readFileSync(join(SITE, 'search/index.html'), 'utf8');
    expect(html).toContain('../data/search_report.json');
    expect(html).toContain('../data/search_targets.json');
  });
});
