// lib.js expects THREE as a global, same as it is in the browser (provided
// there by the CDN <script> tag). Using the real `three` package here
// (pinned to the same 0.128.0 the page loads) rather than a hand-rolled
// stub, so tests exercise the actual library behavior, not an approximation
// of it.
import * as THREE from 'three';
globalThis.THREE = THREE;
