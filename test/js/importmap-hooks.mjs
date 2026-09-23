// Node module-resolution hooks that apply the browser import map from
// threejs_scene.html, so tests import the very same modules the page does.
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const html = readFileSync(path.join(root, 'threejs_scene.html'), 'utf8');
const { imports } = JSON.parse(html.match(/<script type="importmap">([\s\S]*?)<\/script>/)[1]);

// Longest prefix first, so 'three/addons/' wins over 'three'.
const entries = Object.entries(imports).sort((a, b) => b[0].length - a[0].length);

export async function resolve(specifier, context, next) {
  for (const [key, target] of entries) {
    const hit = key.endsWith('/') ? specifier.startsWith(key) : specifier === key;
    if (hit) {
      const mapped = target + specifier.slice(key.length);
      return next(pathToFileURL(path.join(root, mapped)).href, context);
    }
  }
  return next(specifier, context);
}
