// Resolve bare imports ('three', 'three/addons/…', 'gaussian-splats-3d')
// through the viewer page's import map, so Node loads the modules the page
// loads. Use with `node --import ./headless/register.mjs`.
import { register } from 'node:module';
register('./importmap-hooks.mjs', import.meta.url);
