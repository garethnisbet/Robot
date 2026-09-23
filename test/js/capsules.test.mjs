// The planner's capsules must enclose the meshes the viewer collides, so
// its fast check can be stricter than the viewer's but never looser.
// Regenerate with: node --import ./headless/register.mjs headless/fit-capsules.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readdirSync, readFileSync, existsSync } from 'node:fs';
import { createEngine } from '../../headless/engine.mjs';
import { linkVertices, segmentDistance } from '../../headless/fit-capsules.mjs';

const root = new URL('../../', import.meta.url);
const read = (f) => JSON.parse(readFileSync(new URL(f, root), 'utf8'));
const serialConfigs = readdirSync(root).filter(f => f.endsWith('_config.json'))
  .filter(f => read(f).type !== 'hexapod').sort();

for (const configFile of serialConfigs) {
  const capsuleFile = configFile.replace(/_config\.json$/, '_capsules.json');

  test(`${capsuleFile} encloses every mesh vertex of every link`, async () => {
    assert.ok(existsSync(new URL(capsuleFile, root)), `${capsuleFile} is missing; run fit-capsules.mjs`);
    const capsules = read(capsuleFile);
    const engine = await createEngine();
    const dev = await engine.addDevice(configFile);
    engine.scene.updateMatrixWorld(true);

    assert.equal(capsules.model, dev.config.model, 'fitted to a different model');
    assert.deepEqual(capsules.links.map(l => [l.name, l.joint]),
                     dev.robotLinkMeshes.map(l => [l.name, l.jointIdx]),
                     'links differ from the ones the viewer collides; refit');

    for (const link of dev.robotLinkMeshes) {
      const cap = capsules.links.find(c => c.name === link.name);
      let worst = 0;
      for (const p of linkVertices(dev, link)) {
        worst = Math.max(worst, segmentDistance(p, cap.p0, cap.p1) - cap.radius);
      }
      assert.ok(worst <= 1e-9, `${link.name}: a vertex lies ${(worst * 1000).toFixed(3)} mm outside its capsule`);

      // The parts: every vertex inside at least one of them.
      let worstPart = 0;
      for (const p of linkVertices(dev, link)) {
        let best = Infinity;
        for (const c of cap.parts) best = Math.min(best, segmentDistance(p, c.p0, c.p1) - c.radius);
        worstPart = Math.max(worstPart, best);
      }
      assert.ok(worstPart <= 1e-9, `${link.name}: a vertex lies ${(worstPart * 1000).toFixed(3)} mm outside every part`);
    }
  });
}
