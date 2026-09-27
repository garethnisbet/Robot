// ============================================================
// headless/stdio.mjs — the headless engine as a subprocess
// ------------------------------------------------------------
//   node --import ./headless/register.mjs headless/stdio.mjs
//
// One JSON request per line on stdin: {"id": 1, "msg": {"cmd": ...}}.
// One JSON response per line on stdout: {"id": 1, "replies": [...]} or
// {"id": 1, "error": "..."}. Requests are handled in order. Anything the
// viewer's modules log goes to stderr, so stdout carries only responses.
// headless_client.py drives this.
// ============================================================
import { createInterface } from 'node:readline';

// Keep stdout for the protocol.
console.log = console.info = console.warn = console.debug = (...a) => process.stderr.write(a.join(' ') + '\n');

const { createEngine } = await import('./engine.mjs');
const engine = await createEngine();

const send = (obj) => process.stdout.write(JSON.stringify(obj) + '\n');
send({ id: 0, ready: true });

let queue = Promise.resolve();
createInterface({ input: process.stdin }).on('line', (line) => {
  if (!line.trim()) return;
  queue = queue.then(async () => {
    let id = null;
    try {
      const req = JSON.parse(line);
      id = req.id;
      send({ id, replies: await engine.handle(req.msg) });
    } catch (e) {
      send({ id, error: String(e && e.message || e) });
    }
  });
}).on('close', () => queue.then(() => process.exit(0)));
