// Render every frame of a movie3d timeline to PNG (prototype; adapted from
// the removed py_router/stage3d/render.mjs, #1081).
//
//   node render.mjs job.json
//
// job = { browser|null, width, height, outDir, scene, timeline, glb|null }
// Writes <outDir>/f00000.png ... one per timeline frame, and prints one JSON
// object per line on stdout: {"type":"info",...} first (the WebGL renderer,
// the browser version, three's revision), progress lines, then
// {"type":"done"}. Any failure prints {"type":"error","why":...} and exits 2.
//
// The page is served through page.route from this directory -- never
// file://, where Chromium refuses ES-module imports. Nothing leaves the
// machine: every other host is refused. Chromium runs on SwiftShader (a CPU
// rasteriser), so the pixels do not depend on the machine's GPU.
import { chromium } from 'playwright-core';
import fs from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const here = path.dirname(url.fileURLToPath(import.meta.url));
const out = (o) => process.stdout.write(JSON.stringify(o) + '\n');
const TYPES = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript',
                '.json': 'application/json', '.glb': 'model/gltf-binary' };
const ROOTS = { '/page/': path.join(here, 'page'),
                '/three/': path.join(here, 'node_modules', 'three', 'build'),
                '/three-addons/': path.join(here, 'node_modules', 'three', 'examples', 'jsm') };

async function main() {
  const job = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const files = { '/data/scene.json': job.scene, '/data/timeline.json': job.timeline };
  if (job.glb) files['/data/parts.glb'] = job.glb;
  const browser = await chromium.launch({
    executablePath: job.browser || undefined, headless: true,
    // job.gpu: an installed Chrome drawing on the GPU through Metal (make_movie
    // --gpu); else SwiftShader, the software renderer
    args: job.gpu ? ['--use-angle=metal', '--use-gl=angle', '--enable-gpu', '--ignore-gpu-blocklist']
                  : ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--use-gl=angle',
                     '--ignore-gpu-blocklist'] });
  const errors = [];
  try {
    const page = await browser.newPage({ viewport: { width: job.width, height: job.height },
                                         deviceScaleFactor: 1 });
    page.on('pageerror', (e) => errors.push(String(e)));
    page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
    await page.route((u) => !String(u).startsWith('http://movie3d.local/'), (route) => route.abort());
    await page.route('http://movie3d.local/**', async (route) => {
      const p = new URL(route.request().url()).pathname;
      let file = files[p];
      for (const [pre, dir] of Object.entries(ROOTS)) {
        if (!file && p.startsWith(pre)) {
          file = path.join(dir, ...p.slice(pre.length).split('/').filter(Boolean));
          if (!path.resolve(file).startsWith(dir + path.sep)) file = null;
        }
      }
      if (!file || !fs.existsSync(file)) return route.fulfill({ status: 404, body: 'not found' });
      return route.fulfill({ path: file, contentType: TYPES[path.extname(file)] || 'application/octet-stream' });
    });
    await page.goto('http://movie3d.local/page/index.html');
    await page.waitForFunction(() => window.m3dReady === true, null, { timeout: 60000 });
    const info = await page.evaluate((o) => window.m3dInit(o), {
      sceneUrl: 'http://movie3d.local/data/scene.json',
      timelineUrl: 'http://movie3d.local/data/timeline.json',
      glbUrl: job.glb ? 'http://movie3d.local/data/parts.glb' : null,
      width: job.width, height: job.height });
    out({ type: 'info', ...info, browser: browser.version(), errors });
    const cdp = await page.context().newCDPSession(page);
    fs.mkdirSync(job.outDir, { recursive: true });
    const t0 = Date.now();
    const only = job.only || null;           // a list of frame indices, for a quick look
    const todo = only || [...Array(info.frames).keys()];
    for (const i of todo) {
      await page.evaluate((k) => window.m3dFrame(k), i);
      const { data } = await cdp.send('Page.captureScreenshot', {
        format: 'png', clip: { x: 0, y: 0, width: job.width, height: job.height, scale: 1 } });
      fs.writeFileSync(path.join(job.outDir, 'f' + String(i).padStart(5, '0') + '.png'), Buffer.from(data, 'base64'));
      if (errors.length) break;
      if (i % 40 === 39) out({ type: 'progress', done: i + 1, of: info.frames });
    }
    if (errors.length) { out({ type: 'error', why: 'page errors: ' + errors.slice(0, 3).join(' | ') }); process.exitCode = 2; return; }
    out({ type: 'done', frames: todo.length, ms_per_frame: (Date.now() - t0) / Math.max(1, todo.length) });
  } finally {
    await browser.close();
  }
}

main().catch((e) => { out({ type: 'error', why: String(e && e.stack || e).split('\n')[0] }); process.exitCode = 2; });
