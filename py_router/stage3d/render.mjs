// Render every distinct state of a stage3d timeline to PNG (#1081).
//
//   node render.mjs job.json
//
// job = { browser, width, height, outDir, scene, timeline, glb|null, colors }
// Writes <outDir>/s000000.png ... one per timeline state, and prints one JSON
// object per line on stdout: {"type":"info",...} first (the WebGL renderer
// string, the browser version, three's revision), progress lines, then
// {"type":"done"}. Any failure prints {"type":"error","why":...} and exits 2.
//
// The page is served through page.route from this directory -- never file://,
// where Chromium refuses ES-module imports (measured). Chromium runs on
// SwiftShader (--use-angle=swiftshader), a CPU rasteriser, so a render does
// not depend on the machine's GPU or driver: the caller refuses any other.
import { chromium } from 'playwright-core';
import fs from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const here = path.dirname(url.fileURLToPath(import.meta.url));
const out = (o) => process.stdout.write(JSON.stringify(o) + '\n');
const TYPES = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript',
                '.json': 'application/json', '.glb': 'model/gltf-binary' };

async function main() {
  const job = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const files = { '/data/scene.json': job.scene, '/data/timeline.json': job.timeline };
  if (job.glb) files['/data/parts.glb'] = job.glb;
  const browser = await chromium.launch({
    executablePath: job.browser, headless: true,
    args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--use-gl=angle',
           '--ignore-gpu-blocklist', '--disable-gpu-driver-bug-workarounds'] });
  const errors = [];
  try {
    const page = await browser.newPage({ viewport: { width: job.width, height: job.height },
                                         deviceScaleFactor: 1 });
    page.on('pageerror', e => errors.push(String(e)));
    page.on('console', m => { if (m.type() === 'error') errors.push(m.text()); });
    // nothing leaves this machine: every other host is refused (a GLB can
    // carry external buffer URIs)
    await page.route((u) => !String(u).startsWith('http://stage3d.local/'),
                     (route) => route.abort());
    await page.route('http://stage3d.local/**', async (route) => {
      const p = new URL(route.request().url()).pathname;
      let file = files[p];
      if (!file && (p.startsWith('/page/') || p.startsWith('/vendor/'))) {
        file = path.join(here, ...p.split('/').filter(Boolean));
        if (!path.resolve(file).startsWith(here + path.sep)) file = null;
      }
      if (!file || !fs.existsSync(file)) return route.fulfill({ status: 404, body: 'not found' });
      return route.fulfill({ path: file,
                             contentType: TYPES[path.extname(file)] || 'application/octet-stream' });
    });
    await page.goto('http://stage3d.local/page/index.html');
    await page.waitForFunction(() => window.stage3dReady === true, null, { timeout: 60000 });
    const info = await page.evaluate((o) => window.stage3dInit(o), {
      sceneUrl: 'http://stage3d.local/data/scene.json',
      timelineUrl: 'http://stage3d.local/data/timeline.json',
      glbUrl: job.glb ? 'http://stage3d.local/data/parts.glb' : null,
      width: job.width, height: job.height, colors: job.colors });
    out({ type: 'info', ...info, browser: browser.version(), errors });
    // refuse a GPU BEFORE rendering anything: its pixels depend on the
    // machine, and a 2000-state job would render for minutes to be thrown away
    if (!/swiftshader/i.test(String(info.renderer || ''))) {
      out({ type: 'error', why: 'renderer is ' + info.renderer + ', not SwiftShader' });
      process.exitCode = 2;
      return;
    }
    if (job.probe !== undefined && job.probe !== null) {
      out({ type: 'probe', ...(await page.evaluate((k) => window.stage3dProbe(k), job.probe)) });
    }
    const cdp = await page.context().newCDPSession(page);
    fs.mkdirSync(job.outDir, { recursive: true });
    const t0 = Date.now();
    for (let i = 0; i < info.states; i++) {
      await page.evaluate((k) => window.renderState(k), i);
      const { data } = await cdp.send('Page.captureScreenshot', {
        format: 'png', clip: { x: 0, y: 0, width: job.width, height: job.height, scale: 1 } });
      fs.writeFileSync(path.join(job.outDir, 's' + String(i).padStart(6, '0') + '.png'),
                       Buffer.from(data, 'base64'));
      if (i % 50 === 49) out({ type: 'progress', done: i + 1, of: info.states });
    }
    if (errors.length) { out({ type: 'error', why: 'page errors: ' + errors.slice(0, 3).join(' | ') }); process.exitCode = 2; return; }
    out({ type: 'done', states: info.states, ms_per_state: (Date.now() - t0) / Math.max(1, info.states) });
  } finally {
    await browser.close();
  }
}

main().catch((e) => { out({ type: 'error', why: String(e && e.stack || e).split('\n')[0] }); process.exitCode = 2; });
