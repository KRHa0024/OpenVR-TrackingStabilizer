// Renders the demo video page frame by frame in headless Edge and encodes it with ffmpeg.
//
//   node tools/demo-video/capture.mjs [--fps 60] [--from 0] [--to <duration>] [-o build/demo-video/main.mp4]
//   node tools/demo-video/capture.mjs --stills 3,12.5,40 [-o build/demo-video/stills]
//
// Talks to Edge over the DevTools protocol with Node's built-in WebSocket (Node 22+), so nothing has
// to be installed. Each frame calls window.renderAt(t) in the page and takes a screenshot.
import { spawn } from "node:child_process";
import { mkdirSync, mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, "..", "..");
const EDGE = "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe";
const PORT = 9333;

const args = process.argv.slice(2);
const opt = (name, fallback) => {
  const i = args.indexOf(name);
  return i >= 0 ? args[i + 1] : fallback;
};
const fps = Number(opt("--fps", 60));
const stills = opt("--stills", null);
const out = resolve(root, opt("-o", stills ? "build/demo-video/stills" : "build/demo-video/main.mp4"));

const profile = mkdtempSync(join(tmpdir(), "ots-demo-edge-"));
const edge = spawn(EDGE, [
  "--headless=new", `--remote-debugging-port=${PORT}`, `--user-data-dir=${profile}`, "--hide-scrollbars",
  "--mute-audio", "--allow-file-access-from-files", "--force-device-scale-factor=1", "about:blank",
], { stdio: "ignore" });

async function waitForEdge() {
  for (let i = 0; i < 100; ++i) {
    try {
      const list = await (await fetch(`http://127.0.0.1:${PORT}/json/list`)).json();
      const page = list.find((t) => t.type === "page");
      if (page) return page.webSocketDebuggerUrl;
    } catch { /* not up yet */ }
    await new Promise((r) => setTimeout(r, 100));
  }
  throw new Error("Edge did not start");
}

function connect(url) {
  const ws = new WebSocket(url);
  let id = 0;
  const pending = new Map(), listeners = [];
  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.id && pending.has(msg.id)) {
      const { ok, fail } = pending.get(msg.id);
      pending.delete(msg.id);
      msg.error ? fail(new Error(JSON.stringify(msg.error))) : ok(msg.result);
    } else if (msg.method) listeners.forEach((l) => l(msg));
  };
  const send = (method, params = {}) => new Promise((ok, fail) => {
    pending.set(++id, { ok, fail });
    ws.send(JSON.stringify({ id, method, params }));
  });
  const once = (method) => new Promise((ok) => listeners.push((m) => m.method === method && ok(m.params)));
  return new Promise((ok) => { ws.onopen = () => ok({ send, once, close: () => ws.close() }); });
}

async function evaluate(cdp, expression) {
  const r = await cdp.send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
  if (r.exceptionDetails) throw new Error(r.exceptionDetails.exception?.description || r.exceptionDetails.text);
  return r.result.value;
}

async function main() {
  const cdp = await connect(await waitForEdge());
  await cdp.send("Page.enable");
  await cdp.send("Runtime.enable");
  await cdp.send("Emulation.setDeviceMetricsOverride", { width: 1920, height: 1080, deviceScaleFactor: 1, mobile: false });
  const loaded = cdp.once("Page.loadEventFired");
  const page = pathToFileURL(join(here, "page", "index.html")).href + "?capture=1";
  await cdp.send("Page.navigate", { url: page });
  await loaded;
  await evaluate(cdp, "window.READY");
  const duration = await evaluate(cdp, "window.DURATION");
  const once = async (t) => {
    await evaluate(cdp, `window.renderAt(${t})`);
    const r = await cdp.send("Page.captureScreenshot", { format: "png", clip: { x: 0, y: 0, width: 1920, height: 1080, scale: 1 } });
    return Buffer.from(r.data, "base64");
  };
  // A screenshot now and then never answers; give up on it after a while and try again.
  const shot = async (t) => {
    for (let attempt = 1; ; ++attempt) {
      let timer;
      const timeout = new Promise((_, fail) => { timer = setTimeout(() => fail(new Error(`frame at ${t} timed out`)), 20000); });
      try {
        return await Promise.race([once(t), timeout]);
      } catch (e) {
        if (attempt >= 5) throw e;
        console.log(`${e.message}, retrying`);
      } finally {
        clearTimeout(timer);
      }
    }
  };

  if (stills) {
    mkdirSync(out, { recursive: true });
    for (const t of stills.split(",").map(Number)) {
      writeFileSync(join(out, `t${t.toFixed(2).padStart(6, "0")}.png`), await shot(t));
      console.log("still", t);
    }
  } else {
    const from = Number(opt("--from", 0)), to = Number(opt("--to", duration));
    mkdirSync(dirname(out), { recursive: true });
    const ffmpeg = spawn("ffmpeg", ["-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", String(fps), "-i", "-",
      "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", out],
      { stdio: ["pipe", "inherit", "inherit"] });
    const frames = Math.round((to - from) * fps);
    const started = Date.now();
    for (let f = 0; f < frames; ++f) {
      const png = await shot(from + f / fps);
      if (!ffmpeg.stdin.write(png)) await new Promise((r) => ffmpeg.stdin.once("drain", r));
      if (f % (fps * 5) === 0) console.log(`frame ${f}/${frames} (${((Date.now() - started) / 1000).toFixed(0)} s)`);
    }
    ffmpeg.stdin.end();
    await new Promise((r) => ffmpeg.on("close", r));
    console.log("wrote", out);
  }
  cdp.close();
}

main().catch((e) => { console.error(e); process.exitCode = 1; }).finally(() => {
  edge.kill();
  setTimeout(() => { try { rmSync(profile, { recursive: true, force: true }); } catch { /* still locked */ } }, 500);
});
