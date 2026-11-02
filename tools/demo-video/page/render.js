// Draws one frame of the demo video at a given time: window.renderAt(seconds).
// The capture script calls it frame by frame; opening the page in a browser plays it (?t= jumps).
"use strict";

const W = 1920, H = 1080;
const canvas = document.getElementById("view");
const ctx = canvas.getContext("2d");
const overlay = document.getElementById("overlay");

const COLORS = {
  tracked: "#5aa2ff", estimating: "#ff9e33", drifting: "#ff6b5a", lost: "#7a8394",
  bone: "rgba(225, 235, 250, .92)", ghost: "rgba(255, 255, 255, .28)", frozen: "#b9c2d3",
  grid: "rgba(120, 150, 200, .16)", cloth: "rgba(214, 200, 176, .62)",
};

// ---------- math ----------
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = (a) => Math.sqrt(dot(a, a));
const norm = (a, fb = [0, 1, 0]) => { const n = len(a); return n > 1e-6 ? mul(a, 1 / n) : fb; };
const lerp = (a, b, k) => add(a, mul(sub(b, a), k));
function qrot(q, v) {
  const [w, x, y, z] = q, u = [x, y, z];
  const c = cross(u, v), cc = cross(u, c);
  return add(v, add(mul(c, 2 * w), mul(cc, 2)));
}
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const ease = (k) => k * k * (3 - 2 * k);

// Two-bone IK: the middle joint for a chain root -> end with the given lengths, bent towards `pole`.
function bend(root, end, l1, l2, pole) {
  const d = sub(end, root), dist = clamp(len(d), 1e-3, l1 + l2 - 1e-3), dir = norm(d);
  const a = (l1 * l1 - l2 * l2 + dist * dist) / (2 * dist), h = Math.sqrt(Math.max(0, l1 * l1 - a * a));
  const p = norm(sub(pole, mul(dir, dot(pole, dir))), [0, -1, 0]);
  return add(add(root, mul(dir, a)), mul(p, h));
}

// ---------- camera ----------
function camera(view, facing) {
  const c = view.cam, az = facing + (c.az || 0) * Math.PI / 180, el = (c.el || 10) * Math.PI / 180;
  const target = c.target || [0, .95, 0];
  const eye = c.eye || add(target, mul([Math.cos(el) * Math.sin(az), Math.sin(el), Math.cos(el) * Math.cos(az)], c.dist || 3.2));
  const look = c.look || target;
  const f = norm(sub(look, eye)), r = norm(cross(f, [0, 1, 0]), [1, 0, 0]), u = cross(r, f);
  const [x, y, w, h] = view.rect;
  const focal = h / 2 / Math.tan((c.fov || 38) * Math.PI / 360);
  return {
    eye, focal,
    project(p) {
      const v = sub(p, eye), z = dot(v, f);
      if (z < .05) return null;
      return [x + w / 2 + dot(v, r) * focal / z, y + h / 2 - dot(v, u) * focal / z, z];
    },
  };
}

// ---------- scene data ----------
function frameOf(scene, sceneT) {
  return clamp(Math.round(sceneT * scene.fps), 0, scene.frames - 1);
}

// Where the target device is drawn in a column, and in which state.
function deviceState(scene, mode, i) {
  const target = scene.name === "hand" ? "lhand" : "waist";
  const raw = scene.joints[target];
  const t = i / scene.fps, mask = scene.mask;
  const masked = mask && t >= mask[0] && t < mask[1];
  if (mode === "truth" || mode === "ghost") return { joint: target, pos: raw.pos[i], color: COLORS.tracked };
  if (mode === "off") {
    if (raw.pos[i] && i - lastSeen(raw, i) < 3) return { joint: target, pos: raw.pos[i], color: raw.ok[i] ? COLORS.tracked : COLORS.drifting };
    const j = lastSeen(raw, i);
    return { joint: target, pos: j >= 0 ? raw.pos[j] : null, color: COLORS.lost, lost: true };
  }
  if (mode === "vrchat") {
    const freeze = mask ? mask[0] + .25 : Infinity;
    if (t >= freeze && t < mask[1]) return { joint: target, pos: raw.pos[frameOf(scene, freeze)], color: COLORS.frozen, frozen: true };
    return { joint: target, pos: raw.pos[i], color: COLORS.tracked };
  }
  const out = scene.outputs[mode];
  const holding = mask ? masked : !raw.ok[i];
  return { joint: target, pos: out[i], color: holding ? COLORS.estimating : COLORS.tracked };
}

function lastSeen(track, i) {
  for (let j = i; j >= 0; --j) if (track.pos[j] && track.ok[j]) return j;
  for (let j = i; j >= 0; --j) if (track.pos[j]) return j;
  return -1;
}
// For the cover scene: the last frame the raw stream still had a pose (lighthouse goes silent after ~2 s).
function lastRaw(track, i) {
  for (let j = i; j >= 0; --j) if (track.pos[j]) return j;
  return -1;
}

// Upper/lower limb lengths for this scene: the longest root-to-end distance seen (98th percentile),
// so straight limbs look straight and bent ones bend.
function limbs(scene) {
  if (scene._limbs) return scene._limbs;
  const leg = [], arm = [];
  for (let i = 0; i < scene.frames; i += 3) {
    const s = skeleton(scene, i, null, { leg: 9, arm: 9 });
    if (!s) continue;
    for (const side of ["l", "r"]) {
      if (s[side + "foot"]) leg.push(len(sub(s[side + "hip"], s[side + "foot"])));
      if (s[side + "hand"]) arm.push(len(sub(s[side + "sh"], s[side + "hand"])));
    }
  }
  const pct = (v, fb) => (v.length ? v.sort((a, b) => a - b)[Math.floor(v.length * .98)] : fb);
  return (scene._limbs = { leg: clamp(pct(leg, .8) * 1.01, .55, .95), arm: clamp(pct(arm, .6) * 1.01, .45, .75) });
}

function skeleton(scene, i, override, size) {
  const L = size || limbs(scene);
  const J = scene.joints, get = (n) => (override && override.joint === n ? override.pos : J[n].pos[i]);
  const H = J.head.pos[i], q = J.head.rot[i];
  if (!H || !q) return null;
  const fh = qrot(q, [0, 0, -1]), f = norm([fh[0], 0, fh[2]], [0, 0, 1]), r = [-f[2], 0, f[0]];
  const headC = add(H, qrot(q, [0, .02, .07]));
  const neck = add(H, qrot(q, [0, -.16, .1]));
  const shoulder = (s) => add(add(neck, [0, -.04, 0]), mul(r, .18 * s));
  const waist = get("waist");
  const pelvis = waist ? add(waist, mul(f, .07)) : add(neck, [0, -.5, 0]);
  const hip = (s) => add(add(pelvis, [0, -.07, 0]), mul(r, .1 * s));
  const out = { headC, neck, pelvis, f, r };
  for (const [side, s] of [["l", -1], ["r", 1]]) {
    const sh = shoulder(s), hand = get(side + "hand");
    out[side + "sh"] = sh;
    if (hand) {
      const pole = add(add([0, -1, 0], mul(f, -.6)), mul(r, .5 * s));
      out[side + "el"] = bend(sh, hand, L.arm * .52, L.arm * .48, pole);
      out[side + "hand"] = hand;
    }
    const hp = hip(s), foot = get(side + "foot");
    out[side + "hip"] = hp;
    if (foot) {
      out[side + "knee"] = bend(hp, foot, L.leg * .51, L.leg * .49, add(f, mul(r, .15 * s)));
      out[side + "foot"] = foot;
      out[side + "toe"] = add([foot[0], Math.max(.02, foot[1] - .06), foot[2]], mul(f, .13));
    }
  }
  return out;
}

// ---------- drawing ----------
function line(cam, a, b, width, color) {
  const p = cam.project(a), q = cam.project(b);
  if (!p || !q) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = Math.max(1.5, width * cam.focal / ((p[2] + q[2]) / 2));
  ctx.lineCap = "round";
  ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
}
function dotAt(cam, p3, radius, fill, ring) {
  const p = cam.project(p3);
  if (!p) return null;
  const r = radius * cam.focal / p[2];
  ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, Math.PI * 2);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (ring) { ctx.strokeStyle = ring; ctx.lineWidth = Math.max(2, r * .35); ctx.stroke(); }
  return p;
}

function drawFloor(cam) {
  ctx.lineWidth = 1.5;
  for (let k = -6; k <= 6; ++k) {
    for (const [a, b] of [[[k * .5, 0, -3], [k * .5, 0, 3]], [[-3, 0, k * .5], [3, 0, k * .5]]]) {
      const p = cam.project(a), q = cam.project(b);
      if (!p || !q) continue;
      ctx.strokeStyle = COLORS.grid;
      ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
    }
  }
}

function drawFigure(cam, s, style) {
  if (!s) return;
  const c = style.ghost ? COLORS.ghost : COLORS.bone, w = style.ghost ? .03 : .035;
  const chain = (...ps) => { for (let k = 0; k + 1 < ps.length; ++k) if (ps[k] && ps[k + 1]) line(cam, ps[k], ps[k + 1], w, c); };
  chain(s.lsh, s.neck, s.rsh);
  chain(s.lsh, s.lel, s.lhand);
  chain(s.rsh, s.rel, s.rhand);
  if (style.lostWaist) {
    ctx.setLineDash([14, 12]); chain(s.neck, s.pelvis); ctx.setLineDash([]);
  } else chain(s.neck, s.pelvis);
  chain(s.lhip, s.pelvis, s.rhip);
  chain(s.lhip, s.lknee, s.lfoot, s.ltoe);
  chain(s.rhip, s.rknee, s.rfoot, s.rtoe);
  const head = !style.noHead && cam.project(s.headC);
  if (head) {
    ctx.beginPath(); ctx.arc(head[0], head[1], .11 * cam.focal / head[2], 0, Math.PI * 2);
    ctx.lineWidth = Math.max(2, w * cam.focal / head[2]); ctx.strokeStyle = c; ctx.stroke();
  }
}

function drawDevices(cam, s, state, ghost) {
  if (!s) return;
  for (const n of ["lhand", "rhand", "lfoot", "rfoot"]) {
    if (state && state.joint === n) continue;
    if (s[n]) dotAt(cam, s[n], .038, ghost ? COLORS.ghost : COLORS.tracked);
  }
  if (!state || state.joint !== "waist") {
    const J = s.pelvis && sub(s.pelvis, mul(s.f, .07));
    if (J) dotAt(cam, J, .038, ghost ? COLORS.ghost : COLORS.tracked);
  }
}

function drawState(cam, state) {
  if (!state || !state.pos) return null;
  if (state.lost) return dotAt(cam, state.pos, .042, null, COLORS.lost);
  return dotAt(cam, state.pos, .046, state.color, "rgba(0,0,0,.35)");
}

function drawCloth(cam, at) {
  const p = cam.project(at);
  if (!p) return;
  const w = .42 * cam.focal / p[2], h = .3 * cam.focal / p[2];
  ctx.fillStyle = COLORS.cloth;
  ctx.beginPath();
  ctx.roundRect(p[0] - w / 2, p[1] - h * .45, w, h, h * .25);
  ctx.fill();
}

// ---------- overlay ----------
function el(cls, html, style) {
  const d = document.createElement("div");
  d.className = cls;
  d.innerHTML = html;
  if (style) Object.assign(d.style, style);
  overlay.appendChild(d);
  return d;
}
function fade(t, a, b, k = .25) {
  return clamp(Math.min((t - a) / k, (b - t) / k), 0, 1);
}

// ---------- segments ----------
function renderScene(seg, t) {
  const scene = window.SCENES[seg.scene];
  let sceneT = seg.from + (t - seg.start) * (seg.speed || 1);
  if (seg.until !== undefined) sceneT = Math.min(sceneT, seg.until);
  const i = frameOf(scene, sceneT);
  for (const view of seg.views) {
    const cam = camera(view.firstPerson ? firstPerson(view, scene, i) : view, scene.facing);
    ctx.save();
    ctx.beginPath(); ctx.rect(...view.rect); ctx.clip();
    drawFloor(cam);
    const state = view.mode ? deviceState(scene, view.mode, i) : null;
    if (view.ghost) {
      const g = skeleton(scene, i);
      drawFigure(cam, g, { ghost: true });
      drawDevices(cam, g, null, true);
    }
    const s = skeleton(scene, i, state);
    const covered = clothAt(scene, i, seg);
    if (covered) drawCloth(cam, covered);
    drawFigure(cam, s, { lostWaist: state && state.lost });
    drawDevices(cam, s, state, false);
    const p = drawState(cam, state);
    ctx.restore();
    if (view.label) el("label " + (view.label.cls || "plain"), view.label.text, { left: view.rect[0] + 40 + "px", top: view.rect[1] + 36 + "px" });
    if (view.timer && p && scene.name === "cover") timer(scene, i, p, view.mode === "off");
  }
}

// A camera just behind and above the HMD, looking where the user faces: close to what the user sees
// of their own body in VR, while still showing the head.
function firstPerson(view, scene, i) {
  const H = scene.joints.head.pos[i], q = scene.joints.head.rot[i];
  const fh = qrot(q, [0, 0, -1]), f = norm([fh[0], 0, fh[2]], [0, 0, 1]);
  const eye = add(add(H, mul(f, -.85)), [0, .4, 0]);
  const look = add(add(H, mul(f, 1.0)), [0, -.95, 0]);
  return { rect: view.rect, cam: { ...view.cam, eye, look } };
}

// When the cover scene's waist is under the cloth (from shortly before lighthouse stops calling it
// healthy), or a masked device is "hidden" in the other scenes.
function clothAt(scene, i, seg) {
  if (seg.cloth === false) return null;
  if (scene.name === "cover") {
    const k = coverStart(scene);
    if (i < k - 15) return null;
    return scene.outputs.hold[i] || scene.joints.waist.pos[k];
  }
  const t = i / scene.fps, m = scene.mask;
  if (!m || t < m[0] || t >= m[1]) return null;
  return (scene.name === "hand" ? scene.joints.lhand : scene.joints.waist).pos[i];
}
function coverStart(scene) {
  if (scene._cover !== undefined) return scene._cover;
  const ok = scene.joints.waist.ok;
  let k = 0;
  while (k < ok.length && ok[k]) ++k;
  return (scene._cover = k);
}
function timer(scene, i, p, unfiltered) {
  const k = coverStart(scene);
  if (i < k) return;
  const raw = scene.joints.waist, silent = unfiltered && i - lastRaw(raw, i) > 3;
  const secs = ((i - k) / scene.fps).toFixed(1);
  el("timer" + (silent ? " lost" : ""), silent ? "ロスト" : `隠れてから ${secs} 秒`, { left: p[0] + "px", top: p[1] - 50 + "px" });
}

function renderChart(seg, t) {
  const k = ease(clamp((t - seg.start) / 1.2, 0, 1));
  const box = el("chart", `<h3>${seg.title}</h3>`);
  for (const b of seg.bars) {
    const row = document.createElement("div");
    row.className = "bar";
    row.innerHTML = `<div class="name">${b.name}</div><div class="fill" style="width:${b.width * k}px;background:${b.color}"></div><div class="value">${b.value}</div>`;
    box.appendChild(row);
  }
}

// Images are loaded once (window.READY) and the same element is shown every frame, so a captured
// frame never catches an image that is still decoding.
const images = {};
window.READY = Promise.all(window.TIMELINE.segments.filter((s) => s.src).map((s) => new Promise((done) => {
  const img = new Image();
  img.onload = img.onerror = done;
  img.src = s.src;
  images[s.src] = img;
}))).then(() => document.fonts.ready);

function renderShot(seg, t) {
  const k = clamp((t - seg.start) / (seg.end - seg.start), 0, 1);
  const d = el("shot", "", {
    left: seg.rect[0] + "px", top: seg.rect[1] + "px", width: seg.rect[2] + "px", height: seg.rect[3] + "px",
  });
  const img = images[seg.src];
  img.style.transformOrigin = seg.zoomAt || "50% 50%";
  img.style.transform = `scale(${1 + ((seg.zoom || 1.06) - 1) * ease(k)})`;
  d.appendChild(img);
}

function background() {
  const g = ctx.createLinearGradient(0, 0, 0, H);
  g.addColorStop(0, getComputedStyle(document.documentElement).getPropertyValue("--bg-top"));
  g.addColorStop(1, getComputedStyle(document.documentElement).getPropertyValue("--bg-bottom"));
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, W, H);
}

window.renderAt = function (t) {
  background();
  overlay.innerHTML = "";
  for (const seg of window.TIMELINE.segments) {
    if (t < seg.start || t >= seg.end) continue;
    ctx.globalAlpha = seg.dim ? seg.dim : 1;
    if (seg.kind === "scene") renderScene(seg, t);
    ctx.globalAlpha = 1;
    if (seg.kind === "chart") renderChart(seg, t);
    if (seg.kind === "shot") renderShot(seg, t);
    if (seg.kind === "title") el("title", `<h1>${seg.title}</h1>` + (seg.sub ? `<p>${seg.sub}</p>` : ""), { opacity: fade(t, seg.start, seg.end, .4) });
    if (seg.kind === "card") el("card", seg.html, { opacity: fade(t, seg.start, seg.end, .4) });
    if (seg.source) el("source", seg.source);
  }
  for (const c of window.TIMELINE.captions) {
    if (t < c.start || t >= c.end) continue;
    el(c.note ? "note" : "caption", c.text, { opacity: fade(t, c.start, c.end, .2) });
  }
};
window.DURATION = window.TIMELINE.duration;

// Browser preview: ?t=12.5 shows one frame, otherwise plays in real time.
(function preview() {
  const q = new URLSearchParams(location.search);
  if (q.has("capture")) return;
  if (q.has("t")) { window.renderAt(parseFloat(q.get("t"))); return; }
  const start = performance.now();
  const tick = () => {
    const t = ((performance.now() - start) / 1000) % window.DURATION;
    window.renderAt(t);
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
})();
