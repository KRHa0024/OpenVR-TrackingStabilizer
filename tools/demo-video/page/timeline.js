// The main demo video: which scene is drawn when, from which camera, and the captions.
// Times are video seconds. Scenes come from build/demo-video/data.js (tools/demo-video/prepare.py).
"use strict";

const FULL = [0, 0, 1920, 1080];
const LEFT = [0, 0, 960, 1080], RIGHT = [960, 0, 960, 1080];
const THIRDS = [[0, 0, 640, 1080], [640, 0, 640, 1080], [1280, 0, 640, 1080]];

const OFF = { text: "補正なし", cls: "off" }, ON = { text: "補正あり", cls: "on" };
const coverCam = { az: 30, el: 14, dist: 3.0, target: [.15, .55, .2] };
const handCam = { az: 30, el: 12, dist: 3.3, target: [0, .75, .1] };
const waistCam = { az: 0, el: 10, dist: 3.6, target: [-.05, .8, 0] };
const REAL = "実際の記録";
const MASKED = "実際の記録で、見えていた区間を3秒だけ隠して再生";

window.TIMELINE = {
  duration: 96,
  segments: [
    // Opening: the author's sentence over the cover scene, without and with the filter.
    { kind: "scene", start: 0, end: 9, scene: "cover", from: 0, dim: .45, views: [
      { rect: LEFT, cam: coverCam, mode: "off", label: OFF },
      { rect: RIGHT, cam: coverCam, mode: "hold", label: ON }] },
    { kind: "title", start: 0, end: 9, title: "OpenVR-TrackingStabilizer" },

    // Hidden under a cloth: what lighthouse sends, then what the filter sends.
    { kind: "scene", start: 9, end: 18, scene: "cover", from: 0, source: REAL, views: [
      { rect: FULL, cam: coverCam, mode: "off", label: OFF, timer: true }] },
    { kind: "scene", start: 18, end: 24, scene: "cover", from: 1.0, source: REAL, views: [
      { rect: FULL, cam: coverCam, mode: "hold", label: ON, timer: true }] },

    // A hidden hand follows the body: hold vs the hand model, real time then half speed.
    { kind: "scene", start: 24, end: 31, scene: "hand", from: 0, source: MASKED, views: [
      { rect: LEFT, cam: handCam, mode: "hold", ghost: true, label: { text: "その場で保持" } },
      { rect: RIGHT, cam: handCam, mode: "model", ghost: true, label: { text: "このツール", cls: "on" } }] },
    { kind: "scene", start: 31, end: 38, scene: "hand", from: 1.5, speed: .5, source: MASKED, views: [
      { rect: LEFT, cam: handCam, mode: "hold", ghost: true, label: { text: "その場で保持" } },
      { rect: RIGHT, cam: handCam, mode: "model", ghost: true, label: { text: "このツール", cls: "on" } }] },
    { kind: "chart", start: 38, end: 48, title: "体を動かしながら3秒隠したときの平均のずれ（実際の記録）", bars: [
      { name: "その場で保持", value: "約16 cm", width: 16 * 42, color: "#7a8394" },
      { name: "このツール", value: "約10〜12 cm", width: 11 * 42, color: "#ff9e33" }] },

    // VRChat's freeze vs the body model, on the waist.
    { kind: "scene", start: 48, end: 55, scene: "waist", from: 0, source: MASKED + "。VRChat の列は隠して0.25秒後の位置で止めた再現", views: [
      { rect: THIRDS[0], cam: waistCam, mode: "truth", label: { text: "本物" } },
      { rect: THIRDS[1], cam: waistCam, mode: "vrchat", ghost: true, label: { text: "VRChat の固定" } },
      { rect: THIRDS[2], cam: waistCam, mode: "model", ghost: true, label: { text: "このツール", cls: "on" } }] },
    { kind: "scene", start: 55, end: 72, scene: "waist", from: 1.5, speed: .5, until: 6.9, source: MASKED + "。VRChat の列は隠して0.25秒後の位置で止めた再現", views: [
      { rect: THIRDS[0], cam: waistCam, mode: "truth", label: { text: "本物" } },
      { rect: THIRDS[1], cam: waistCam, mode: "vrchat", ghost: true, label: { text: "VRChat の固定" } },
      { rect: THIRDS[2], cam: waistCam, mode: "model", ghost: true, label: { text: "このツール", cls: "on" } }] },

    // The manager app, then the body model as seen from inside VR.
    { kind: "shot", start: 72, end: 77, src: "../../../build/shots/main_ja_2x.png", rect: [200, 70, 1520, 950],
      zoom: 2.3, zoomAt: "86% 22%" },
    { kind: "scene", start: 77, end: 82, scene: "waist", from: 1.8, views: [
      { rect: FULL, mode: "model", firstPerson: true, cam: { fov: 60 } }] },

    // How to get it.
    { kind: "card", start: 82, end: 96, html:
      `<h2>OpenVR-TrackingStabilizer</h2>
       <div class="row"><span>対応</span>SteamVR の Lighthouse コントローラーとトラッカー</div>
       <div class="row"><span>導入</span>SteamVR を閉じて、zip 内の Setup.cmd を実行します</div>` },
  ],
  captions: [
    { start: 0, end: 9, text: "Lighthouse 系のトラッカーが布やオブジェクトに<br>隠れていても、飛びにくくするツールを作りました" },
    { start: 9, end: 13, text: "トラッカーが布やものに隠れると" },
    { start: 13, end: 18, text: "位置が飛びやすく、約2秒でロストします" },
    { start: 18, end: 24, text: "このツールなら、隠れてもその場に留まります" },
    { start: 24, end: 30, text: "隠れている間も頭の動きから位置を補間します" },
    { start: 30, end: 35, text: "体を動かすと、隠れた手もついてきます" },
    { start: 35, end: 40, text: "手の向きの変化から、肘を支点に手を動かします" },
    { start: 40, end: 48, text: "体を動かしながら3秒隠したとき<br>平均のずれ 約16 cm → 約10〜12 cm" },
    { start: 40, end: 48, note: true, text: "じっとしているときは、保持とほぼ同じです（差は2 cm以内）" },
    { start: 48, end: 56, text: "VRChat にも、トラッキングが切れたトラッカーを<br>その場で固定する仕組みがあります" },
    { start: 56, end: 62, text: "ただし固定したあとは、体を動かしても止まったままです" },
    { start: 62, end: 72, text: "このツールは隠れた地点で留まり、腰が体の動きについてきます" },
    { start: 62, end: 72, note: true, text: "腰の補間は実験的機能です" },
    { start: 72, end: 77, text: "管理画面のマップで、推定中の体の形を見られます" },
    { start: 77, end: 82, text: "VR の中では、自分の体に重ねて表示できます" },
    { start: 77, end: 82, note: true, text: "VR の中の表示はデバッグ用です（画面は再現）" },
  ],
};
