OpenVR Tracking Stabilizer
==========================

SteamVR のトラッカー・コントローラーがロスト（遮蔽など）したときに飛んでいくのを抑える
SteamVR ドライバと、その管理画面（SteamVR ダッシュボード / デスクトップ）です。

Keeps SteamVR trackers and controllers from flying away when they lose tracking, plus a
dashboard app (SteamVR dashboard and desktop window) to control it.


インストール / Install
----------------------
1. SteamVR を終了します。 / Close SteamVR.
2. このフォルダの Setup.cmd をダブルクリックし、[1] を選びます。
   Double-click Setup.cmd in this folder and choose [1].
3. SteamVR を起動し、ダッシュボードの「Tracking Stabilizer」を開きます。
   Start SteamVR and open "Tracking Stabilizer" in the dashboard.

管理者権限は不要です。インストール先は %LOCALAPPDATA%\OpenVR-TrackingStabilizer です。
No administrator rights needed. Files go to %LOCALAPPDATA%\OpenVR-TrackingStabilizer.

更新するときも同じ手順です（設定はそのまま残ります）。
To update, do the same again (settings are kept).

アンインストール / Uninstall
----------------------------
SteamVR を終了してから Setup.cmd を実行し、[2] を選びます。
Close SteamVR, run Setup.cmd and choose [2].

困ったとき / Troubleshooting
----------------------------
- SteamVR の設定 → スタートアップ/シャットダウン → アドオンの管理 で
  "trackingstabilizer" をオフにすると、ドライバを止められます。
  To turn the driver off: SteamVR Settings > Startup / Shutdown > Manage Add-ons > "trackingstabilizer".
- デスクトップのウィンドウはスタートメニューの「OpenVR Tracking Stabilizer」から開けます（SteamVR の起動中のみ）。
  The desktop window: Start menu > "OpenVR Tracking Stabilizer" (while SteamVR is running).

動作環境 / Requirements
-----------------------
Windows 10 / 11 (64-bit), SteamVR.

制作 / Made by: くろ～は (kuroha)  https://github.com/KRHa0024
使っているソフトウェアのライセンスは THIRD_PARTY_NOTICES.txt にあります。
Licenses of the software this uses are in THIRD_PARTY_NOTICES.txt.
