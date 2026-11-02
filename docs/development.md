# 開発者向けメモ

## 構成

| パス | 内容 |
|---|---|
| `driver/` | SteamVR ドライバ `driver_trackingstabilizer.dll`（デバイス登録なし・フックと記録のみ） |
| `filter/` | Phase 1 のフィルタ `TrackingFilter`（OpenVR の実行時機能に依存しない） |
| `tools/ots-replay/` | 記録をフィルタに通して再生し、フィルタ後の `.otsrec` を書き出す |
| `common/` | ドライバとツールで共有する定数・記録フォーマット |
| `tools/ots-overlay/` | SteamVR ダッシュボードとデスクトップのウィンドウに出す管理画面（RmlUi）。仕様は [docs/phase2.md](phase2.md) |
| `tools/ots-cli/` | 制御パイプ経由で記録の開始/停止・マーカー・実験プロトコルを実行する CLI |
| `tools/analysis/` | `.otsrec` の読み込み・CSV 出力・グラフ化（Python） |
| `tests/` | フィルタの単体テスト、ドライバの結合テスト、記録フォーマットの往復テスト |
| `external/` | openvr SDK / MinHook / RmlUi / FreeType / lunasvg（git submodule） |

## ビルド

Visual Studio 2022（C++ ワークロード）が必要。CMake は VS 付属のものでよい。

```powershell
git submodule update --init --recursive --depth 1
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

`build/package/trackingstabilizer/` に SteamVR から読み込める形のドライバフォルダが生成される。

## インストール（利用者向け）

リリースの zip を展開し、SteamVR を終了してから `Setup.cmd` をダブルクリックして [1] を選ぶ。
ドライバとアプリが `%LOCALAPPDATA%\OpenVR-TrackingStabilizer` に入り、SteamVR への登録（ドライバ・アプリの自動起動）と
スタートメニューのショートカットまで済む。管理者権限は不要。更新も同じ手順、削除は [2]。
手順の詳細は [installer/README.txt](../installer/README.txt)。

## 配布物の作成

```powershell
.\scripts\make-release.ps1   # Release ビルドして build\dist\OpenVR-TrackingStabilizer-<version>.zip を作る
```

zip には `driver\`（ドライバ）、`app\`（管理画面）、インストーラ（`Setup.cmd` / `setup.ps1`）、`README.txt`、
`THIRD_PARTY_NOTICES.txt`（各サブモジュールのライセンスから生成）が入る。バージョンはルートの `CMakeLists.txt` の `project(... VERSION)`。

## 開発中のインストール

SteamVR を終了した状態で:

```powershell
.\scripts\install-driver.ps1             # ビルドしたドライバを %LOCALAPPDATA%\OpenVR-TrackingStabilizer\driver にコピーして登録
.\scripts\install-driver.ps1 -Uninstall  # 登録解除と削除
```

管理画面はビルドフォルダの `ots-overlay.exe` を一度起動すれば登録される（インストール版から切り替えたときも、起動したほうに登録し直す）。

問題が起きたときは SteamVR の設定 → スタートアップ/シャットダウン → アドオンの管理 で無効化するか、
`steamvr.vrsettings` の `driver_trackingstabilizer.enable` を `false` にする。

## 設定（`steamvr.vrsettings` の `driver_trackingstabilizer`）

| キー | 既定 | 説明 |
|---|---|---|
| `hookPoses` | `true` | `IVRServerDriverHost::TrackedDevicePoseUpdated` をフックしてポーズを記録（フィルタにも必要） |
| `filterEnabled` ほか | — | Phase 1 のフィルタの設定。一覧は [docs/phase1.md](phase1.md) |
| `filterIncludeSerials` / `filterExcludeSerials` | `""` | デバイスごとの補正の切り替え（シリアルのカンマ区切り）。管理画面から設定する |
| `smoothing` | `0` | ジッター補正（0–100 %） |
| `hookDriverLog` | `true` | `IVRDriverLog::Log` をフックしてドライバログを記録 |
| `autoRecord` | `false` | SteamVR 起動時から記録を開始 |
| `recordDir` | `""` | 記録先。空なら `%LOCALAPPDATA%\OpenVR-TrackingStabilizer\sessions` |

## 使い方

管理画面: SteamVR を起動した状態で `build\tools\ots-overlay\Release\ots-overlay.exe` を一度起動すると、
SteamVR のダッシュボードに「Tracking Stabilizer」が追加され、以後は SteamVR と一緒に起動する。

```powershell
.\build\tools\ots-cli\Release\ots-cli.exe status
.\build\tools\ots-cli\Release\ots-cli.exe protocol occlude --target right_hand --record
python tools\analysis\otsrec.py <session.otsrec>
python tools\analysis\plot_session.py <session.otsrec>
```

フィルタの検証（Phase 0 の全記録を再生して合格基準を表示）:

```powershell
.\build\tests\Release\filter_test.exe
.\scripts\replay-all.ps1                          # 既定値
.\scripts\replay-all.ps1 -Set "probation=0.5"     # パラメータを変えて比較
```

## 遮蔽中の腰の身体モデル補完

設定画面に、既定オフの「遮蔽中の腰を身体モデルで補完」を追加しています。
腰トラッカーとHMDを明示指定し、正常時に学習した寸法と遮蔽中のpose回転から腰位置を推定します。
正常時は実測を使い、入力の信頼度が足りない場合は位置保持へ戻ります。
使い方・制限・オフライン比較は [docs/body-model.md](body-model.md) を参照してください。

3点構成（HMDと両手）向けには、独立した「遮蔽中の手を肩に追従させる」トグルもあります（既定オフ）。
最後の手位置を肩に対して保持し、腕の到達距離と推定時間で制限します。
使い方・制限・検証結果は [docs/hand-model.md](hand-model.md) を参照してください。
