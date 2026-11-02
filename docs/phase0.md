# Phase 0: 計測

## 目的

トラッキングロストを確実に検出できる信号を、実測と解析（静的・動的）の両面から特定し、
Phase 1（ロスト時の位置ホールド）の判定方法としきい値を決める。
**この段階ではポーズを一切書き換えない。**

## 事前に分かっていること（2026-10-24 時点、SteamVR 2.18.4）

- 追跡処理はすべて `drivers/lighthouse/bin/win64/driver_lighthouse.dll`（署名付き）が行う。
  `drivers/tundra_labs` は `resourceOnly: true` で、Tundra Tracker も同じ DLL で追跡されている。
- Lighthouse ドライバが使うホストのインターフェースは `IVRServerDriverHost_006`（と `_004`）、ログは `IVRDriverLog_001`。
- DLL 内の文字列から、内部に `CImuOnlyTracker` / `CKalmanQuaternion` があり、
  `Switching Lighthouse -> IMU Only` / `Switching IMU Only -> Lighthouse` で状態を切り替えている。
- vrserver のログに `Resetting tracking: no optical samples for 2002ms` が出ている。
  光学サンプルが途切れても約 2 秒は IMU だけで推測を続け、そのあとリセットしている（＝吹っ飛びの主因と推定）。
- `IMU went off scale.` も出ている（衝撃や速い振りで測定範囲を超えた状態）。
- `SOB: add/drop S-<ch>` は、各ベースステーション（チャンネル）が見えた・見えなくなったという記録。

## 構成

| 要素 | 役割 |
|---|---|
| `driver_trackingstabilizer.dll` | vrserver に読み込まれ、`TrackedDevicePoseUpdated`（`_006` / `_005`）と `IVRDriverLog::Log` を MinHook でフックする。受け取った内容は記録したあと、そのまま元の関数へ渡す |
| 制御パイプ `\\.\pipe\OpenVR-TrackingStabilizer` | 1行のコマンドに 1行の JSON で応答する。将来のダッシュボード用オーバーレイもこれを使う |
| `ots-cli` | 記録の開始/停止、マーカー、音で合図する実験プロトコル |
| `tools/analysis` | `.otsrec` → CSV / PNG |

### 制御コマンド

| コマンド | 説明 |
|---|---|
| `ping` | 疎通確認 |
| `status` | 記録状態、件数、取りこぼし数、フックの状態 |
| `start [label]` | 記録開始（`<recordDir>/<日時>_<label>.otsrec`） |
| `stop` | 記録停止 |
| `mark <text>` | マーカーを記録 |

### 記録内容（`common/RecordFormat.h`）

| 種類 | 内容 |
|---|---|
| Pose | フックが受け取った `DriverPose_t` をそのまま（デバイス番号・どのフック経由か付き） |
| DriverLog | 全ドライバのログ1行（呼び出し元ドライバの `IVRDriverLog` インスタンスのアドレス付き） |
| Marker | クライアントからのマーカー |
| DeviceInfo | シリアル・クラス・型番・メーカー・追跡システム名・役割 |
| RawPoses | `GetRawTrackedDevicePoses` の 1 秒ごとのスナップショット（ベースステーションの姿勢を含む） |
| Note | 記録の開始・停止、フック状態 |

時刻はすべて QPC（`QueryPerformanceCounter`）。

## 実験手順

前提: SteamVR 起動中、`ots-cli status` で `"pose0":"IVRServerDriverHost_006"` と `"log":true` を確認しておく。

| ID | 内容 | コマンド例 |
|---|---|---|
| E1 | 全デバイス静止、全 BS 可視で 60 秒 | `ots-cli protocol static --duration 60 --record` |
| E2 | 置いたトラッカー / コントローラーに布や箱を被せて完全に隠す（1・3・5・10 秒） | `ots-cli protocol occlude --target right_hand --record` |
| E3 | 全デバイスを置いたまま、BS を 1 台ずつ遮る（A→B→C） | `ots-cli protocol timed --label bs_block --details bs=A,bs=B,bs=C --duration 10 --settle 5 --repeat 2 --switch 15 --record` |
| E4 | フルトラを装着して、体の陰に入る姿勢をとる（前屈・しゃがむ・あぐら・横向きに寝る） | `ots-cli protocol e4 --record`（日本語音声ガイド付き） |
| E5 | コントローラー同士を軽くぶつける・速く振る・足踏み | `ots-cli protocol e5 --record`（日本語音声ガイド付き） |
| E6 | 覆ったまま動かす（トンネルに通す） | `ots-cli protocol timed --label occlude_moving --detail target=tundra_slide --duration 2 --settle 2 --repeat 5 --record` |
| E7 | 覆った状態から復帰 | E2 / E6 の `_end` 直後を解析 |

対象: Index コントローラー（左右）、Tundra Tracker。HMD（Bigscreen Beyond）は記録のみ。

`occlude` / `timed` の 1 試行の流れとマーカー:

| 合図 | マーカー | やること |
|---|---|---|
| 短いビープ ×3 | — | 準備 |
| 長いビープ | `<name>_cover` | `--settle` 秒（既定 3 秒）以内に、ゆっくり被せる |
| 短い 2 連音 | `<name>_start` | 被せたまま静止（`hold` 秒） |
| 高いビープ | `<name>_uncover` | `--settle` 秒以内に、ゆっくり外す |
| — | `<name>_end` | 休憩（`--rest` 秒） |

`_start`〜`_uncover` の区間は確実に条件が成立している。実際に遮られた瞬間は `_cover`〜`_start` の間にあり、ログの `SOB: drop` で特定する。

E2 の初回（`20261024-160112_occlude.otsrec`）は、被せる時間が足りず途中で中止した（マーカー `aborted_by_user`）。

解析:

```powershell
python tools\analysis\otsrec.py    <session.otsrec>   # summary.txt と CSV
python tools\analysis\plot_session.py <session.otsrec> [--device N] [--window T0 T1]
```

## 答えを出す問い

| # | 問い | 結果 |
|---|---|---|
| Q1 | ロスト中に `result` / `poseIsValid` は変わるか。変わるなら何 ms 後か | 判明（E2-1）。最後の光学サンプルから **約 250ms で `Calibrating_OutOfRange`**（`poseIsValid` は true のまま）、**2000ms で `poseIsValid=false`** |
| Q2 | IMU だけの区間の速度・加速度の特徴 | 判明（E2-1, E6-1）。静止状態なら 2 秒間でずれは 1.4cm 以下。移動中は約 250ms で速度推定が急に小さくなり、経路方向にはほぼ止まる一方、高さ・横方向に毎秒 2〜6cm ずれる。`vecAcceleration` は常に 0（Lighthouse は埋めない）。更新は約 4ms 間隔の一定リズムになる |
| Q3 | 2 秒リセット時に何が出力されるか（ジャンプ、`deviceIsConnected` など） | 判明（E2-1）。`poseIsValid=false` の仮ポーズ（driver 座標 (0,0,−2)）を 1 回送ったあと、**再ブートストラップまでポーズの更新が完全に止まる**（最長 14 秒）。`deviceIsConnected` は true のまま |
| Q4 | ログ（`SOB drop` / `Resetting tracking` など）とポーズ変化の時間差 | 判明（E2-1）。`Resetting tracking` と `poseIsValid=false` は同時刻、`BOOTSTRAPPED` と `Running_OK` 復帰も同時刻。`SOB: drop` は 7 秒程度覆い続けたときにだけ出る（同期信号の喪失）ので、短い遮蔽の検出には使えない |
| Q5 | 測定範囲超過（off scale）時の飛び方 | 判明（E5-1）。超過だけなら段差 5cm 以下でロストなし。強い衝撃では `unlikely gravity` で即座にリセットされ、0.1〜0.3 秒止まって再開時に最大 5cm 程度の段差 |
| Q6 | 実際の更新周期と `poseTimeOffset` | 一部判明（E1-1）。HMD は間隔 p50 1.0ms（約1kHz）、`poseTimeOffset` p50 −1.7ms。Index コントローラーは間隔 p50 1.5ms / p90 5.7ms で不規則（平均約440Hz）、`poseTimeOffset` p50 −6.2ms |
| Q7 | Space Calibrator のフックとの共存（どちらが先か、両方効くか） | 未。現在 SC のドライバはセーフモードでブロックされていて読み込まれない |

## 計測ログ

### E1-1: 静止 60 秒（2026-10-24 15:50, `20261024-155027_static.otsrec`）

- 機材: Beyond（LHR-1A2B3C01）、Index コントローラー 左（LHR-1A2B3C06）・右（LHR-1A2B3C04）、BS2.0 ×3。Tundra Tracker は電源オフ。
  コントローラーは記録開始から約 23 秒後に電源が入ったため、ブートストラップの様子も記録されている。
- フック: `_006` と `_005` は別の関数で、両方フックできた。**Lighthouse のポーズはすべて `_006` 経由**（`_005` 経由は 0 件）。
  ドライバログは 72 行取得でき、`LHR-xxxx C:` 形式の行でデバイスと対応付けられる。
- 状態の変化:
  - 電源投入直後（ブートストラップ前）: `Calibrating_OutOfRange`, `poseIsValid=false`, 位置 (0, 0, −2)、`poseTimeOffset` が約 −100 秒。
    0.6〜0.9 秒後の `BOOTSTRAPPED` ログと同時に `Running_OK` になる。
  - 安定動作中にも `Calibrating_OutOfRange` が短時間出る（HMD で 43ms と 0.5 秒）。このとき **`poseIsValid` は true のまま**。
- 静止時のノイズ（world 位置の標準偏差）: HMD 0.08〜0.15mm、左コントローラー 0.07〜0.11mm、右コントローラー 0.5〜1.0mm（置き方の違いと思われる）。
- 座標系（R3 の一部）:
  - Lighthouse の driver 空間の原点は BS **5D6E7F01**。`vecWorldFromDriverTranslation` がこの BS の raw 位置と一致する（ブートストラップに使われた BS）。
  - raw（universe）空間の原点は BS **5D6E7F03**。`lighthousedb.json` の `known_universes` で姿勢が単位行列になっている BS（10進 2544018812 = 0x5D6E7F03）と一致する。
  - このため、raw 座標では HMD の y が約 −1.4m になる（原点の BS が高い位置にあるため）。床基準への変換は、その後段でチェイパロンの設定によって行われる。

### E2-1: 右コントローラーの遮蔽（2026-10-24 16:04, `20261024-160435_occlude.otsrec`）

- 机に置いた右コントローラー（LHR-1A2B3C04）に布を被せる。1・3・5・10 秒 × 3 回。`--settle 3`。
- 機材: Beyond、Index 左右、Tundra ×2（LHR-1A2B3C02, LHR-1A2B3C05）。LHR-1A2B3C03 は記録されていない。
- 解析: `python tools\analysis\trials.py <session> --device LHR-1A2B3C04`

ロストの流れ（12 回とも同じ）:

| 光学ロストからの時間 | 出力 |
|---|---|
| 0〜約 250ms | `Running_OK`, `poseIsValid=true`（IMU だけで推測中だが、状態は OK のまま） |
| 約 250ms〜2000ms | `Calibrating_OutOfRange`, `poseIsValid=true`。静止状態ならずれは 1.4cm 以下 |
| 2000ms | ログ `Resetting tracking: no optical samples for 2000ms`。`poseIsValid=false` の仮ポーズを 1 回送る |
| 以降 | ポーズの更新が止まる。覆いを外してから 1.9〜3.5 秒後に `BOOTSTRAPPED` で再開 |

**吹っ飛びの再現（12 回中 5 回）**: 再開時のブートストラップには 2 種類あった。

- `BOOTSTRAPPED base 5D6E7F02 (delayed)`: 正しい位置（ずれ 0.1〜0.5cm）で再開し、そのまま安定。
- `BOOTSTRAPPED base 5D6E7F01 (immediate)`: 最初から 5〜8cm ずれた位置で再開する。`Running_OK` は 200〜270ms しか続かずに `Calibrating_OutOfRange` に落ち、
  **コントローラーは静止している（角速度ほぼ 0）のに、0.3〜0.5m/s の一定速度で直線的に流れ続ける**（主に下向き＝床を突き抜ける方向）。
  約 2 秒後に `Calibration failed: no optical samples from base 5D6E7F01 for 2003ms` でリセットされるまでに **50〜85cm** 流れた。
  これが何度か繰り返されることもある。机の上の置き方では、5D6E7F01 からはごく一部のセンサーしか見えていないと思われる。

**Phase 1 ルールの試算**: 「`Running_OK` かつ `poseIsValid` が N 秒続くまで位置を更新せず、直前の位置を保持する」を記録データに当てた結果（各試行の、置いた位置からの最大ずれ [cm]）:

| | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 生の出力 | 0.9 | 0.6 | 84.8 | 2.7 | 69.6 | 51.7 | 35.9 | 0.9 | 67.2 | 51.1 | 0.8 | 2.0 |
| N=0 | 0.5 | 0.1 | 7.5 | 0.2 | 7.7 | 7.6 | 25.1 | 0.1 | 14.0 | 6.0 | 0.3 | 0.6 |
| N=0.3 秒 | 0.5 | 0.1 | 0.1 | 0.2 | 0.1 | 0.1 | 19.1 | 0.1 | 0.1 | 4.6 | 0.2 | 0.1 |

N=0.3 秒で、悪いブートストラップによる吹っ飛びはすべて消える（0.5 秒・1 秒でも同じ）。
7 番は直前の試行の吹っ飛びが終わる前に始まっていて基準位置自体がずれており、10 番は布を被せたときに実際に 4.4cm 動いている。

**Q7（Space Calibrator）**: SC のドライバが読み込まれた状態で、こちらが記録した値（head 位置に変換）と vrserver 側の raw ポーズの差は全デバイスで 0.2mm 以下。
今の構成では SC は Lighthouse デバイスのポーズを変えていない。フックの重なる順番はこのデータからは判定できない。

### E6-1: 移動中の遮蔽（トンネル）（2026-10-24 16:22, `20261024-162244_timed.otsrec`）

- Tundra Tracker（LHR-1A2B3C03）を紙に載せて机の上を引き、タオルのトンネルを通過させる。5 回。
  `timed --label occlude_moving --duration 2 --settle 2`。経路は約 40cm の直線、移動速度は約 0.15〜0.3m/s。
- 机の上を滑らせているので、正しい高さは一定、正しい経路は「開始位置→最終位置」の直線。この制約からのずれを誤差とみなす。

光学ロスト後の IMU 推測（Lighthouse 自身の出力）:

| 試行 | ロスト時点の位置（経路上） | リセット時点（2 秒後） | 経路長 | 高さのずれ（最大） | 横方向のずれ（最大） |
|---|---|---|---|---|---|
| 1 | 18.4cm | 23.7cm | 43.7cm | 12.4cm | 12.6cm |
| 2 | 14.8cm | 28.1cm | 44.9cm | 4.5cm | 7.0cm |
| 3 | 20.1cm | 27.0cm | 40.6cm | 7.8cm | 11.7cm |
| 4 | 10.3cm | 18.9cm | 38.6cm | 4.4cm | 1.4cm |
| 5 | 17.7cm | 22.2cm | 45.3cm | 3.7cm | 1.0cm |

- 光学ロストから約 250ms で速度の推定値が急に小さくなり（0.15〜0.3 → 0.02〜0.1m/s）、**実際はまだ動いているのに位置はほとんど進まなくなる**。
  一方で高さと横方向は毎秒 2〜6cm の割合でずれ続ける。E2 のような 50cm 以上の吹っ飛びは起きなかった。
- つまり移動中のロストでも、Lighthouse の IMU 推測は経路方向にはほぼ止まっており、それ以外の方向に誤差を積み上げているだけ。
  **ロストを検出した時点で位置を固定する方が、2 秒間 IMU 推測を流すより誤差が小さい**（高さ・横方向のずれは検出時点の 3cm 以下で止まる）。
- 再開はすべて `BOOTSTRAPPED base 5D6E7F01 (immediate)` で、デバイスを引き出している最中だったが、その後に流れる現象は起きなかった。

**早期検出の手がかり（ポーズ更新のリズム）**: 光学データがあるときは、IMU による一定間隔の更新に光学による更新が割り込むため、
更新間隔の約 45% が 1ms 未満になる（平均 約 450Hz）。光学データが途切れると、即座に約 4ms 間隔の一定リズム（約 250Hz）だけになる。
「直近 50ms に 1ms 未満の更新間隔が 1 つもない」を検出条件にした試算:

| 記録 | デバイス | 光学ロストからの検出遅れ | 通常追跡中の誤検出（ポーズ数比） |
|---|---|---|---|
| E2-1 | 右コントローラー（机の上） | 71〜136ms（11 回） | 0.36% |
| E2-1 | 左コントローラー | — | 0% |
| E2-1 | Tundra LHR-1A2B3C02 | — | 0.07% |
| E6-1 | Tundra LHR-1A2B3C03（移動中） | 72〜85ms（5 回） | 0% |
| E6-1 | Tundra LHR-1A2B3C02 / 右コントローラー | — | 0% |

`result` が変わる 250ms より 2〜3 倍早く気づける。誤検出は、一部のセンサーしか見えていない置き方で短時間だけ起きる。
確定のロスト判定ではなく「疑い」として使い、光学由来の更新が戻ればすぐ解除する使い方なら実用になりそう。
ただし、この間隔はベースステーション 2.0 の掃引周期に依存している可能性があるので、BS1.0 の環境や BS の台数が違う場合は別途確認が必要。

### E3-1: ベースステーションを 1 台ずつ遮る（2026-10-24 16:39, `20261024-163912_bs_block.otsrec`）

- 全デバイスを置いたまま、BS の正面を段ボールで 10 秒遮る。各 BS 2 回。解析: `python tools\analysis\static_conditions.py <session>`
- ログの `SOB: drop` から、ユーザーの呼び名との対応は **A = 5D6E7F02、B = 5D6E7F01、C = 5D6E7F03（universe 原点）**。
- 結果: どの BS を遮っても、全デバイスが `Running_OK` のままで、ポーズの途切れもなし。
  遮っている間の平均位置のずれは 0.0〜1.7mm、ノイズ（標準偏差）は 0.1mm 前後から最大 1.0mm 程度に増える程度。
- B（5D6E7F01）を遮った 1 回目のあと、全デバイスが同時に約 5mm ずれた。2 回目のあとは元に戻った。
  BS の位置関係の再推定（universe 全体の補正）によるものと思われる。デバイス単位のロストではないので、Phase 1 の対象外。
- 結論: BS が 3 台ある環境では、1 台が見えなくなってもトラッキングの問題はほぼ起きない。問題になるのは、デバイスから見える BS が 0 台になったとき（E2・E6）。

### E5-1: 衝撃と速い動き（`20261024-165535_impact.otsrec` clap/swing、`20261024-170712_impact.otsrec` stomp）

- 解析: `python tools\analysis\motion_trials.py <session> --name impact`
- clap/swing の記録は、途中のクラッシュ（下記）でファイル末尾が欠けているが、6 試行分は完全に残っている。

| 条件 | IMU went off scale | ロスト | 補正による位置の段差（最大） |
|---|---|---|---|
| clap（コントローラー 2 本 × 3 回） | 6 回中 5 回 | 2 回、`Resetting tracking: unlikely gravity 10.3 m/s/s` | 5.4cm（リセット後の再ブートストラップ時） |
| swing（2 本 × 3 回） | 6 回中 4 回 | なし | 4.7cm |
| stomp（足 2 台 × 3 回、腰は対象外） | 6 回中 3 回 | なし | 1.4cm |

- **衝撃では別の理由でリセットされる**: 光学ロストの 2 秒を待たずに、`unlikely gravity`（重力の大きさが不自然）で即座にリセットされ、
  `poseIsValid=false` のあと 96〜270ms ポーズが止まってから再開する。再開時に最大 5cm 程度の段差が出る。
- 測定範囲超過だけ（リセットなし）なら、ロストもせず、段差も 5cm 以下。速い動きの最中の通常の補正と区別できない程度。
- 訂正（Phase 1 の作業中に判明）: 最初の版の段差の計算は、driver 座標の速度をそのまま world 座標の位置に足していた。
  速く動くデバイスほど過大に出ていたので、world 座標に回転してから計算し直した値に差し替えた（clap 15.6cm → 5.4cm、stomp 2.8cm → 1.4cm）。
  E4 の表の値は、動きが遅い区間で起きた段差なので、ほぼ変わらない（右足 16.5cm → 16.7cm）。

### E4-1: 体の陰になる姿勢（`20261024-170855_body.otsrec`）

- 解析: `python tools\analysis\motion_trials.py <session> --name body`（装着位置は下表）
- `jump` = 連続したポーズの間で、報告された速度では説明できない位置の段差。`reacq` = ポーズが 50ms 以上止まった前後の位置の差。

| 姿勢 | 主にロストしたデバイス | ロスト時間 | 位置の段差（最大） |
|---|---|---|---|
| 前屈 ×3 | 腰（毎回）、HMD（1 回 1.15 秒）、右手（1 回） | 腰 6.4〜7.8 秒（2 秒でリセット → 立ち上がってから再開） | 腰の再開時 21〜41cm、HMD 10.9cm、右手 21.6cm |
| しゃがむ ×3 | 腰（毎回）、右手（2 回） | 腰 6.7〜7.6 秒、右手 2.9〜5.4 秒 | 腰の再開時 38〜72cm、右手 22.5cm |
| あぐら ×3 | 右手（毎回）、腰・足（短時間） | 0.15〜5.5 秒（リセットなしが多い） | **腰 43.7cm**、右手 22.0cm、右足 16.5cm |
| 横向きに寝る ×3 | 右手・腰・足（短時間） | 0.2〜1.4 秒 | **右手 33.7cm**、腰 9.1cm |

- 前屈・しゃがみの腰は、体の陰で毎回 2 秒以上光が途切れてリセットされ、立ち上がってから再開するまで合計 6〜8 秒ロストした。
  再開時の段差には実際の動き（立ち上がり）も含まれるので、全部が誤差ではない。
- **あぐら・横向きに寝るでは、リセットまで行かない短いロスト（0.2〜5 秒）が多く、光が戻った瞬間に 10〜44cm の段差が出た**。
  連続したポーズの間で起きるので、アプリからは「瞬間移動」に見える。Phase 1 で復帰時に補間すれば消せる種類のずれ。
- 左手（LHR-1A2B3C06）は全試行でロストなし。右手が多いのは、姿勢をとるときに右手を体の近くや床側に置いていたためと思われる。

### デバイスの装着位置（E4・E5 用）

| シリアル | 装着位置 |
|---|---|
| LHR-1A2B3C05 | 腰 |
| LHR-1A2B3C02 | 左足の甲 |
| LHR-1A2B3C03 | 右足の甲 |

### vrserver のクラッシュ 2 件（2026-10-24 16:47 / 16:58）と修正

| 時刻 | 例外の場所 | 状況 |
|---|---|---|
| 16:47 | `driver_01spacecalibrator.dll+0x1ec7`（アドレス 0x1 の読み込み） | Space Calibrator 有効。記録なし |
| 16:58 | `driver_trackingstabilizer.dll` の memcpy（アドレス 0x0 の読み込み） | Space Calibrator 無効。E5 の記録中、Tundra の電源を入れた直後 |

- どちらも呼び出し元は **BeyondSteamVR ドライバのスレッド** で、vrserver の `IVRServerDriverHost_005` の vtable 1 番目の関数 → こちらのフック `PoseDetour<1>`（`_005` 用）を経由していた。
  16:58 はこちらのフックが、16:47 はその先の Space Calibrator のフックが、3 番目の引数をポーズとして読んで落ちた。
- 原因: vrserver の `IVRServerDriverHost_005` の vtable 1 番目は、実際には `TrackedDevicePoseUpdated` ではない別の関数で、BeyondSteamVR が 3 番目の引数を null（または 1）で呼ぶ。
  ポーズとして扱うフックは、この呼び出しで不正なアドレスを読む。こちらは記録中だけ読むので、記録中に Beyond がこの関数を呼んだときにだけ落ちた。
  Space Calibrator も `_005` をフックしているので、同じ取り違えで落ちていると考えられる（この日の最初からセーフモードでブロックされていたのも同じ原因の可能性が高い）。
- 解析方法: Steam の `dumps\crash_vrserver.exe_*.dmp` から例外アドレスとスタック上の戻りアドレスを取り出し、同じソースを `/MAP` 付きでビルドしたマップで関数名に変換した（`.text` が一致することを確認済み）。
- 修正: `_005` のフックを削除（Lighthouse のポーズは E1〜E6 ですべて `_006` 経由で、`_005` 経由は 0 件）。
  フックではポーズをポインタで受け取り、null・サイズ不足・デバイス番号範囲外のときは記録しないようにした。
- Space Calibrator のドライバは無効のままにしておく（`driver_01spacecalibrator.enable=false`）。Phase 1 では、読み込まれていたら警告を出す。

## Phase 1 への示唆（案）

1. **ロスト判定**: `result != Running_OK` または `poseIsValid=false` でロストとみなす（光学ロストから約 250ms で検出できる）。
   早期検出として、更新リズムの変化（直近 50ms に 1ms 未満の更新間隔がない）を「疑い」に使う（光学ロストから約 70〜130ms）。
2. **ホールド**: ロスト中は位置を直前の確定位置に固定し、速度・加速度を 0 にする。回転は IMU の値をそのまま流す。
3. **ポーズ停止中の補完**: 2 秒リセット以降は Lighthouse がポーズを送らなくなるので、こちらからホールド中のポーズを定期的に送る必要がある（`poseIsValid=true` で送るかは設定にする）。
4. **復帰の保留期間**: `Running_OK` が 0.3〜0.5 秒続くまでは復帰させない（悪いブートストラップを捨てる）。
5. **復帰時の補間**: 保持位置から新しい位置へ 100〜300ms かけて滑らかに移す。
6. 移動中のロスト（E6-1）でも、検出時点で位置を固定する方が IMU 推測を流すより誤差が小さい。
   ただし固定すると、実際には動いているデバイスがその場に止まって見える。経路方向へ少しだけ外挿して減速させる（直前の光学速度を 100〜300ms で 0 に落とす）案は Phase 1 で比較する。
7. 生の IMU（加速度）が取れれば「静止しているのに速度が出ている」矛盾を直接検出できる（E2 の吹っ飛び対策の補強）。
8. **復帰時の補間は、リセットを伴わない短いロストにも必要**（E4-1 のあぐら・横向きで 10〜44cm の瞬間移動）。
   `Running_OK` 以外から戻るときは、保留期間のあと必ず補間する。
9. 衝撃による即時リセット（`unlikely gravity`、E5-1）も、`poseIsValid=false` と更新停止で検出できるので、同じ保持・復帰の仕組みで扱える。

## Phase 0 のまとめ

| 完了条件 | 状態 |
|---|---|
| 計測ドライバを入れたまま普段どおり遊べる | 一部。`_005` フックによるクラッシュを修正済み（修正版で E4・E5 を問題なく計測）。長時間の常用はまだ |
| E1〜E7 のデータ | 完了（E7 は E2・E6 の復帰部分で確認） |
| Q1〜Q7 の答え | Q1〜Q6 完了。Q7 は Space Calibrator 側の問題が判明したため無効化で決着 |
| Phase 1 のロスト判定の案 | 完了（上の「Phase 1 への示唆」） |

未着手: 逆アセンブルによる解析（R1〜R5）、生の IMU データの取得。どちらも Phase 1 の試作と並行して進められる。

## 解析タスク（静的: Ghidra / 動的: x64dbg）

改変はしない。結果は `docs/re-notes.md` に記録し、Valve のコードやバイナリそのものは含めない。

| # | 内容 | 結果 |
|---|---|---|
| R1 | `Switching Lighthouse -> IMU Only` と `Resetting tracking: no optical samples for %dms` の参照元から、状態を切り替える関数・条件・定数（2000ms など）を特定する | 未 |
| R2 | `enableImuFallback` / `disableimu` / `disableimuexcepthmd` の意味と適用範囲（文字列の並びからは HMD 初期化周りと推定） | 未 |
| R3 | `lighthousedb.json` の姿勢データ（7 要素）の並び順と座標系、universe の解決方法。RawPoses のベースステーション姿勢と突き合わせる | 一部判明（E1-1）。下記「計測ログ」参照 |
| R4 | 内部のロスト状態を直接読み取れる・フックできる場所の候補（更新で壊れる前提で、パターン検索で見つける方式にする） | 未 |
| R5 | `lighthouse_console.exe` のコマンド一覧 | 未 |

vrserver にはスレッド監視機能があり、停止すると強制終了する（デバッガ接続中は強制終了しない、という文字列あり）。
動的解析は記録用のセッションとは分けて行う。

## 完了条件

1. 計測ドライバを入れたまま普段どおり遊んで、遅延・フレーム落ち・クラッシュがない
2. E1〜E7 のデータが取れている
3. Q1〜Q7 の答えがこのファイルに記入されている
4. Phase 1 のロスト判定方法（使う信号としきい値）の案が決まっている

## 未対応（Phase 0 中に追加検討）

- 生の IMU データ：Lighthouse は `IVRIOBuffer_002` で IMU サンプルを流しているらしい。パスが分かればクライアントから読める
- アプリ側から見えるポーズ（`GetDeviceToAbsoluteTrackingPose`）の同時記録。Space Calibrator の補正や SteamVR の予測がかかった後の値との比較用
