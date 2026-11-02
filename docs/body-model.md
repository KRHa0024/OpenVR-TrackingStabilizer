# 遮蔽中だけ腰を身体モデルで補完（実験的）

3点構成でロストした手を補完する機能は、独立した [手の遮蔽補完](hand-model.md) を参照。

通常時は Lighthouse の実測を使い、明示指定した腰トラッカーがロストした間だけ位置を推定する。
設定画面の **「遮蔽中の腰を身体モデルで補完」** で切り替える。既定は **オフ**。
オフでは従来の位置保持を使う。全体の補正スイッチ、腰のデバイス別補正スイッチも有効にする必要がある。
HMD は補完の基準として読むだけなので、「HMD も補正する」をオンにする必要はない。

## 使い方

1. 設定画面で身体モデルのトグルをオンにする。
2. 腰に装着しているトラッカーと、使用中の HMD を選ぶ。シリアルで保存する。
3. 両方をベースステーションから見える状態にして、1秒以上まっすぐ立つ。
4. 「学習済み」と寸法が表示されたら、腰を短時間遮り、少し移動・前屈して比較する。
5. 同じ動きをトグルのオフでも確認する。オンが不自然な姿勢ではオフに戻せる。

`Prop_ControllerRoleHint_Int32 == 3` は腰の判別に使わない。
足・膝などを番号順や高さから腰だと推定する処理も入れていない。
取り付け位置を変えたときは、トグルをオフ→オンにして再学習する。

## 実装範囲

頭―首の短いセグメントと、首―腰の1本の剛体セグメントからなる小さな身体モデル。
全身 IK、膝関節、足の接地固定、足の持ち上げ判定は含まない。

world 座標で、概ね次の計算をする（`R` は回転、`p` は位置）。

```
p_head = p_HMD_device + R_HMD_device * driverFromHead.translation
R_head = R_HMD_device * driverFromHead.rotation
p_neck = p_head + R_head * (0, -neckDown, +neckBack)
torso_local = inverse(R_waist) * (p_neck - p_waist)  // 正常時に学習
p_waist_estimated = p_neck - R_waist * torso_local + onset_residual
```

`torso_local` の長さと方向を実測から学習するので、腰トラッカーの取り付け方向を固定しない。
腰の回転によって推定位置が変わる点が、HMD の移動量をそのまま足す相対追従との違い。
ただし胴体を1本の剛体で近似するため、首・背骨の独立した動き、座位や寝姿勢などでは誤差が出る。
正常時に身体モデルを出力へ適用する処理はなく、既存のジッター補正・段差補正だけが働く。

### 信頼度と保持へのフォールバック

- 学習には同時に正常な腰・HMD の観測が20回以上、0.5秒以上必要。推定した首―腰の長さは25～85cmに制限する。
- 学習中の局所ベクトルが8cmを超えて変わると学習をやり直す。学習後も正常な観測と8cm以上食い違えば再学習する。
- HMD の正常な実測と、連続性を確認できた腰の回転が両方とも100ms以内に必要。
- 回転は、前回採用した回転からの変化が `0.15 + 8 * dt` rad（dt は最大 0.1 秒で計算）以内なら採用する。無効・非有限・非単位の回転は拒否する。
  - ロスト中（範囲外の IMU 回転）は、超えた回転を一時的な乱れとみなす。ただし採用が 0.1 秒以上途切れたら、その時点の回転をそのまま採用し直す（1 回の途切れでそのロストの間ずっと回転が止まることはない）。
  - SUSPECT と PROBATION の正常な回転も同じ条件で採用する。超えた分は再ブートストラップによる補正とみなして保留し、BLEND でなめらかに合わせる。
- 遮蔽開始時のモデルと保持位置の差が10cm以内なら、その小さな残差を固定して開始位置を合わせる。10cmを超えた回は身体モデルを使わない。
- 推定位置と現在の保持位置が35cmを超えて離れた場合も採用しない。採用する移動は最大1.5m/s、1回5cmまで。
- 条件を満たさなくなったら、**その時点の出力位置**で保持する。遮蔽開始地点への巻き戻しはしない。
- 腰またはHMDの明示的な切断、トグル変更、基準の割り当て変更は学習を破棄する。

無効な入力を1回も送らず腰の更新だけが止まるケースも、オンの腰に限り更新が 0.1 秒（`resetGap`）途切れたら HOLD に入る。それより短い途切れ（USB や無線の一瞬の詰まり）はロストとみなさない。
ただし腰の回転が100ms以上途絶えた後は、HMD が動いても位置推定を続けない。Lighthouse はロストから約 2 秒で「Resetting tracking」を出し、以後は光学が戻るまで何も送らないので、今は推定できるのはロスト後およそ 2 秒まで。生 IMU（`/devices/lighthouse/<シリアル>/imu` の IOBuffer）から自前で回転を出す仕組みを検討中。
既定の早期検出 `suspectEnabled=false` は変更していないため、ロスト通知前の IMU 推測区間を完全に検知するものではない。

### 光学復帰

既存の HOLD → PROBATION → BLEND を使う。
オンの腰では、復帰待ち中に1更新で8cm以上の位置変化または0.35rad以上の回転変化があれば待ち時間をやり直す。
復帰待ち中は復帰直後の回転を直接出さず、BLEND で world 回転を最短経路の slerp で実測へ戻す。
補間中の角速度・角加速度は0にして SteamVR の予測による復帰補正の先走りを抑える。
復帰後の正常な実測が継続しても姿勢の正しさを外部から証明できるわけではなく、外部の ground truth は使っていない。

## 設定と記録

設定画面の操作は既存と同じく即時反映し、`steamvr.vrsettings` の `driver_trackingstabilizer` に保存する。
CLI の `set` は従来と同じく実行中だけの変更。

| キー | 既定 | 意味 |
|---|---|---|
| `bodyEnabled` | `false` | 身体モデルのオン・オフ |
| `bodyWaistSerial` | 空 | 腰のシリアル（GenericTracker のみ） |
| `bodyHmdSerial` | 空 | 基準 HMD のシリアル |
| `bodyNeckDownCm` | `12` | HMD の頭座標原点から首までの下方向成分、0～30cm |
| `bodyNeckBackCm` | `8` | 同じく後方向成分（+Z）、0～30cm |

寸法の2つの設定は CLI / vrsettings で調整できる。首―腰の長さ自体は学習値で、固定の身長を仮定しない。

```
ots-cli set bodyWaistSerial <腰のシリアル>
ots-cli set bodyHmdSerial <HMDのシリアル>
ots-cli set bodyEnabled true
ots-cli devices
ots-cli set bodyEnabled false
```

記録形式のバージョンは変更しない。生の `Pose` と実際に送った `FilteredPose` は従来どおり保存する。
開始時の Note に全設定とデバイス状態を保存し、設定変更の Note と身体モデルの状態変更の Note を追加した。
`devices` に `bodyReady` / `bodyActive` / `torsoLengthCm` を追加した。
古い解析ツールでも Note として読める。

## リプレイと擬似遮蔽評価

リプレイでは記録内 `DeviceInfo` の **その録画での番号**を明示する。
番号は接続ごとに変わり得る。番号とクラスが一致しなければ身体モデルは起動しない。

```powershell
# 番号は例。対象録画の DeviceInfo で腰と HMD を確認して置き換える。
build\tools\ots-replay\Release\ots-replay.exe input.otsrec -o output.otsrec `
  --body-waist 7 --body-hmd 0 --set bodyEnabled=1

# 腰の回転は残して、10～11秒、20～21秒の位置を疑似的に遮蔽する。
build\tools\ots-replay\Release\ots-replay.exe input.otsrec -o masked.otsrec `
  --body-waist 7 --body-hmd 0 --set bodyEnabled=1 --mask 10:11 --mask 20:21

# 正常な観測が続く区間から1秒の遮蔽区間を自動作成し、3方式を比較する。
python tools\analysis\eval_body_model.py input.otsrec `
  --waist <腰のシリアル> --hmd <HMDのシリアル> --out build\body-eval
```

比較ツールは world 固定、HMD の平行移動だけの追従、身体モデル＋保持へのフォールバックを比較し、
`summary.txt` / `errors.csv` / 2方式のリプレイ録画を出力する。
原記録は書き換えない。リプレイで首の設定を変更する場合のキーは `bodyNeckDown` / `bodyNeckBack`（**m単位**）。
リプレイは指定した設定での比較用であり、記録内の設定変更を自動適用しない。
学習状態も新規に作るため、記録開始前に済んでいた学習や、途中でのトグル変更を含むライブ出力の完全再現ではない。

### ローカル検証結果（2026-10-25）

`LHR-1A2B3C05` を比較対象、`LHR-1A2B3C01` をHMDとして明示して実行した。
過去録画の DeviceInfo には腰の装着部位が記録されていないので、これは**指定したトラッカーでの比較**であり、装着部位の独立確認ではない。
1秒の擬似遮蔽、学習・復帰の間隔2秒、両方の入力が正常な区間だけで評価した。

| 録画 | 遮蔽区間数 / サンプル | world 固定 平均 / p95 | HMD 平行移動 平均 / p95 | 身体モデル＋保持 平均 / p95 |
|---|---:|---:|---:|---:|
| `20261024-170855_body` | 103 / 40,511 | 2.82 / 10.19cm | 3.44 / 16.50cm | 2.42 / 9.52cm |
| `20261024-182503_body` | 110 / 46,254 | 2.90 / 14.21cm | 3.95 / 15.91cm | 2.29 / 8.79cm |

モデルが有効だった評価サンプルは概算87.0% / 91.1%。残りは保持など。
最大誤差は身体モデルでも46.63 / 44.04cm残ったため、全姿勢で改善した、あるいは長い遮蔽でも正しいとは結論しない。
参照値は正常扱いの Lighthouse 実測であり、外部計測の真値ではない。
擬似遮蔽は pose 回転を残すため、実際の遮蔽でその回転が取得できるかは評価できない。

Release ビルド、既存 filter テスト、新規身体モデルテスト、模擬vrserverのドライバハーネス、
バイナリ記録を介したリプレイ統合テスト、日英の設定画面の画像生成を実施した。

```powershell
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
python tools\analysis\test_body_replay.py build\tools\ots-replay\Release\ots-replay.exe
```

## 実機への反映と未検証事項

ローカルのビルド成果物は `build/package/trackingstabilizer` と `build/tools/ots-overlay/Release`。
今回インストール済みドライバの差し替え、SteamVR の再起動、push、公開は行っていない。
反映する際は使用を終えて SteamVR を終了し、既存の `scripts/install-driver.ps1` でドライバをコピー・登録する。
SteamVR 起動後に `build/tools/ots-overlay/Release/ots-overlay.exe` を起動すると、ビルドした管理画面に登録が切り替わる。
インストール版とビルド版を同時には起動しない。詳細は README の「開発中のインストール」を参照。
その後、トグルの保存・再起動後の復元、両方が見える状態、腰だけの遮蔽、両方の遮蔽、再接続を実機で確認する。

利用しているのは `DriverPose_t` の回転であり、生IMUではない。
IVRIOBuffer の読み取りは実装しておらず、Tundra / Index で生IMUが取得できるか、
長い遮蔽・Lighthouse のリセット後も回転が継続するかは未確認。
加速度の二重積分で長時間の位置を保持できるとは仮定しない。
