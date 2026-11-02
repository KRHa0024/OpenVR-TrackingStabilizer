#include "UiText.h"

#include <unordered_map>

namespace ots {
namespace {

struct Entry {
    const char* key;
    const char* en;
    const char* ja;
};

// Keep the wording plain: one term for the feature (補正 / correction), ロスト for lost tracking.
const Entry kEntries[] = {
    {"lang", "en", "ja"},
    // header and tabs
    {"recordingBadge", "Recording", "レコーディング中"},
    {"tabMain", "Correction", "補正"},
    {"tabDevices", "Devices", "デバイス"},
    {"tabSettings", "Settings", "設定"},
    {"warnNoVr", "SteamVR is not running", "SteamVR が起動していません"},
    {"warnNoDriver", "Driver not connected", "ドライバに接続できません"},
    // correction screen
    {"correction", "Correction", "補正"},
    {"on", "ON", "ON"},
    {"off", "OFF", "OFF"},
    {"targetsLine", "Correcting {0} of {1} devices", "補正の対象 {0} / {1} 台"},
    {"heroOff", "Lost devices may fly away", "ロストすると位置が飛ぶことがあります"},
    {"statDevices", "Devices", "現在のデバイス数"},
    {"statHolding", "Correcting", "補正中"},
    {"session", "Corrections this session: {0}", "このセッションの総補正回数 {0} 回"},
    {"laneTitle", "Last 20 s", "直近 20 秒"},
    {"lanesEmpty", "No devices to show yet.", "表示できるデバイスがまだありません。"},
    {"lanesNoTrace", "The graph needs the updated driver.", "グラフの表示には新しいドライバが必要です。"},
    {"legendOut", "Corrected", "補正後"},
    {"legendRaw", "Raw", "生の値"},
    {"legendHeld", "Correcting", "補正中"},
    {"legendLost", "Uncorrected", "未補正"},
    {"now", "now", "現在"},
    {"drift", "drift {0} cm", "ずれ {0} cm"},
    {"map", "Map", "マップ"},
    {"mapView", "Map view", "マップの表示"},
    {"mapDefault", "Initial map view", "マップの初期表示"},
    {"mapTop", "Top view", "上から"},
    {"mapSide", "Side view", "横から"},
    {"map3d", "3D view", "3D"},
    {"mapHint", "Drag to orbit · Scroll to zoom", "ドラッグで回転 / ホイールで拡大"},
    {"mapHintPan", "Drag to move · Scroll to zoom", "ドラッグで移動 / ホイールで拡大"},
    {"toolOrbit", "Rotate", "回転"},
    {"toolPan", "Move", "移動"},
    {"stTracking", "Tracking", "追跡中"},
    {"stHold", "Correcting", "補正中"},
    {"stVerify", "Checking return", "復帰を確認中"},
    {"stReturn", "Returning", "復帰中"},
    {"stLost", "Lost (uncorrected)", "ロスト（未補正）"},
    {"stOff", "Not tracking", "追跡していません"},
    {"notTarget", "Not corrected", "補正の対象外"},
    {"notLighthouse", "Not a Lighthouse device", "Lighthouse のデバイスではありません"},
    {"hmdLocked", "Enable in Settings → Also correct the HMD", "設定の「HMD も補正する」で有効にできます"},
    {"lostFor", "Lost for {0} s", "{0} 秒ロスト"},
    {"rawPos", "Raw position", "生の位置"},
    {"baseStation", "Base station", "ベースステーション"},
    {"log", "Log", "ログ"},
    {"logFilterBtn", "Filter", "絞り込み"},
    {"logDriver", "Lighthouse driver log", "Lighthouse ドライバのログ"},
    {"logDriverHelp", "Messages from SteamVR's own tracking driver", "SteamVR 純正のトラッキングドライバが出すメッセージ"},
    {"logOts", "Stabilizer driver log", "Stabilizer ドライバのログ"},
    {"logOtsHelp", "What this tool's driver is doing (connection, recording…)", "このツールのドライバの動作（接続・レコーディングなど）"},
    {"logFilter", "Correction changes", "補正の状態の変化"},
    {"logFilterHelp", "When correction starts on a lost device, and when it comes back", "ロストしたデバイスの補正の開始や、復帰などの切り替わり"},
    {"logUi", "Actions in this window", "この画面での操作"},
    {"logUiHelp", "Settings changes and other actions taken here", "設定の変更など、ここで行った操作"},
    // devices screen
    {"devices", "Devices", "デバイス"},
    {"colDevice", "Device", "デバイス"},
    {"colCorrect", "Correct", "補正"},
    {"colBattery", "Battery", "電池"},
    {"colStatus", "Status", "状態"},
    {"colHolds", "Corrections", "補正回数"},
    {"baseStations", "Base stations", "ベースステーション"},
    {"colSerial", "Serial", "シリアル"},
    {"colHeight", "Height", "高さ"},
    {"colTilt", "Tilt", "下向き"},
    {"colState", "State", "状態"},
    {"bsSeen", "Seen", "認識中"},
    {"imuToggle", "Keep rotation from the tracker's IMU after lighthouse gives up (experimental)",
     "Lighthouse が止まった後も、トラッカーの IMU で回転を続ける（実験的）"},
    {"imuHelp", "About 2 s into an occlusion lighthouse stops sending the device's pose. With this on, the held pose keeps "
                "turning with the device's own gyro, and the waist / hand estimation keeps going. Each device first learns its "
                "IMU while visible: move it around a little.",
     "遮蔽から約 2 秒で Lighthouse はデバイスの姿勢を送らなくなります。オンにすると、保持中の姿勢がデバイス自身のジャイロで回り続け、"
     "腰や手の補完も続きます。各デバイスは見えている間に IMU の向きを学習するので、最初に少し動かしてください。"},
    {"imuOff", "IMU: off", "IMU：オフ"},
    {"modelOnMap", "Show the body model on the map", "マップに体モデルを表示する"},
    {"modelInVr", "Show the body model around you in VR (debugging)", "VR 空間に体モデルを表示する（デバッグ用）"},
    {"imuReady", "IMU: ready on {0}", "IMU：準備完了 {0} 台"},
    {"imuLearning", "· learning on {0} (move them a little)", "・学習中 {0} 台（少し動かしてください）"},
    {"imuFailed", "· cannot read {0} (error {1})", "・読めない {0} 台（エラー {1}）"},
    {"imuActiveNow", "· in use now on {0}", "・いま使用中 {0} 台"},
    {"toastImuOn", "IMU rotation on", "IMU による回転をオンにしました"},
    {"toastImuOff", "IMU rotation off", "IMU による回転をオフにしました"},
    {"bsUnseen", "Not seen", "見えていない"},
    // settings screen
    {"filterTitle", "Correction settings", "補正の設定"},
    {"filterNote", "Changes apply right away.", "変更はすぐに反映されます。"},
    {"changeTargets", "Choose devices to correct", "補正するデバイスを選ぶ"},
    {"allowHmd", "Also correct the HMD", "HMD も補正する"},
    {"allowHmdWarn", "The view may move unexpectedly, which can cause VR sickness.", "視界が予期せず動くことがあり、VR 酔いにつながるおそれがあります。"},
    {"groupJitter", "Small shakes", "小刻みな揺れ"},
    {"bodyTitle", "Body model (experimental)", "身体モデル（実験的）"},
    {"handToggle", "Follow the shoulder when a hand is occluded", "遮蔽中の手を肩に追従させる"},
    {"handHelp", "For headset + controllers: keep the last hand position relative to its shoulder. Left/right roles come from SteamVR. Choose the reference HMD below. Head turns do not rotate the hidden hand. This cannot reconstruct unseen arm motion. Dimensions and maximum prediction time are adjustable below.",
     "HMDと両手の構成で、最後の手位置を肩に対して保持します。左右はSteamVRの役割を使います。下で基準HMDを選んでください。頭を横に向けても手は一緒に回しません。見えない腕振りは再現できません。寸法と推定時間は下で調整できます。"},
    {"handOff", "Hand model off: use the existing position hold", "手の補完オフ：従来の位置保持を使用"},
    {"handHeadNeeded", "Choose the reference HMD", "基準HMDを指定してください"},
    {"handRolesNeeded", "Waiting for SteamVR left/right controller roles", "SteamVRの左右コントローラーの役割を待機中"},
    {"handWaiting", "Waiting for healthy HMD and hand tracking / hand correction", "正常なHMD・手の実測と、手の補正有効化を待機中"},
    {"handReady", "Hand model ready or holding", "手の補完準備済み、または位置保持中"},
    {"handEstimating", "Following the shoulder during hand occlusion", "遮蔽中の手を肩に追従中"},
    {"groupHand", "Hand occlusion model", "手の遮蔽補完"},
    {"pHandShoulder", "Shoulder width", "肩幅"},
    {"pHandShoulderHelp", "Distance between the estimated left and right shoulder pivots.", "左右の肩の支点の間隔です。"},
    {"pHandReach", "Maximum arm reach", "腕の最大到達距離"},
    {"pHandReachHelp", "Shoulder to controller origin, including the hand. An out-of-reach pose uses position hold.", "肩からコントローラー原点までの距離です。手の長さも含めます。範囲外の姿勢では位置保持を使います。"},
    {"pHandDuration", "Maximum prediction time", "推定を続ける時間"},
    {"pHandDurationHelp", "After this time, freeze the current output until optical tracking returns.", "この時間を過ぎたら、その時点の位置で光学復帰を待ちます。"},
    {"bodyToggle", "Estimate the waist only during occlusion", "遮蔽中の腰を身体モデルで補完"},
    {"bodyHelp", "Choose the tracker worn at your waist and your HMD. Stand upright with both visible for at least 1 second to learn the neck-to-waist segment. Healthy tracking passes through. Missing or unreliable input falls back to holding the current position. Keep waist correction enabled in Devices.",
     "腰に装着したトラッカーとHMDを選び、両方が見える状態で1秒以上まっすぐ立つと、首から腰までの寸法を学習します。正常時は実測を使い、入力が古い・不確かな場合はその場で保持します。デバイス画面で腰の補正もオンにしてください。"},
    {"bodyWaist", "Waist tracker", "腰トラッカー"},
    {"bodyHmd", "Reference HMD", "基準HMD"},
    {"bodyClear", "Clear", "解除"},
    {"bodyUnset", "Not assigned", "未指定"},
    {"bodyHmdAutoLabel", "{0} (automatic: the only HMD)", "{0}（自動：接続中の HMD）"},
    {"bodyOff", "Off: use the existing position hold", "オフ：従来の位置保持を使用"},
    {"bodyMappingNeeded", "Choose both the waist tracker and HMD", "腰トラッカーとHMDを指定してください"},
    {"bodyLearning", "Waiting for stable healthy observations / waist correction", "安定した実測・腰の補正有効化を待機中"},
    {"bodyReady", "Calibrated: tracking or holding", "学習済み：実測または位置保持中"},
    {"bodyEstimating", "Estimating waist position", "腰位置を身体モデルで補完中"},
    {"groupLoss", "Recovering from tracking loss", "トラッキングロスト時の復帰"},
    {"groupWarp", "Warps (sudden jumps)", "ワープ（瞬間移動）"},
    {"pSmoothing", "Jitter smoothing", "ジッター補正"},
    {"pSmoothingHelp", "Calms small, rapid shakes so movement looks smooth. Higher is smoother but adds a little lag. 0 = off.",
     "小刻みな揺れを抑えて、動きを滑らかにします。強くするほど滑らかになりますが、動きがわずかに遅れます。0 でオフ。"},
    {"pProbation", "Wait after a reset", "復帰を待つ時間（リセット後）"},
    {"pProbationHelp", "When tracking comes back after a reset, keep the corrected position this long until it settles. Longer is safer, but slower to come back.",
     "トラッキングがリセットされて戻ってきたとき、位置が落ち着くまで補正を続ける時間です。長いほど飛びにくく、元に戻るのは遅くなります。"},
    {"pShortProbation", "Wait after a short loss", "復帰を待つ時間（短いロスト）"},
    {"pShortProbationHelp", "The same, for a brief loss that ends without a reset.", "リセットを伴わない短いロストから戻ったときに、補正を続ける時間です。"},
    {"pBlend", "Time to return", "元の位置へ戻る時間"},
    {"pBlendHelp", "How long it takes to glide from the corrected position back to the real one. Shorter is quicker, longer is smoother.",
     "補正をやめて、実際の位置へ戻るまでにかける時間です。短いと素早く、長いと滑らかに戻ります。"},
    {"pStepLimit", "Warp distance", "ワープとみなす距離"},
    {"pStepLimitHelp", "A move larger than this in one frame is treated as a warp and joined up smoothly.",
     "1 フレームでこれより大きく動いたら、ワープ（瞬間移動）とみなして滑らかにつなぎます。"},
    {"pStepBlend", "Warp smoothing time", "ワープをならす時間"},
    {"pStepBlendHelp", "How long it takes to glide to the new position after a warp.", "ワープしたとき、新しい位置まで滑らかに移動させる時間です。"},
    {"sec", "{0} s", "{0} 秒"},
    {"resetDefaults", "Reset to defaults", "既定値に戻す"},
    {"recordTitle", "Recording (debug)", "レコーディング（デバッグ用）"},
    {"recordNote", "Saves device positions (raw and corrected), the Lighthouse driver log and correction events to an .otsrec file, for investigating problems.",
     "デバイスの位置（生の値と補正後）、Lighthouse ドライバのログ、補正の動作を .otsrec ファイルに保存します。不具合を調べるときに使います。"},
    {"recordStart", "Start recording", "レコーディング開始"},
    {"recordStop", "Stop recording", "レコーディング停止"},
    {"addMarker", "Add marker", "マーカーを追加"},
    {"language", "Language", "言語"},
    {"theme", "Color theme", "カラーテーマ"},
    {"defaultTag", "Default", "標準"},
    {"display", "Display", "表示"},
    {"desktopWindow", "Also show a window on the desktop", "デスクトップにもウィンドウを表示する"},
    {"savedTo", "Display settings are saved to:", "表示の設定の保存先:"},
    {"about", "About", "このアプリについて"},
    {"madeBy", "Made by", "制作"},
    {"creator", "kuroha", "くろ～は（kuroha）"},
    // toasts and log lines written by the UI
    {"toastFilterOn", "Correction turned on", "補正をオンにしました"},
    {"toastFilterOff", "Correction turned off", "補正をオフにしました"},
    {"toastDeviceOn", "Correcting {0}", "{0} を補正します"},
    {"toastDeviceOff", "Not correcting {0}", "{0} を補正しません"},
    {"toastDefaults", "Settings reset to defaults", "設定を既定値に戻しました"},
    {"toastRecStart", "Recording started", "レコーディングを開始しました"},
    {"toastRecStop", "Recording stopped", "レコーディングを停止しました"},
    {"toastMarker", "Marker added", "マーカーを追加しました"},
    {"toastSaveFailed", "Could not save settings", "設定を保存できませんでした"},
};

using Table = std::unordered_map<std::string, std::string>;

const Table& GetTable(const std::string& lang) {
    static const auto build = [](bool ja) {
        Table t;
        for (const Entry& e : kEntries) t[e.key] = ja ? e.ja : e.en;
        return t;
    };
    static const Table en = build(false), ja = build(true);
    return lang == "ja" ? ja : en;
}

std::string XmlEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        switch (c) {
        case '&': o += "&amp;"; break;
        case '<': o += "&lt;"; break;
        case '>': o += "&gt;"; break;
        case '"': o += "&quot;"; break;
        default: o += c;
        }
    }
    return o;
}

} // namespace

const std::string& Text(const std::string& lang, const std::string& key) {
    const Table& t = GetTable(lang);
    auto it = t.find(key);
    if (it != t.end()) return it->second;
    static thread_local std::string missing;
    missing = key;
    return missing;
}

std::string Fmt(const std::string& lang, const std::string& key, const std::string& a0, const std::string& a1) {
    std::string s = Text(lang, key);
    for (auto [token, value] : {std::pair<const char*, const std::string*>{"{0}", &a0}, {"{1}", &a1}})
        for (size_t p; (p = s.find(token)) != std::string::npos;) s.replace(p, 3, *value);
    return s;
}

std::string TranslateRml(const std::string& rml, const std::string& lang) {
    std::string out;
    out.reserve(rml.size() + rml.size() / 4);
    size_t i = 0;
    while (i < rml.size()) {
        const size_t open = rml.find("{%", i);
        if (open == std::string::npos) break;
        const size_t close = rml.find("%}", open + 2);
        if (close == std::string::npos) break;
        out.append(rml, i, open - i);
        out += XmlEscape(Text(lang, rml.substr(open + 2, close - open - 2)));
        i = close + 2;
    }
    out.append(rml, i, std::string::npos);
    return out;
}

} // namespace ots
