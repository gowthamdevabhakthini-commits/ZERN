// @ts-nocheck
import AsyncStorage from "@react-native-async-storage/async-storage";
import { Audio } from "expo-av";
import * as FileSystem from "expo-file-system";
import * as MediaLibrary from "expo-media-library";
import React, { useEffect, useRef, useState } from "react";
import {
  ActivityIndicator,
  Alert,
  Dimensions,
  FlatList,
  Image,
  Modal,
  Platform,
  ScrollView,
  StatusBar,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";
import { WebView } from "react-native-webview";

const { width: SCREEN_W } = Dimensions.get("window");

// ─── Theme ───────────────────────────────────────────────
const C = {
  bg: "#0a0a0a",
  surface: "#141414",
  surface2: "#1e1e1e",
  border: "#2a2a2a",
  border2: "#333",
  accent: "#e8ff47",
  accentDim: "rgba(232,255,71,0.12)",
  red: "#ff4545",
  green: "#44ff88",
  blue: "#4d9fff",
  text: "#f0f0f0",
  muted: "#666",
  muted2: "#3a3a3a",
  white: "#ffffff",
};

// ─── Helpers ─────────────────────────────────────────────
function useLog() {
  const [logs, setLogs] = useState([{ t: "info", m: "// ready" }]);
  const add = (m, t = "info") => {
    const ts = new Date().toLocaleTimeString("en-US", { hour12: false });
    setLogs((p) => [...p.slice(-40), { t, m: `[${ts}] ${m}` }]);
  };
  return [logs, add];
}

// ─── Components ──────────────────────────────────────────
function Btn({
  label,
  onPress,
  color = C.border,
  textColor = C.muted,
  style,
  disabled,
}) {
  return (
    <TouchableOpacity
      onPress={onPress}
      disabled={disabled}
      style={[
        styles.btn,
        { borderColor: color, opacity: disabled ? 0.3 : 1 },
        style,
      ]}
      activeOpacity={0.7}
    >
      <Text style={[styles.btnText, { color: textColor }]}>{label}</Text>
    </TouchableOpacity>
  );
}

function PrimaryBtn({ label, onPress, disabled, style }) {
  return (
    <TouchableOpacity
      onPress={onPress}
      disabled={disabled}
      style={[styles.primaryBtn, { opacity: disabled ? 0.3 : 1 }, style]}
      activeOpacity={0.8}
    >
      <Text style={styles.primaryBtnText}>{label}</Text>
    </TouchableOpacity>
  );
}

function Toggle({ value, onChange, label }) {
  return (
    <View style={styles.toggleRow}>
      <Text style={styles.toggleLabel}>{label}</Text>
      <TouchableOpacity
        onPress={() => onChange(!value)}
        style={[styles.toggleTrack, value && styles.toggleTrackOn]}
        activeOpacity={0.8}
      >
        <View style={[styles.toggleThumb, value && styles.toggleThumbOn]} />
      </TouchableOpacity>
    </View>
  );
}

function Slider({ label, value, min, max, step = 1, onChange, unit = "" }) {
  const steps = Math.floor((max - min) / step);
  const pct = (value - min) / (max - min);
  return (
    <View style={styles.sliderWrap}>
      <View style={styles.sliderHeader}>
        <Text style={styles.sliderLabel}>{label}</Text>
        <Text style={styles.sliderVal}>
          {value}
          {unit}
        </Text>
      </View>
      <View style={styles.sliderTrack}>
        <View style={[styles.sliderFill, { width: `${pct * 100}%` }]} />
        <View style={styles.sliderSteps}>
          {Array.from({ length: steps + 1 }).map((_, i) => (
            <TouchableOpacity
              key={i}
              style={styles.sliderStep}
              onPress={() => onChange(min + i * step)}
            />
          ))}
        </View>
      </View>
      <View style={styles.sliderMinMax}>
        <Text style={styles.sliderMin}>{min}</Text>
        <Text style={styles.sliderMin}>{max}</Text>
      </View>
    </View>
  );
}

// ─── Main App ─────────────────────────────────────────────
export default function App() {
  const [ip, setIp] = useState("10.205.157.200");
  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);
  const [tab, setTab] = useState("stream"); // stream | gallery | settings | log
  const [photos, setPhotos] = useState([]);
  const [capturing, setCapturing] = useState(false);
  const [recording, setRecording] = useState(false);
  const [audioRec, setAudioRec] = useState(null);
  const [recordingAudio, setRecordingAudio] = useState(false);
  const [videoFrames, setVideoFrames] = useState([]);
  const [videoInterval, setVideoIntervalRef] = useState(null);
  const [selectedPhoto, setSelectedPhoto] = useState(null);
  const [logs, addLog] = useLog();
  const [mediaPermission, setMediaPermission] = useState(false);
  const [audioPermission, setAudioPermission] = useState(false);

  // Camera settings
  const [res, setRes] = useState("5");
  const [quality, setQuality] = useState(10);
  const [brightness, setBrightness] = useState(0);
  const [contrast, setContrast] = useState(0);
  const [hmirror, setHmirror] = useState(false);
  const [vflip, setVflip] = useState(false);
  const [awb, setAwb] = useState(true);
  const [aec, setAec] = useState(true);

  const streamKey = useRef(0);

  const baseUrl = `http://${ip}`;
  const streamUrl = `http://${ip}:81/stream`;

  const RES_OPTIONS = [
    { label: "96×96", value: "0" },
    { label: "160×120", value: "1" },
    { label: "176×144", value: "2" },
    { label: "240×176", value: "3" },
    { label: "320×240 QVGA", value: "4" },
    { label: "640×480 VGA", value: "5" },
    { label: "800×600 SVGA", value: "6" },
    { label: "1280×720 HD", value: "8" },
    { label: "1600×1200 UXGA", value: "10" },
  ];

  useEffect(() => {
    (async () => {
      const { status } = await MediaLibrary.requestPermissionsAsync();
      setMediaPermission(status === "granted");
      const { status: as } = await Audio.requestPermissionsAsync();
      setAudioPermission(as === "granted");
      const saved = await AsyncStorage.getItem("esp32_ip");
      if (saved) setIp(saved);
    })();
  }, []);

  const handleConnect = async () => {
    if (!ip.trim()) {
      addLog("Enter a valid IP address", "err");
      return;
    }
    await AsyncStorage.setItem("esp32_ip", ip.trim());
    setConnecting(true);
    addLog(`Connecting to ${baseUrl}…`);
    try {
      const r = await fetch(`${baseUrl}/status`, {
        signal: AbortSignal.timeout(4000),
      });
      if (r.ok || r.status === 404) {
        setConnected(true);
        setConnecting(false);
        streamKey.current += 1;
        addLog("Connected!", "ok");
      } else {
        throw new Error(`HTTP ${r.status}`);
      }
    } catch (e) {
      // Try stream port as fallback
      try {
        await fetch(`http://${ip}:81`, { signal: AbortSignal.timeout(3000) });
        setConnected(true);
        setConnecting(false);
        streamKey.current += 1;
        addLog("Connected via stream port", "ok");
      } catch {
        setConnecting(false);
        addLog(`Connection failed: ${e.message}`, "err");
        Alert.alert(
          "Connection Failed",
          `Could not reach ${baseUrl}\n\nMake sure:\n• Phone and ESP32 are on same WiFi\n• IP address is correct`,
        );
      }
    }
  };

  const handleDisconnect = () => {
    setConnected(false);
    stopVideoRecording();
    stopAudioRecording();
    addLog("Disconnected");
  };

  const sendCtrl = async (variable, value) => {
    try {
      await fetch(`${baseUrl}/control?var=${variable}&val=${value}`);
      addLog(`${variable} → ${value}`, "ok");
    } catch (e) {
      addLog(`Control failed: ${e.message}`, "err");
    }
  };

  const capturePhoto = async () => {
    if (!connected || capturing) return;
    setCapturing(true);
    addLog("Capturing photo…");
    try {
      const url = `${baseUrl}/capture?_t=${Date.now()}`;
      const path = FileSystem.cacheDirectory + `esp32_${Date.now()}.jpg`;
      const { uri } = await FileSystem.downloadAsync(url, path);
      const ts = new Date().toLocaleTimeString("en-US", { hour12: false });

      if (mediaPermission) {
        const asset = await MediaLibrary.createAssetAsync(uri);
        addLog(`Photo saved to gallery`, "ok");
        setPhotos((p) => [{ uri: asset.uri, ts, id: asset.id }, ...p]);
      } else {
        setPhotos((p) => [{ uri, ts, id: Date.now().toString() }, ...p]);
        addLog(`Photo captured (no gallery permission)`, "ok");
      }
    } catch (e) {
      addLog(`Capture failed: ${e.message}`, "err");
    } finally {
      setCapturing(false);
    }
  };

  const startVideoRecording = () => {
    if (!connected || recording) return;
    setRecording(true);
    setVideoFrames([]);
    addLog("Video recording started…", "ok");
    let frames = [];
    const interval = setInterval(async () => {
      try {
        const url = `${baseUrl}/capture?_t=${Date.now()}`;
        const path = FileSystem.cacheDirectory + `frame_${Date.now()}.jpg`;
        const { uri } = await FileSystem.downloadAsync(url, path);
        frames.push(uri);
        setVideoFrames([...frames]);
        addLog(`Frame ${frames.length} captured`);
      } catch (e) {
        addLog(`Frame error: ${e.message}`, "err");
      }
    }, 1000);
    setVideoIntervalRef(interval);
  };

  const stopVideoRecording = async () => {
    if (!recording) return;
    clearInterval(videoInterval);
    setVideoIntervalRef(null);
    setRecording(false);
    addLog(`Video stopped — ${videoFrames.length} frames captured`, "ok");
    if (videoFrames.length > 0) {
      Alert.alert(
        "Video Recorded",
        `${videoFrames.length} frames captured.\nFrames saved to gallery.`,
        [
          { text: "Save Frames to Gallery", onPress: saveVideoFrames },
          { text: "Discard", style: "destructive" },
        ],
      );
    }
  };

  const saveVideoFrames = async () => {
    if (!mediaPermission) {
      addLog("No gallery permission", "err");
      return;
    }
    let saved = 0;
    for (const uri of videoFrames) {
      try {
        await MediaLibrary.createAssetAsync(uri);
        saved++;
      } catch {}
    }
    addLog(`${saved} frames saved to gallery`, "ok");
  };

  const startAudioRecording = async () => {
    if (!audioPermission) {
      addLog("No microphone permission", "err");
      return;
    }
    if (recordingAudio) return;
    try {
      await Audio.setAudioModeAsync({
        allowsRecordingIOS: true,
        playsInSilentModeIOS: true,
      });
      const { recording } = await Audio.Recording.createAsync(
        Audio.RecordingOptionsPresets.HIGH_QUALITY,
      );
      setAudioRec(recording);
      setRecordingAudio(true);
      addLog("Audio recording started…", "ok");
    } catch (e) {
      addLog(`Audio error: ${e.message}`, "err");
    }
  };

  const stopAudioRecording = async () => {
    if (!recordingAudio || !audioRec) return;
    try {
      await audioRec.stopAndUnloadAsync();
      const uri = audioRec.getURI();
      setAudioRec(null);
      setRecordingAudio(false);
      addLog("Audio saved", "ok");
      if (mediaPermission && uri) {
        await MediaLibrary.createAssetAsync(uri);
        addLog("Audio saved to gallery", "ok");
      }
    } catch (e) {
      addLog(`Audio stop error: ${e.message}`, "err");
    }
  };

  // ── Stream HTML ──
  const streamHtml = `
    <!DOCTYPE html>
    <html>
    <head>
    <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
    <style>
      * { margin:0; padding:0; box-sizing:border-box; }
      body { background:#000; display:flex; align-items:center; justify-content:center; height:100vh; overflow:hidden; }
      img { width:100%; height:100%; object-fit:contain; display:block; }
      .err { color:#666; font-family:monospace; font-size:12px; text-align:center; padding:20px; }
    </style>
    </head>
    <body>
      <img src="${streamUrl}" onerror="this.style.display='none';document.body.innerHTML='<div class=err>NO SIGNAL<br>${streamUrl}</div>'" />
    </body>
    </html>
  `;

  // ── Render Tabs ──
  const renderStream = () => (
    <View style={styles.streamContainer}>
      <View style={styles.streamBox}>
        {connected ? (
          <WebView
            key={streamKey.current}
            source={{ html: streamHtml }}
            style={styles.webview}
            scrollEnabled={false}
            bounces={false}
            allowsInlineMediaPlayback
            mediaPlaybackRequiresUserAction={false}
            mixedContentMode="always"
          />
        ) : (
          <View style={styles.noSignal}>
            <Text style={styles.noSignalIcon}>⊘</Text>
            <Text style={styles.noSignalText}>NO SIGNAL</Text>
            <Text style={styles.noSignalSub}>Connect to your ESP32 above</Text>
          </View>
        )}
        {connected && (
          <View style={styles.streamBadge}>
            <View style={styles.liveDot} />
            <Text style={styles.liveText}>LIVE</Text>
          </View>
        )}
      </View>

      <View style={styles.captureBar}>
        <PrimaryBtn
          label={capturing ? "..." : "⊙ CAPTURE"}
          onPress={capturePhoto}
          disabled={!connected || capturing}
          style={{ flex: 1 }}
        />
        <Btn
          label={recording ? "⏹ STOP REC" : "⏺ VIDEO"}
          onPress={recording ? stopVideoRecording : startVideoRecording}
          disabled={!connected}
          color={recording ? C.red : C.border}
          textColor={recording ? C.red : C.muted}
          style={{ flex: 1 }}
        />
        <Btn
          label={recordingAudio ? "⏹ STOP" : "🎙 AUDIO"}
          onPress={recordingAudio ? stopAudioRecording : startAudioRecording}
          disabled={!connected}
          color={recordingAudio ? C.red : C.border}
          textColor={recordingAudio ? C.red : C.muted}
          style={{ flex: 1 }}
        />
      </View>

      {recording && (
        <View style={styles.recIndicator}>
          <View style={styles.recDot} />
          <Text style={styles.recText}>
            RECORDING — {videoFrames.length} frames
          </Text>
        </View>
      )}
    </View>
  );

  const renderGallery = () => (
    <View style={{ flex: 1 }}>
      {photos.length === 0 ? (
        <View style={styles.emptyGallery}>
          <Text style={styles.emptyIcon}>□</Text>
          <Text style={styles.emptyText}>NO CAPTURES YET</Text>
          <Text style={styles.emptySub}>
            Capture photos from the Stream tab
          </Text>
        </View>
      ) : (
        <FlatList
          data={photos}
          keyExtractor={(i) => i.id}
          numColumns={2}
          contentContainerStyle={styles.galleryGrid}
          renderItem={({ item, index }) => (
            <TouchableOpacity
              style={styles.thumb}
              onPress={() => setSelectedPhoto(item)}
              activeOpacity={0.8}
            >
              <Image source={{ uri: item.uri }} style={styles.thumbImg} />
              <View style={styles.thumbOverlay}>
                <Text style={styles.thumbTs}>{item.ts}</Text>
              </View>
            </TouchableOpacity>
          )}
        />
      )}
    </View>
  );

  const renderSettings = () => (
    <ScrollView
      style={styles.settingsScroll}
      showsVerticalScrollIndicator={false}
    >
      <View style={styles.panel}>
        <Text style={styles.panelTitle}>RESOLUTION</Text>
        <ScrollView
          horizontal
          showsHorizontalScrollIndicator={false}
          style={styles.resScroll}
        >
          {RES_OPTIONS.map((o) => (
            <TouchableOpacity
              key={o.value}
              onPress={() => {
                setRes(o.value);
                sendCtrl("framesize", o.value);
              }}
              style={[styles.resChip, res === o.value && styles.resChipActive]}
            >
              <Text
                style={[
                  styles.resChipText,
                  res === o.value && styles.resChipTextActive,
                ]}
              >
                {o.label}
              </Text>
            </TouchableOpacity>
          ))}
        </ScrollView>
      </View>

      <View style={styles.panel}>
        <Text style={styles.panelTitle}>IMAGE QUALITY</Text>
        <Slider
          label="JPEG Quality"
          value={quality}
          min={4}
          max={63}
          onChange={(v) => {
            setQuality(v);
            sendCtrl("quality", v);
          }}
        />
        <Slider
          label="Brightness"
          value={brightness}
          min={-2}
          max={2}
          onChange={(v) => {
            setBrightness(v);
            sendCtrl("brightness", v);
          }}
        />
        <Slider
          label="Contrast"
          value={contrast}
          min={-2}
          max={2}
          onChange={(v) => {
            setContrast(v);
            sendCtrl("contrast", v);
          }}
        />
      </View>

      <View style={styles.panel}>
        <Text style={styles.panelTitle}>TRANSFORMS</Text>
        <Toggle
          label="H-Mirror"
          value={hmirror}
          onChange={(v) => {
            setHmirror(v);
            sendCtrl("hmirror", v ? 1 : 0);
          }}
        />
        <Toggle
          label="V-Flip"
          value={vflip}
          onChange={(v) => {
            setVflip(v);
            sendCtrl("vflip", v ? 1 : 0);
          }}
        />
        <Toggle
          label="Auto White Balance"
          value={awb}
          onChange={(v) => {
            setAwb(v);
            sendCtrl("awb", v ? 1 : 0);
          }}
        />
        <Toggle
          label="Auto Exposure"
          value={aec}
          onChange={(v) => {
            setAec(v);
            sendCtrl("aec", v ? 1 : 0);
          }}
        />
      </View>
      <View style={{ height: 40 }} />
    </ScrollView>
  );

  const renderLog = () => (
    <ScrollView style={styles.logScroll} showsVerticalScrollIndicator={false}>
      {[...logs].reverse().map((l, i) => (
        <Text
          key={i}
          style={[
            styles.logLine,
            l.t === "ok"
              ? styles.logOk
              : l.t === "err"
                ? styles.logErr
                : styles.logInfo,
          ]}
        >
          {l.m}
        </Text>
      ))}
    </ScrollView>
  );

  return (
    <View style={styles.app}>
      <StatusBar barStyle="light-content" backgroundColor={C.bg} />

      {/* Header */}
      <View style={styles.header}>
        <View style={styles.headerLeft}>
          <Text style={styles.logo}>
            ESP32<Text style={styles.logoDim}>—</Text>CAM
          </Text>
          <View
            style={[
              styles.statusPill,
              connected
                ? styles.statusLive
                : connecting
                  ? styles.statusConnecting
                  : {},
            ]}
          >
            <View
              style={[
                styles.statusDot,
                {
                  backgroundColor: connected
                    ? C.green
                    : connecting
                      ? C.accent
                      : C.muted,
                },
              ]}
            />
            <Text
              style={[
                styles.statusText,
                {
                  color: connected ? C.green : connecting ? C.accent : C.muted,
                },
              ]}
            >
              {connected ? "LIVE" : connecting ? "CONNECTING" : "OFFLINE"}
            </Text>
          </View>
        </View>
      </View>

      {/* IP Bar */}
      <View style={styles.connectBar}>
        <TextInput
          style={styles.ipInput}
          value={ip}
          onChangeText={setIp}
          placeholder="192.168.x.x"
          placeholderTextColor={C.muted}
          keyboardType="numeric"
          autoCapitalize="none"
          onSubmitEditing={handleConnect}
        />
        {!connected ? (
          <TouchableOpacity
            onPress={handleConnect}
            disabled={connecting}
            style={[styles.connectBtn, connecting && { opacity: 0.5 }]}
            activeOpacity={0.8}
          >
            {connecting ? (
              <ActivityIndicator size="small" color="#000" />
            ) : (
              <Text style={styles.connectBtnText}>CONNECT</Text>
            )}
          </TouchableOpacity>
        ) : (
          <TouchableOpacity
            onPress={handleDisconnect}
            style={styles.stopBtn}
            activeOpacity={0.8}
          >
            <Text style={styles.stopBtnText}>STOP</Text>
          </TouchableOpacity>
        )}
      </View>

      {/* Tab Bar */}
      <View style={styles.tabBar}>
        {[
          { id: "stream", label: "▶ STREAM" },
          {
            id: "gallery",
            label: `◫ GALLERY${photos.length > 0 ? ` (${photos.length})` : ""}`,
          },
          { id: "settings", label: "⚙ SETTINGS" },
          { id: "log", label: "» LOG" },
        ].map((t) => (
          <TouchableOpacity
            key={t.id}
            onPress={() => setTab(t.id)}
            style={[styles.tabItem, tab === t.id && styles.tabItemActive]}
            activeOpacity={0.7}
          >
            <Text
              style={[styles.tabText, tab === t.id && styles.tabTextActive]}
            >
              {t.label}
            </Text>
          </TouchableOpacity>
        ))}
      </View>

      {/* Content */}
      <View style={styles.content}>
        {tab === "stream" && renderStream()}
        {tab === "gallery" && renderGallery()}
        {tab === "settings" && renderSettings()}
        {tab === "log" && renderLog()}
      </View>

      {/* Photo Preview Modal */}
      <Modal visible={!!selectedPhoto} transparent animationType="fade">
        <View style={styles.modal}>
          <TouchableOpacity
            style={styles.modalClose}
            onPress={() => setSelectedPhoto(null)}
          >
            <Text style={styles.modalCloseText}>✕ CLOSE</Text>
          </TouchableOpacity>
          {selectedPhoto && (
            <Image
              source={{ uri: selectedPhoto.uri }}
              style={styles.modalImage}
              resizeMode="contain"
            />
          )}
          <Text style={styles.modalTs}>{selectedPhoto?.ts}</Text>
        </View>
      </Modal>
    </View>
  );
}

// ─── Styles ──────────────────────────────────────────────
const styles = StyleSheet.create({
  app: { flex: 1, backgroundColor: C.bg },
  header: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    paddingHorizontal: 16,
    paddingTop: 50,
    paddingBottom: 10,
    borderBottomWidth: 0.5,
    borderBottomColor: C.border,
  },
  headerLeft: { flexDirection: "row", alignItems: "center", gap: 10 },
  logo: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 16,
    fontWeight: "700",
    color: C.accent,
  },
  logoDim: { color: C.muted },
  statusPill: {
    flexDirection: "row",
    alignItems: "center",
    gap: 5,
    paddingHorizontal: 8,
    paddingVertical: 3,
    borderRadius: 2,
    borderWidth: 0.5,
    borderColor: C.border,
  },
  statusLive: { borderColor: C.green },
  statusConnecting: { borderColor: C.accent },
  statusDot: { width: 5, height: 5, borderRadius: 3 },
  statusText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 9,
    letterSpacing: 1,
  },

  connectBar: {
    flexDirection: "row",
    gap: 8,
    padding: 12,
    borderBottomWidth: 0.5,
    borderBottomColor: C.border,
  },
  ipInput: {
    flex: 1,
    backgroundColor: C.surface,
    borderWidth: 0.5,
    borderColor: C.border,
    borderRadius: 2,
    paddingHorizontal: 12,
    paddingVertical: 8,
    color: C.text,
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 13,
  },
  connectBtn: {
    backgroundColor: C.accent,
    borderRadius: 2,
    paddingHorizontal: 16,
    paddingVertical: 8,
    justifyContent: "center",
    alignItems: "center",
    minWidth: 90,
  },
  connectBtnText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 11,
    fontWeight: "700",
    color: "#000",
    letterSpacing: 1,
  },
  stopBtn: {
    borderWidth: 0.5,
    borderColor: C.red,
    borderRadius: 2,
    paddingHorizontal: 16,
    paddingVertical: 8,
    justifyContent: "center",
    alignItems: "center",
    minWidth: 90,
  },
  stopBtnText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 11,
    color: C.red,
    letterSpacing: 1,
  },

  tabBar: {
    flexDirection: "row",
    borderBottomWidth: 0.5,
    borderBottomColor: C.border,
    backgroundColor: C.surface,
  },
  tabItem: { flex: 1, paddingVertical: 10, alignItems: "center" },
  tabItemActive: { borderBottomWidth: 1.5, borderBottomColor: C.accent },
  tabText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 9,
    color: C.muted,
    letterSpacing: 0.5,
  },
  tabTextActive: { color: C.accent },

  content: { flex: 1 },

  // Stream
  streamContainer: { flex: 1, padding: 12, gap: 10 },
  streamBox: {
    flex: 1,
    backgroundColor: "#000",
    borderRadius: 4,
    borderWidth: 0.5,
    borderColor: C.border,
    overflow: "hidden",
    position: "relative",
    minHeight: 240,
  },
  webview: { flex: 1, backgroundColor: "#000" },
  noSignal: { flex: 1, alignItems: "center", justifyContent: "center", gap: 8 },
  noSignalIcon: { fontSize: 36, color: C.muted2 },
  noSignalText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 13,
    color: C.muted,
    letterSpacing: 2,
  },
  noSignalSub: { fontSize: 11, color: C.muted2 },
  streamBadge: {
    position: "absolute",
    top: 8,
    left: 8,
    flexDirection: "row",
    alignItems: "center",
    gap: 5,
    backgroundColor: "rgba(0,0,0,0.6)",
    paddingHorizontal: 8,
    paddingVertical: 3,
    borderRadius: 2,
  },
  liveDot: { width: 6, height: 6, borderRadius: 3, backgroundColor: C.red },
  liveText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 9,
    color: "#fff",
    letterSpacing: 1,
  },

  captureBar: { flexDirection: "row", gap: 8 },
  btn: {
    borderWidth: 0.5,
    borderRadius: 2,
    paddingVertical: 10,
    alignItems: "center",
    justifyContent: "center",
  },
  btnText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 10,
    letterSpacing: 0.5,
  },
  primaryBtn: {
    backgroundColor: C.accent,
    borderRadius: 2,
    paddingVertical: 10,
    alignItems: "center",
    justifyContent: "center",
  },
  primaryBtnText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 11,
    fontWeight: "700",
    color: "#000",
    letterSpacing: 1,
  },

  recIndicator: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
    backgroundColor: "rgba(255,69,69,0.1)",
    borderWidth: 0.5,
    borderColor: C.red,
    borderRadius: 2,
    padding: 8,
  },
  recDot: { width: 8, height: 8, borderRadius: 4, backgroundColor: C.red },
  recText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 10,
    color: C.red,
    letterSpacing: 1,
  },

  // Gallery
  galleryGrid: { padding: 10, gap: 6 },
  thumb: {
    flex: 1,
    margin: 3,
    aspectRatio: 4 / 3,
    borderRadius: 3,
    overflow: "hidden",
    borderWidth: 0.5,
    borderColor: C.border,
  },
  thumbImg: { width: "100%", height: "100%" },
  thumbOverlay: {
    position: "absolute",
    bottom: 0,
    left: 0,
    right: 0,
    backgroundColor: "rgba(0,0,0,0.6)",
    padding: 4,
  },
  thumbTs: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 8,
    color: C.muted,
  },
  emptyGallery: {
    flex: 1,
    alignItems: "center",
    justifyContent: "center",
    gap: 8,
  },
  emptyIcon: { fontSize: 40, color: C.muted2 },
  emptyText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 12,
    color: C.muted,
    letterSpacing: 2,
  },
  emptySub: { fontSize: 12, color: C.muted2 },

  // Settings
  settingsScroll: { flex: 1, padding: 12 },
  panel: {
    backgroundColor: C.surface,
    borderWidth: 0.5,
    borderColor: C.border,
    borderRadius: 4,
    padding: 14,
    marginBottom: 12,
  },
  panelTitle: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 9,
    color: C.muted,
    letterSpacing: 2,
    marginBottom: 12,
  },
  resScroll: { marginHorizontal: -4 },
  resChip: {
    paddingHorizontal: 10,
    paddingVertical: 6,
    borderRadius: 2,
    borderWidth: 0.5,
    borderColor: C.border,
    marginHorizontal: 3,
  },
  resChipActive: { backgroundColor: C.accentDim, borderColor: C.accent },
  resChipText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 10,
    color: C.muted,
  },
  resChipTextActive: { color: C.accent },

  sliderWrap: { marginBottom: 16 },
  sliderHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    marginBottom: 8,
  },
  sliderLabel: { fontSize: 11, color: C.muted },
  sliderVal: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 11,
    color: C.accent,
  },
  sliderTrack: {
    height: 2,
    backgroundColor: C.border,
    borderRadius: 1,
    position: "relative",
  },
  sliderFill: {
    position: "absolute",
    top: 0,
    left: 0,
    height: 2,
    backgroundColor: C.accent,
    borderRadius: 1,
  },
  sliderSteps: {
    position: "absolute",
    top: -10,
    left: 0,
    right: 0,
    bottom: -10,
    flexDirection: "row",
  },
  sliderStep: { flex: 1, height: "100%" },
  sliderMinMax: {
    flexDirection: "row",
    justifyContent: "space-between",
    marginTop: 4,
  },
  sliderMin: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 9,
    color: C.muted2,
  },

  toggleRow: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    paddingVertical: 10,
    borderBottomWidth: 0.5,
    borderBottomColor: C.border,
  },
  toggleLabel: { fontSize: 13, color: C.muted },
  toggleTrack: {
    width: 38,
    height: 22,
    borderRadius: 11,
    backgroundColor: C.surface2,
    borderWidth: 0.5,
    borderColor: C.border,
    justifyContent: "center",
    padding: 3,
  },
  toggleTrackOn: { backgroundColor: C.accentDim, borderColor: C.accent },
  toggleThumb: {
    width: 14,
    height: 14,
    borderRadius: 7,
    backgroundColor: C.muted,
  },
  toggleThumbOn: { backgroundColor: C.accent, alignSelf: "flex-end" },

  // Log
  logScroll: { flex: 1, padding: 12 },
  logLine: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 10,
    letterSpacing: 0.3,
    paddingVertical: 2,
  },
  logOk: { color: C.green },
  logErr: { color: C.red },
  logInfo: { color: C.muted },

  // Modal
  modal: {
    flex: 1,
    backgroundColor: "rgba(0,0,0,0.95)",
    alignItems: "center",
    justifyContent: "center",
    padding: 20,
  },
  modalClose: { position: "absolute", top: 50, right: 20, padding: 10 },
  modalCloseText: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 11,
    color: C.muted,
    letterSpacing: 1,
  },
  modalImage: { width: SCREEN_W - 40, height: SCREEN_W - 40, borderRadius: 4 },
  modalTs: {
    fontFamily: Platform.OS === "ios" ? "Courier New" : "monospace",
    fontSize: 10,
    color: C.muted,
    marginTop: 12,
    letterSpacing: 1,
  },
});
