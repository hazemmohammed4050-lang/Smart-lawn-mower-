// =============================================================================
//  ESP32-WROOM-32 Rover Controller — Access Point Mode
//  Motor Driver : L298N
//  Connection   : ESP32 acts as WiFi Access Point
//  Web UI       : Served at http://192.168.4.1
//  Ultrasonic   : HC-SR04 via NewPing
// =============================================================================
//
//  HOW TO USE:
//    1. Upload this sketch to your ESP32
//    2. On your phone/PC, connect to WiFi:
//         Network : "ESP32_Rover"
//         Password: "rover1234"
//    3. Open browser and go to: http://192.168.4.1
//    4. Use the on-screen joystick to control the rover!
//
//  REQUIRED LIBRARIES:
//    • NewPing by Tim Eckel  (Arduino Library Manager)
//    • WiFi, WebServer       (included in ESP32 core)
//
//  L298N WIRING:
//    ENA  → GPIO 14  |  IN1 → GPIO 26  |  IN2 → GPIO 27  (Right Motor)
//    ENB  → GPIO 25  |  IN3 → GPIO 32  |  IN4 → GPIO 33  (Left Motor)
//    Extra Motor ENA → GPIO 13 | IN1 → GPIO 18 | IN2 → GPIO 19
//    HC-SR04 TRIG → GPIO 22  |  ECHO → GPIO 23
// =============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <NewPing.h>

// ---------------------------------------------------------------------------
//  ACCESS POINT CREDENTIALS
// ---------------------------------------------------------------------------
const char* AP_SSID     = "ESP32_Rover";
const char* AP_PASSWORD = "rover1234";      // min 8 chars, or "" for open

// ---------------------------------------------------------------------------
//  PIN DEFINITIONS
// ---------------------------------------------------------------------------
#define MOTOR_R_EN   14
#define MOTOR_R_IN1  26
#define MOTOR_R_IN2  27

#define MOTOR_L_EN   25
#define MOTOR_L_IN1  32
#define MOTOR_L_IN2  33

#define MOTOR_X_EN   13
#define MOTOR_X_IN1  18
#define MOTOR_X_IN2  19

// Third Motor Y  2190 change pins to match your wiring
#define MOTOR_Y_EN    2
#define MOTOR_Y_IN1   4
#define MOTOR_Y_IN2   5

#define TRIG_PIN     22
#define ECHO_PIN     23

// ---------------------------------------------------------------------------
//  PARAMETERS
// ---------------------------------------------------------------------------
#define MOTOR_SPEED       200
#define TURN_SPEED        150
#define STRAIGHT_SPEED    180
#define OBSTACLE_DIST_CM   20
#define PING_INTERVAL_MS   60
#define MAX_DIST_CM       400
#define TURN_DURATION_MS  750
#define LANE_WIDTH_CM      30
#define ROVER_SPEED_CMS    20

// ---------------------------------------------------------------------------
//  OBJECTS
// ---------------------------------------------------------------------------
WebServer server(80);
NewPing   sonar(TRIG_PIN, ECHO_PIN, MAX_DIST_CM);

// ---------------------------------------------------------------------------
//  STATE
// ---------------------------------------------------------------------------
String        lastCmd          = "";
bool          obstacleDetected = false;
unsigned long lastPingTime     = 0;

bool areaModeActive = false;
int  areaLength_cm  = 0;
int  areaWidth_cm   = 0;
bool lengthSet      = false;
bool widthSet       = false;

enum SweepState { SWEEP_IDLE, SWEEP_FORWARD, SWEEP_TURN1, SWEEP_SIDEWAYS, SWEEP_TURN2, SWEEP_DONE };
SweepState    sweepState     = SWEEP_IDLE;
unsigned long sweepTimer     = 0;
bool          sweepTurnDir   = true;
int           sweepLanesLeft = 0;
int           sweepLanesDone = 0;

// =============================================================================
//  HTML PAGE  (stored in flash)
// =============================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Smart Mow</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Share+Tech+Mono&display=swap');

  :root {
    --bg:       #0a0c10;
    --panel:    #0f1318;
    --border:   #1e2a38;
    --accent:   #00d4ff;
    --accent2:  #ff6b00;
    --danger:   #ff2244;
    --safe:     #00ff88;
    --text:     #c8d8e8;
    --dim:      #4a6070;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Share Tech Mono', monospace;
    min-height: 100dvh;
    display: flex;
    flex-direction: column;
    align-items: center;
    overflow-x: hidden;
    padding-bottom: 24px;
  }

  /* scanline overlay */
  body::before {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(0deg, transparent, transparent 2px, rgba(0,212,255,.018) 2px, rgba(0,212,255,.018) 4px);
    pointer-events: none; z-index: 999;
  }

  /* ── HEADER ── */
  header {
    width: 100%; max-width: 480px;
    display: flex; align-items: center; justify-content: space-between;
    padding: 14px 20px 10px;
    border-bottom: 1px solid var(--border);
  }
  .logo {
    font-family: 'Orbitron', sans-serif;
    font-size: 1.15rem; font-weight: 900;
    color: var(--accent);
    letter-spacing: .12em;
    text-shadow: 0 0 18px rgba(0,212,255,.5);
  }
  .logo span { color: var(--accent2); }

  /* status pill */
  #status-pill {
    display: flex; align-items: center; gap: 7px;
    font-size: .7rem; letter-spacing: .08em;
    color: var(--dim);
  }
  #status-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--dim);
    transition: background .3s, box-shadow .3s;
  }
  #status-dot.ok   { background: var(--safe);   box-shadow: 0 0 8px var(--safe); }
  #status-dot.warn { background: var(--danger);  box-shadow: 0 0 8px var(--danger); }

  /* ── TELEMETRY BAR ── */
  .telem {
    width: 100%; max-width: 480px;
    display: flex; gap: 10px;
    padding: 10px 20px;
  }
  .telem-box {
    flex: 1;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 8px 10px;
    text-align: center;
  }
  .telem-label { font-size: .6rem; color: var(--dim); letter-spacing: .1em; }
  .telem-value { font-family: 'Orbitron', sans-serif; font-size: .95rem; color: var(--accent); margin-top: 2px; }
  #obs-val { color: var(--safe); }
  #obs-val.danger { color: var(--danger); }

  /* ── MAIN CARD ── */
  .card {
    width: calc(100% - 32px); max-width: 440px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 22px 18px;
    margin: 0 16px 14px;
    position: relative;
    overflow: hidden;
  }
  .card::before {
    content: '';
    position: absolute; top: 0; left: 0; right: 0; height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
  }
  .card-title {
    font-family: 'Orbitron', sans-serif;
    font-size: .65rem; letter-spacing: .18em;
    color: var(--dim); margin-bottom: 18px;
    text-align: center;
  }

  /* ── D-PAD ── */
  .dpad {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr 1fr;
    gap: 10px;
    width: 220px; margin: 0 auto;
  }

  .btn {
    background: #141a22;
    border: 1px solid var(--border);
    border-radius: 10px;
    color: var(--accent);
    font-family: 'Orbitron', sans-serif;
    font-size: 1.4rem;
    height: 64px;
    cursor: pointer;
    display: flex; align-items: center; justify-content: center;
    transition: background .12s, border-color .12s, box-shadow .12s, transform .1s;
    user-select: none;
    -webkit-user-select: none;
  }
  .btn:active, .btn.pressed {
    background: rgba(0,212,255,.12);
    border-color: var(--accent);
    box-shadow: 0 0 16px rgba(0,212,255,.3), inset 0 0 8px rgba(0,212,255,.1);
    transform: scale(.93);
  }
  .btn-stop {
    color: var(--danger);
    border-color: #2a1520;
    font-size: 1rem; letter-spacing: .05em;
  }
  .btn-stop:active, .btn-stop.pressed {
    background: rgba(255,34,68,.12);
    border-color: var(--danger);
    box-shadow: 0 0 16px rgba(255,34,68,.3);
  }
  .btn-extra {
    color: var(--accent2);
    border-color: #2a1a10;
    font-size: .75rem; letter-spacing: .06em; font-weight: 700;
  }
  .btn-extra:active, .btn-extra.pressed {
    background: rgba(255,107,0,.12);
    border-color: var(--accent2);
    box-shadow: 0 0 16px rgba(255,107,0,.3);
  }

  /* grid placement */
  #btn-F  { grid-column: 2; grid-row: 1; }
  #btn-L  { grid-column: 1; grid-row: 2; }
  #btn-X  { grid-column: 2; grid-row: 2; }
  #btn-R  { grid-column: 3; grid-row: 2; }
  #btn-B  { grid-column: 2; grid-row: 3; }

  /* ── EXTRA MOTOR ROW ── */
  .extra-row {
    display: flex; gap: 10px; margin-top: 16px;
  }
  .extra-row .btn { flex: 1; height: 52px; font-size: .8rem; }

  /* ── AREA MODE ── */
  .area-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px; margin-bottom: 14px;
  }
  .input-group label {
    display: block; font-size: .6rem; letter-spacing: .1em;
    color: var(--dim); margin-bottom: 5px;
  }
  .input-group input {
    width: 100%;
    background: #141a22;
    border: 1px solid var(--border);
    border-radius: 7px;
    color: var(--accent);
    font-family: 'Share Tech Mono', monospace;
    font-size: .95rem;
    padding: 9px 12px;
    outline: none;
    transition: border-color .2s;
  }
  .input-group input:focus { border-color: var(--accent); }

  .btn-area {
    width: 100%; height: 48px;
    background: linear-gradient(135deg, rgba(0,212,255,.1), rgba(0,212,255,.05));
    border: 1px solid var(--accent);
    border-radius: 8px;
    color: var(--accent);
    font-family: 'Orbitron', sans-serif;
    font-size: .7rem; letter-spacing: .1em;
    cursor: pointer;
    transition: background .15s, box-shadow .15s;
  }
  .btn-area:active {
    background: rgba(0,212,255,.2);
    box-shadow: 0 0 20px rgba(0,212,255,.3);
  }

  /* ── LOG ── */
  #log {
    font-size: .68rem; color: var(--dim);
    line-height: 1.7;
    max-height: 90px; overflow-y: auto;
    padding: 10px;
    background: #090c0f;
    border-radius: 6px;
    border: 1px solid var(--border);
  }
  #log .ok   { color: var(--safe); }
  #log .err  { color: var(--danger); }
  #log .info { color: var(--accent); }

  /* scrollbar */
  #log::-webkit-scrollbar { width: 4px; }
  #log::-webkit-scrollbar-thumb { background: var(--border); border-radius: 2px; }
</style>
</head>
<body>

<!-- HEADER -->
<header>
  <div><div class="logo">SMART <span>MOW</span></div><div style="font-size:.5rem;letter-spacing:.12em;color:var(--dim);margin-top:2px;text-align:center;">POWERED BY D3POLES</div></div>
  <div id="status-pill">
    <div id="status-dot"></div>
    <span id="status-text">CONNECTING</span>
  </div>
</header>

<!-- TELEMETRY -->
<div class="telem">
  <div class="telem-box">
    <div class="telem-label">LAST CMD</div>
    <div class="telem-value" id="last-cmd-val">--</div>
  </div>
  <div class="telem-box">
    <div class="telem-label">OBSTACLE</div>
    <div class="telem-value" id="obs-val">CLEAR</div>
  </div>
  <div class="telem-box">
    <div class="telem-label">MODE</div>
    <div class="telem-value" id="mode-val">MANUAL</div>
  </div>
</div>

<!-- CONTROLS -->
<div class="card">
  <div class="card-title">DIRECTIONAL CONTROL</div>
  <div class="dpad">
    <button class="btn" id="btn-F" data-cmd="F">▲</button>
    <button class="btn" id="btn-L" data-cmd="L">◀</button>
    <button class="btn btn-stop" id="btn-X" data-cmd="X">STOP</button>
    <button class="btn" id="btn-R" data-cmd="R">▶</button>
    <button class="btn" id="btn-B" data-cmd="B">▼</button>
  </div>

  <div class="extra-row">
    <button class="btn btn-extra" id="btn-V" data-cmd="V">⚙ MOTOR ON</button>
    <button class="btn btn-extra" id="btn-P" data-cmd="P">⚙ MOTOR OFF</button>
  </div>
  <div style="margin-top:16px;">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;">
      <span style="font-size:.6rem;letter-spacing:.12em;color:var(--dim);">MOTOR SPEED</span>
      <span style="font-family:'Orbitron',sans-serif;font-size:.85rem;color:var(--accent2);" id="spd-label">100%</span>
    </div>
    <input type="range" id="spd-slider" min="0" max="255" value="255"
      style="width:100%;accent-color:var(--accent2);height:6px;cursor:pointer;">
    <div style="display:flex;justify-content:space-between;font-size:.55rem;color:var(--dim);margin-top:4px;">
      <span>0%</span><span>50%</span><span>100%</span>
    </div>
  </div>
</div>

<!-- MOTOR 3 CARD -->
<div class="card">
  <div class="card-title">MOTOR 3 CONTROL</div>
  <div class="extra-row">
    <button class="btn btn-extra" id="btn-V2" data-cmd="V2">&#9881; MOTOR3 ON</button>
    <button class="btn btn-extra" id="btn-P2" data-cmd="P2">&#9881; MOTOR3 OFF</button>
  </div>
  <div style="margin-top:16px;">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;">
      <span style="font-size:.6rem;letter-spacing:.12em;color:var(--dim);">MOTOR 3 SPEED</span>
      <span style="font-family:'Orbitron',sans-serif;font-size:.85rem;color:var(--accent2);" id="spd2-label">100%</span>
    </div>
    <input type="range" id="spd2-slider" min="0" max="255" value="255" style="width:100%;accent-color:var(--accent2);height:6px;cursor:pointer;">
    <div style="display:flex;justify-content:space-between;font-size:.55rem;color:var(--dim);margin-top:4px;">
      <span>0%</span><span>50%</span><span>100%</span>
    </div>
  </div>
</div>

<!-- AREA MODE -->
<div class="card">
  <div class="card-title">AREA SWEEP MODE</div>
  <div class="area-grid">
    <div class="input-group">
      <label>LENGTH (cm)</label>
      <input type="number" id="inp-length" placeholder="e.g. 200" min="1">
    </div>
    <div class="input-group">
      <label>WIDTH (cm)</label>
      <input type="number" id="inp-width" placeholder="e.g. 150" min="1">
    </div>
  </div>
  <button class="btn-area" id="btn-sweep">▶ START SWEEP</button>
</div>

<!-- LOG -->
<div class="card">
  <div class="card-title">SYSTEM LOG</div>
  <div id="log"><span class="info">// waiting for connection...</span></div>
</div>

<script>
  const BASE = '';   // same host — ESP32 serves this page

  // ── send command ──
  async function send(cmd) {
    try {
      const r = await fetch(`${BASE}/cmd?v=${cmd}`);
      const t = await r.text();
      log(t.startsWith('OK') ? `<span class="ok">${t}</span>` : `<span class="err">${t}</span>`);
      return t;
    } catch(e) {
      log(`<span class="err">ERR: ${e.message}</span>`);
    }
  }

  // ── poll status every 800ms ──
  async function pollStatus() {
    try {
      const r = await fetch(`${BASE}/status`);
      const d = await r.json();

      const dot  = document.getElementById('status-dot');
      const stxt = document.getElementById('status-text');
      dot.className  = d.obstacle ? 'warn' : 'ok';
      stxt.textContent = d.obstacle ? 'OBSTACLE' : 'ONLINE';

      document.getElementById('last-cmd-val').textContent = d.last_cmd || '--';
      const obsEl = document.getElementById('obs-val');
      obsEl.textContent = d.obstacle ? 'BLOCKED' : 'CLEAR';
      obsEl.className = d.obstacle ? 'telem-value danger' : 'telem-value';
      document.getElementById('mode-val').textContent = d.area_mode ? 'SWEEP' : 'MANUAL';
    } catch(e) {
      document.getElementById('status-dot').className = '';
      document.getElementById('status-text').textContent = 'OFFLINE';
    }
  }
  setInterval(pollStatus, 800);
  pollStatus();

  // ── log helper ──
  function log(msg) {
    const el = document.getElementById('log');
    const ts = new Date().toLocaleTimeString('en-GB',{hour12:false});
    el.innerHTML += `<br>[${ts}] ${msg}`;
    el.scrollTop = el.scrollHeight;
  }

  // ── D-pad buttons — press & hold ──
  let holdTimer = null;

  function startCmd(cmd, btnEl) {
    btnEl.classList.add('pressed');
    send(cmd);
    // keep sending every 300ms while held
    holdTimer = setInterval(() => send(cmd), 300);
  }

  function stopCmd(btnEl) {
    btnEl.classList.remove('pressed');
    clearInterval(holdTimer);
    holdTimer = null;
    // only auto-stop for movement commands
    const cmd = btnEl.dataset.cmd;
    if (['F','B','L','R'].includes(cmd)) send('X');
  }

  document.querySelectorAll('.btn[data-cmd]').forEach(btn => {
    const cmd = btn.dataset.cmd;

    // touch
    btn.addEventListener('touchstart', e => { e.preventDefault(); startCmd(cmd, btn); }, {passive:false});
    btn.addEventListener('touchend',   e => { e.preventDefault(); stopCmd(btn); });
    btn.addEventListener('touchcancel',e => { e.preventDefault(); stopCmd(btn); });

    // mouse (desktop)
    btn.addEventListener('mousedown',  () => startCmd(cmd, btn));
    btn.addEventListener('mouseup',    () => stopCmd(btn));
    btn.addEventListener('mouseleave', () => { if(holdTimer) stopCmd(btn); });
  });

  // extra motor & stop don't auto-send X on release
  ['btn-V','btn-P','btn-X'].forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    ['mouseup','touchend'].forEach(ev => {
      el.addEventListener(ev, e => {
        clearInterval(holdTimer); holdTimer = null;
        el.classList.remove('pressed');
      });
    });
  });

  // ── keyboard support ──
  const keyMap = { ArrowUp:'F', ArrowDown:'B', ArrowLeft:'L', ArrowRight:'R', ' ':'X' };
  const keyHeld = {};
  document.addEventListener('keydown', e => {
    const cmd = keyMap[e.key];
    if (!cmd || keyHeld[e.key]) return;
    keyHeld[e.key] = true;
    const btn = document.querySelector(`[data-cmd="${cmd}"]`);
    startCmd(cmd, btn);
  });
  document.addEventListener('keyup', e => {
    const cmd = keyMap[e.key];
    if (!cmd) return;
    keyHeld[e.key] = false;
    const btn = document.querySelector(`[data-cmd="${cmd}"]`);
    stopCmd(btn);
  });

  // ── speed slider ──
  const slider = document.getElementById('spd-slider');
  const spdLabel = document.getElementById('spd-label');
  let sliderTimer = null;
  slider.addEventListener('input', () => {
    const pct = Math.round(slider.value / 255 * 100);
    spdLabel.textContent = pct + '%';
    clearTimeout(sliderTimer);
    sliderTimer = setTimeout(async () => {
      try {
        const r = await fetch(`${BASE}/speed?v=${slider.value}`);
        const t = await r.text();
        log(`<span class="ok">${t}</span>`);
      } catch(e) { log(`<span class="err">ERR: ${e.message}</span>`); }
    }, 80);
  });
  // ── speed slider motor 3 ──
  const slider2 = document.getElementById('spd2-slider');
  const spdLabel2 = document.getElementById('spd2-label');
  let sliderTimer2 = null;
  slider2.addEventListener('input', () => {
    const pct2 = Math.round(slider2.value / 255 * 100);
    spdLabel2.textContent = pct2 + '%';
    clearTimeout(sliderTimer2);
    sliderTimer2 = setTimeout(async () => {
      try {
        const r = await fetch(`${BASE}/speed2?v=${slider2.value}`);
        const t = await r.text();
        log(`<span class="ok">${t}</span>`);
      } catch(e) { log(`<span class="err">ERR: ${e.message}</span>`); }
    }, 80);
  });

  document.getElementById('btn-sweep').addEventListener('click', async () => {
    const l = parseInt(document.getElementById('inp-length').value);
    const w = parseInt(document.getElementById('inp-width').value);
    if (!l || !w || l < 1 || w < 1) {
      log('<span class="err">ERR: enter valid length and width</span>');
      return;
    }
    await send(`Kl${l}`);
    await send(`Kw${w}`);
    log(`<span class="info">Sweep started — ${l}cm × ${w}cm</span>`);
  });
</script>
</body>
</html>
)rawliteral";

// =============================================================================
//  MOTOR CONTROL
// =============================================================================
void rightMotor(bool fwd, uint8_t spd) {
  digitalWrite(MOTOR_R_IN1, fwd ? HIGH : LOW);
  digitalWrite(MOTOR_R_IN2, fwd ? LOW  : HIGH);
  analogWrite(MOTOR_R_EN, spd);
}
void leftMotor(bool fwd, uint8_t spd) {
  digitalWrite(MOTOR_L_IN1, fwd ? HIGH : LOW);
  digitalWrite(MOTOR_L_IN2, fwd ? LOW  : HIGH);
  analogWrite(MOTOR_L_EN, spd);
}
void stopMotors() {
  analogWrite(MOTOR_R_EN, 0); analogWrite(MOTOR_L_EN, 0);
  digitalWrite(MOTOR_R_IN1,LOW); digitalWrite(MOTOR_R_IN2,LOW);
  digitalWrite(MOTOR_L_IN1,LOW); digitalWrite(MOTOR_L_IN2,LOW);
}
uint8_t extraMotorSpeed  = 255;  // Motor X (extra) speed
uint8_t extraMotor2Speed = 255;  // Motor Y (third) speed

void extraMotorOn()   { analogWrite(MOTOR_X_EN, extraMotorSpeed);  digitalWrite(MOTOR_X_IN1,HIGH); digitalWrite(MOTOR_X_IN2,LOW); }
void extraMotorOff()  { analogWrite(MOTOR_X_EN,0); digitalWrite(MOTOR_X_IN1,LOW); digitalWrite(MOTOR_X_IN2,LOW); }
void extraMotor2On()  { analogWrite(MOTOR_Y_EN, extraMotor2Speed); digitalWrite(MOTOR_Y_IN1,HIGH); digitalWrite(MOTOR_Y_IN2,LOW); }
void extraMotor2Off() { analogWrite(MOTOR_Y_EN,0); digitalWrite(MOTOR_Y_IN1,LOW); digitalWrite(MOTOR_Y_IN2,LOW); }

// =============================================================================
//  MOVEMENT
// =============================================================================
void moveForward()  { rightMotor(true,  MOTOR_SPEED); leftMotor(true,  MOTOR_SPEED); }
void moveBackward() { rightMotor(false, MOTOR_SPEED); leftMotor(false, MOTOR_SPEED); }
void turnRight()    { rightMotor(false, TURN_SPEED);  leftMotor(true,  MOTOR_SPEED); }
void turnLeft()     { rightMotor(true,  MOTOR_SPEED); leftMotor(false, TURN_SPEED);  }

bool executeCommand(const String& cmd) {
  if      (cmd == "F") { moveForward();  return true; }
  else if (cmd == "B") { moveBackward(); return true; }
  else if (cmd == "R") { turnRight();    return true; }
  else if (cmd == "L") { turnLeft();     return true; }
  else if (cmd == "X") { stopMotors();   return true; }
  return false;
}

// =============================================================================
//  COMMAND PROCESSOR
// =============================================================================
String processCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return "ERR:empty";

  if (cmd.startsWith("Kl") || cmd.startsWith("kl")) {
    int val = cmd.substring(2).toInt();
    if (val > 0) { areaLength_cm = val; lengthSet = true; tryStartAreaMode(); }
    return "OK:length=" + String(areaLength_cm);
  }
  if (cmd.startsWith("Kw") || cmd.startsWith("kw")) {
    int val = cmd.substring(2).toInt();
    if (val > 0) { areaWidth_cm = val; widthSet = true; tryStartAreaMode(); }
    return "OK:width=" + String(areaWidth_cm);
  }
  if (cmd == "V")  { extraMotorOn();   return "OK:extra_on"; }
  if (cmd == "V2") { extraMotor2On();  return "OK:extra2_on"; }
  if (cmd == "P2") { extraMotor2Off(); return "OK:extra2_off"; }
  if (cmd == "P") { extraMotorOff(); return "OK:extra_off"; }
  if (cmd == "X") {
    stopMotors(); areaModeActive = false; lastCmd = "";
    return "OK:stop";
  }
  if (executeCommand(cmd)) {
    lastCmd = cmd; areaModeActive = false;
    return "OK:" + cmd;
  }
  return "ERR:unknown";
}

// =============================================================================
//  HTTP HANDLERS
// =============================================================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}
void handleCmd() {
  if (!server.hasArg("v")) { server.send(400,"text/plain","ERR:missing_param_v"); return; }
  server.send(200, "text/plain", processCmd(server.arg("v")));
}
void handleSpeed2() {
  if (!server.hasArg("v")) { server.send(400,"text/plain","ERR:missing_param_v"); return; }
  int val = server.arg("v").toInt();
  val = constrain(val, 0, 255);
  extraMotor2Speed = (uint8_t)val;
  analogWrite(MOTOR_Y_EN, extraMotor2Speed);
  server.send(200, "text/plain", "OK:speed2=" + String(extraMotor2Speed));
}

void handleSpeed() {
  if (!server.hasArg("v")) { server.send(400,"text/plain","ERR:missing_param_v"); return; }
  int val = server.arg("v").toInt();
  val = constrain(val, 0, 255);
  extraMotorSpeed = (uint8_t)val;
  // update live if motor is already on
  analogWrite(MOTOR_X_EN, extraMotorSpeed);
  server.send(200, "text/plain", "OK:speed=" + String(extraMotorSpeed));
}
void handleStatus() {
  String j = "{\"obstacle\":" + String(obstacleDetected?"true":"false") +
             ",\"area_mode\":"  + String(areaModeActive?"true":"false")  +
             ",\"last_cmd\":\""  + lastCmd + "\"" +
             ",\"extra_speed\":" + String(extraMotorSpeed) + "}";
  server.send(200, "application/json", j);
}

// =============================================================================
//  OBSTACLE AVOIDANCE
// =============================================================================
void updateUltrasonic() {
  if (millis() - lastPingTime < PING_INTERVAL_MS) return;
  lastPingTime = millis();
  unsigned int dist = sonar.ping_cm();
  bool tooClose = (dist > 0 && dist < OBSTACLE_DIST_CM);
  if (tooClose && !obstacleDetected) {
    obstacleDetected = true; stopMotors();
    Serial.println("OBSTACLE!");
  } else if (!tooClose && obstacleDetected) {
    obstacleDetected = false;
    Serial.println("CLEAR");
    if (areaModeActive) sweepTimer = millis();
    else if (lastCmd.length() > 0) executeCommand(lastCmd);
  }
}

// =============================================================================
//  AREA SWEEP
// =============================================================================
void tryStartAreaMode() {
  if (!lengthSet || !widthSet) return;
  areaModeActive = true; sweepState = SWEEP_FORWARD; sweepTurnDir = true;
  sweepTimer = millis();
  sweepLanesLeft = max(1, areaLength_cm / LANE_WIDTH_CM);
  sweepLanesDone = 0;
  Serial.printf("Area ON: L=%d W=%d Lanes=%d\n", areaLength_cm, areaWidth_cm, sweepLanesLeft);
}

void updateAreaSweep() {
  if (!areaModeActive || obstacleDetected) return;
  unsigned long el   = millis() - sweepTimer;
  unsigned long fwdMs  = (unsigned long)areaWidth_cm  * 1000UL / ROVER_SPEED_CMS;
  unsigned long sideMs = (unsigned long)LANE_WIDTH_CM * 1000UL / ROVER_SPEED_CMS;

  switch (sweepState) {
    case SWEEP_FORWARD:
      rightMotor(true,STRAIGHT_SPEED); leftMotor(true,STRAIGHT_SPEED);
      if (el >= fwdMs) { stopMotors(); sweepState=SWEEP_TURN1; sweepTimer=millis(); }
      break;
    case SWEEP_TURN1:
      if (sweepTurnDir) { rightMotor(false,TURN_SPEED); leftMotor(true,TURN_SPEED); }
      else              { rightMotor(true,TURN_SPEED);  leftMotor(false,TURN_SPEED); }
      if (el >= TURN_DURATION_MS) { stopMotors(); sweepState=SWEEP_SIDEWAYS; sweepTimer=millis(); }
      break;
    case SWEEP_SIDEWAYS:
      rightMotor(true,STRAIGHT_SPEED); leftMotor(true,STRAIGHT_SPEED);
      if (el >= sideMs) { stopMotors(); sweepLanesDone++; sweepState=SWEEP_TURN2; sweepTimer=millis(); }
      break;
    case SWEEP_TURN2:
      if (sweepTurnDir) { rightMotor(false,TURN_SPEED); leftMotor(true,TURN_SPEED); }
      else              { rightMotor(true,TURN_SPEED);  leftMotor(false,TURN_SPEED); }
      if (el >= TURN_DURATION_MS) {
        stopMotors(); sweepTurnDir=!sweepTurnDir;
        sweepState = (sweepLanesDone>=sweepLanesLeft) ? SWEEP_DONE : SWEEP_FORWARD;
        sweepTimer=millis();
      }
      break;
    case SWEEP_DONE:
      stopMotors(); areaModeActive=false;
      Serial.println("Sweep DONE!"); sweepState=SWEEP_IDLE;
      break;
    default: break;
  }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_R_IN1,OUTPUT); pinMode(MOTOR_R_IN2,OUTPUT); pinMode(MOTOR_R_EN,OUTPUT);
  pinMode(MOTOR_L_IN1,OUTPUT); pinMode(MOTOR_L_IN2,OUTPUT); pinMode(MOTOR_L_EN,OUTPUT);
  pinMode(MOTOR_X_IN1,OUTPUT); pinMode(MOTOR_X_IN2,OUTPUT); pinMode(MOTOR_X_EN,OUTPUT);
  pinMode(MOTOR_Y_IN1,OUTPUT); pinMode(MOTOR_Y_IN2,OUTPUT); pinMode(MOTOR_Y_EN,OUTPUT);
  stopMotors(); extraMotorOff(); extraMotor2Off();

  // Start Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP started — SSID: "); Serial.println(AP_SSID);
  Serial.print("Open browser at: http://"); Serial.println(ip);

  server.on("/",       handleRoot);
  server.on("/cmd",    handleCmd);
  server.on("/speed",  handleSpeed);
  server.on("/speed2", handleSpeed2);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("HTTP server started. Rover ready.");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  server.handleClient();
  updateUltrasonic();
  if (areaModeActive) updateAreaSweep();
}

