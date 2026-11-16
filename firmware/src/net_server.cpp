#include "net_server.h"
#include "wifi_config.h"
#include "control.h"
#include "config.h"
#include "encoders.h"
#include "imu.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

static WebServer g_http(HTTP_PORT);
static WebSocketsServer g_ws(WS_PORT);
static bool g_has_controller = false;
static uint8_t g_controller = 0;

static constexpr size_t MAX_WS_COMMAND_BYTES = 256;
static constexpr size_t MAX_TRAJECTORY_BODY_BYTES = 8 * 1024;

// Self-contained controls: the PC needs only a browser on the robot's access point.
static const char CONTROL_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Omni 3 bánh</title>
<style>
body{font:16px system-ui;max-width:800px;margin:24px auto;padding:0 16px;background:#f4f7f7;color:#143633}
section{background:white;border:1px solid #d1dedd;border-radius:12px;padding:18px;margin:16px 0}
button,select,input,textarea{font:inherit;padding:10px;margin:4px;box-sizing:border-box}
button{cursor:pointer}button:disabled{cursor:default}#stop{background:#a82020;color:white;border:0}
.pad{display:grid;grid-template-columns:repeat(3,80px);justify-content:center;gap:5px}
.drive{touch-action:none;user-select:none}textarea{width:95%;height:130px}pre{overflow:auto;font-size:13px}
#error{color:#a82020;white-space:pre-wrap}small{display:block;margin:8px 0;color:#47635e}
</style>
<h1>Xe Omni 3 bánh · ESP32-S3</h1>
<p id="status">Đang kết nối…</p><p id="error" role="alert"></p>
<section><h2>Hiệu chuẩn encoder</h2>
<small>Đo số cạnh lên kênh A trên một vòng bánh, gồm hộp số. Nhập riêng từng bánh; xe phải đứng yên. Cấu hình này cần xác nhận lại sau khi khởi động.</small>
<p>Nâng bánh khỏi mặt đất và đánh dấu từng bánh. Nếu bánh xoay tay được, ngắt nguồn động cơ để đo. Nếu hộp số không xoay tay được, dùng chế độ thử Serial T để motor quay có cấp điện và quan sát số vòng thực tế; không ép quay tay. Bấm STOP trước khi ghi mốc và sau khi đo. Giữ nguyên phiên ESP32 trong suốt phép đo.</p>
<label>Số vòng quay mỗi bánh <input id="revolutions" type="number" min="1" max="1000" step="1" value="5"></label><br>
<button id="capture">1. Ghi số đếm ban đầu</button><button id="calculate">2. Tính CPR sau khi quay đủ vòng</button>
<small id="measurement">Ghi mốc trước, sau đó quay từng bánh đủ số vòng cùng một chiều. Đợi cả ba bánh đứng yên rồi tính CPR.</small>
<label>Bánh 1 <input id="cpr1" type="number" min="1" max="1000000" placeholder="tick/vòng"></label><br>
<label>Bánh 2 <input id="cpr2" type="number" min="1" max="1000000" placeholder="tick/vòng"></label><br>
<label>Bánh 3 <input id="cpr3" type="number" min="1" max="1000000" placeholder="tick/vòng"></label><br>
<button id="calibrate">Xác nhận CPR đã đo</button><button id="reset">Đặt gốc odometry</button>
<button id="imuCalibration">Hiệu chuẩn lại MPU (giữ yên 4 giây)</button>
<small id="sensors">Đang chờ trạng thái encoder / MPU…</small>
</section>
<section><h2>Điều khiển</h2>
<select id="mode"><option>MANUAL</option><option>AUTO</option><option>STOP</option></select>
<button id="apply">Chọn chế độ</button><button id="start">START</button>
<button id="pause">PAUSE</button><button id="resume">RESUME</button><button id="stop">STOP</button>
<small>MANUAL: chọn chế độ, START rồi giữ nút hướng. Nhả nút để dừng. Hướng theo hệ tọa độ thân xe.</small>
<label>Mức tốc độ <input id="speed" type="range" min="0.1" max="1" step="0.05" value="0.25"></label>
<div class="pad"><span></span><button class="drive" data-x="0" data-y="1">+Y</button><span></span>
<button class="drive" data-x="-1" data-y="0">−X</button><span></span><button class="drive" data-x="1" data-y="0">+X</button>
<span></span><button class="drive" data-x="0" data-y="-1">−Y</button><span></span></div>
</section>
<section><h2>Quỹ đạo AUTO</h2><small>Mỗi dòng x,y theo mét trong hệ tọa độ odometry. Dừng xe trước khi nạp.</small>
<textarea id="points" aria-label="Các điểm x,y">0.0,0.0
0.2,0.0
0.2,0.2</textarea><br><button id="upload">Nạp quỹ đạo</button><button id="fetch">Đọc quỹ đạo</button>
<small>Sau khi nạp thành công: chọn AUTO → START. Quỹ đạo nằm trong RAM và mất khi khởi động lại.</small></section>
<section><h2>Phản hồi</h2><pre id="telemetry">Chưa có dữ liệu.</pre></section>
<script>
const $=id=>document.getElementById(id);
let socket, firmwareMode='', firmwareState='', held=null, lastRx=0;
let calibrationBase=null, calibrationTurns=0, calibrationValid=false, wizardActive=false, measuring=false;
const send=value=>{if(socket?.readyState!==WebSocket.OPEN||socket.bufferedAmount!==0)return false;
  socket.send(JSON.stringify(value));return true;};
function release(){if(held){held=null;send({type:'joystick',x:0,y:0});}}
function refresh(){const live=socket?.readyState===WebSocket.OPEN&&Date.now()-lastRx<1500;
  $('apply').disabled=!live;$('start').disabled=!live||firmwareState!=='IDLE'||!['MANUAL','AUTO'].includes(firmwareMode);
  $('pause').disabled=!live||firmwareState!=='RUNNING'||!['MANUAL','AUTO'].includes(firmwareMode);
  $('resume').disabled=!live||firmwareState!=='PAUSED';
  const idle=live&&firmwareState==='IDLE';
  $('capture').disabled=!idle||firmwareMode!=='STOP'||measuring;
  $('calculate').disabled=!idle||firmwareMode!=='STOP'||!calibrationBase||measuring;
  $('calibrate').disabled=!idle||measuring||(wizardActive&&!calibrationValid);
  $('reset').disabled=!idle||measuring;
  if(wizardActive&&!calibrationValid)$('start').disabled=true;
  document.querySelectorAll('.drive').forEach(b=>b.disabled=!live||firmwareMode!=='MANUAL'||firmwareState!=='RUNNING');
  if(!live||firmwareMode!=='MANUAL'||firmwareState!=='RUNNING')release();
  if(!live)$('status').textContent='Mất kết nối hoặc chưa có phản hồi ESP32';
}
function connect(){socket=new WebSocket('ws://'+location.hostname+':__WS_PORT__/');
  socket.onmessage=e=>{let d;try{d=JSON.parse(e.data);}catch{return;}
    if(!['status','telemetry'].includes(d.type))return;
    lastRx=Date.now();firmwareMode=d.mode;firmwareState=d.state;
    $('status').textContent='Đã kết nối · '+d.mode+' / '+d.state;
    if(d.type==='status')$('error').textContent=d.error||'';
    $('sensors').textContent='Encoder: '+(d.encoder_calibrated?'đã cấu hình CPR':'chưa cấu hình CPR')+
      ' · MPU: '+(d.imu_healthy&&d.imu_calibrated?'đã hiệu chuẩn gyro':'chưa sẵn sàng')+(d.fault?' · '+d.fault:'');
    if(d.type==='telemetry')$('telemetry').textContent=JSON.stringify(d,null,2);refresh();};
  socket.onclose=()=>{held=null;lastRx=0;if(wizardActive){calibrationBase=null;calibrationValid=false;
    for(let i=1;i<=3;i++)$('cpr'+i).value='';
    $('measurement').textContent='Mất kết nối: phép đo đã hủy. Kết nối lại rồi ghi mốc mới.';}
    refresh();setTimeout(connect,1000);};
  socket.onerror=()=>socket.close();
}
$('apply').onclick=()=>{release();send({type:'mode',value:$('mode').value});};
async function calibrationSnapshot(){const connection=socket;const response=await fetch('/status',{cache:'no-store'});
  if(!response.ok)throw Error('Không đọc được số đếm từ ESP32.');const snapshot=await response.json();
  if(connection!==socket||socket?.readyState!==WebSocket.OPEN||Date.now()-lastRx>=1500)throw Error('Kết nối đã thay đổi hoặc mất phản hồi. Ghi mốc mới rồi đo lại.');
  if(snapshot.mode!=='STOP'||snapshot.state!=='IDLE')throw Error('Bấm STOP và đợi bánh đứng yên trước khi ghi số đếm.');
  if(!Array.isArray(snapshot.ticks)||snapshot.ticks.length!==3||!snapshot.ticks.every(Number.isSafeInteger))throw Error('Số đếm encoder không hợp lệ.');
  if(!Number.isFinite(snapshot.uptime_ms))throw Error('Không có thời gian hoạt động để kiểm tra phiên đo.');
  return snapshot;}
$('capture').onclick=async()=>{if(measuring)return;measuring=true;calibrationBase=null;calibrationValid=false;wizardActive=true;
  for(let i=1;i<=3;i++)$('cpr'+i).value='';refresh();
  try{const turns=Number($('revolutions').value);if(!Number.isInteger(turns)||turns<1||turns>1000)throw Error('Số vòng phải là số nguyên từ 1 đến 1000.');
    const base=await calibrationSnapshot();calibrationBase=base;calibrationTurns=turns;$('error').textContent='';
    $('measurement').textContent='Mốc '+base.ticks.join(', ')+'. Quay MỖI bánh đúng '+turns+' vòng cùng một chiều, giữ nguyên chiều quay từng bánh. Sau đó để bánh đứng yên và bấm Tính CPR.';
  }catch(e){$('error').textContent=e.message;}finally{measuring=false;refresh();}};
$('calculate').onclick=async()=>{if(measuring||!calibrationBase)return;measuring=true;calibrationValid=false;refresh();
  try{const base=calibrationBase;const end=await calibrationSnapshot();
    if(base!==calibrationBase||end.uptime_ms<base.uptime_ms)throw Error('Phiên ESP32 đã thay đổi. Ghi mốc mới rồi đo lại.');
    const difference=end.ticks.map((tick,i)=>tick-base.ticks[i]);
    if(difference.some(delta=>!Number.isSafeInteger(delta)||delta===0))throw Error('Cả ba bánh phải có chênh lệch số đếm khác 0. Kiểm tra dây encoder và quay đủ '+calibrationTurns+' vòng cho từng bánh.');
    const cpr=difference.map(delta=>Math.abs(delta)/calibrationTurns);
    if(cpr.some(n=>n<1||n>1000000))throw Error('CPR ngoài khoảng 1–1.000.000. Kiểm tra số vòng và đo lại.');
    cpr.forEach((n,i)=>$('cpr'+(i+1)).value=String(n));calibrationValid=true;$('error').textContent='';
    $('measurement').textContent='Δticks = ['+difference.join(', ')+']; chia '+calibrationTurns+' vòng → CPR = ['+cpr.join(', ')+']. Đợi bánh đứng yên rồi bấm Xác nhận CPR đã đo. Dấu Δticks giúp kiểm tra chiều encoder.';
  }catch(e){for(let i=1;i<=3;i++)$('cpr'+i).value='';$('error').textContent=e.message;}finally{measuring=false;refresh();}};
$('calibrate').onclick=()=>{const cpr=[1,2,3].map(i=>Number($('cpr'+i).value));
  if(wizardActive&&!calibrationValid){$('error').textContent='Hoàn tất phép đo đủ cả ba bánh trước khi xác nhận.';return;}
  if(!cpr.every(n=>Number.isFinite(n)&&n>=1&&n<=1000000)){$('error').textContent='Nhập CPR đã đo cho đủ 3 bánh.';return;}
  if(!send({type:'calibration',cpr}))$('error').textContent='Chưa kết nối ESP32.';};
$('reset').onclick=()=>{release();if(!send({type:'command',value:'RESET_ODOMETRY'}))$('error').textContent='Chưa kết nối ESP32.';};
$('imuCalibration').onclick=()=>{release();send({type:'command',value:'CALIBRATE_IMU'});};
for(const id of ['start','pause','resume','stop'])$(id).onclick=()=>{release();
  if(!send({type:'command',value:id.toUpperCase()}))$('error').textContent='Không gửi được lệnh: kiểm tra kết nối.';};
document.querySelectorAll('.drive').forEach(b=>{
  b.onpointerdown=e=>{if(e.button!==0||held||b.disabled)return;e.preventDefault();b.setPointerCapture(e.pointerId);
    held={button:b,pointer:e.pointerId,x:Number(b.dataset.x),y:Number(b.dataset.y)};drive();};
  for(const kind of ['pointerup','pointercancel','lostpointercapture'])b.addEventListener(kind,e=>{if(held?.pointer===e.pointerId)release();});
});
function drive(){if(!held)return;const scale=Number($('speed').value);
  if(!send({type:'joystick',x:held.x*scale,y:held.y*scale}))release();}
addEventListener('blur',release);addEventListener('pagehide',release);
document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
setInterval(()=>{refresh();drive();},80);
$('upload').onclick=async()=>{try{const points=$('points').value.split(/\r?\n/).filter(s=>s.trim()).map((line,i)=>{
    const cells=line.split(',');if(cells.length!==2||cells.some(c=>!c.trim()))throw Error('Dòng '+(i+1)+': cần x,y.');
    const [x,y]=cells.map(Number);if(![x,y].every(n=>Number.isFinite(n)&&Math.abs(n)<=1000))throw Error('Tọa độ phải hữu hạn, trong ±1000 m.');return{x,y};});
    if(points.length<1||points.length>64)throw Error('Cần 1–64 điểm.');
    const r=await fetch('/trajectory',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({type:'trajectory',points})});
    const d=await r.json();if(!r.ok||!d.ok)throw Error(d.err||'Nạp thất bại');$('error').textContent='';$('status').textContent='Đã nạp '+points.length+' điểm. Chọn AUTO → START.';
  }catch(e){$('error').textContent=e.message;}};
$('fetch').onclick=async()=>{try{const r=await fetch('/trajectory');if(!r.ok)throw Error('Không đọc được quỹ đạo');
  const d=await r.json();$('points').value=d.points.map(p=>p.x+','+p.y).join('\n');$('error').textContent='';}catch(e){$('error').textContent=e.message;}};
refresh();connect();
</script></html>)HTML";

static const char* mode_name(ControlMode m) {
  switch (m) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO: return "AUTO";
    case MODE_STOP: return "STOP";
    case MODE_VELOCITY_GLOBAL: return "VELOCITY";
    case MODE_VELOCITY_BODY: return "BODY";
    case MODE_POSE: return "POSE";
    case MODE_MOTOR_TEST: return "MOTOR_TEST";
    default: return "STOP";
  }
}

static const char* state_name(RunState s) {
  switch (s) {
    case STATE_RUNNING: return "RUNNING";
    case STATE_PAUSED: return "PAUSED";
    default: return "IDLE";
  }
}

static void fill_snapshot(JsonDocument& doc, const ControlSnapshot& snapshot) {
  doc["mode"] = mode_name(snapshot.mode);
  doc["state"] = state_name(snapshot.state);
  doc["uptime_ms"] = millis();
  doc["x"] = snapshot.pose.x;
  doc["y"] = snapshot.pose.y;
  doc["theta"] = snapshot.pose.theta;
  doc["speed"] = snapshot.speed;
  doc["distance_m"] = snapshot.distance_m;
  doc["imu_healthy"] = snapshot.imu_healthy;
  doc["imu_calibrated"] = snapshot.imu_calibrated;
  doc["imu_init"] = imu_init_status();
  doc["imu_id"] = imu_identity();
  doc["imu_yaw_sign"] = imu_yaw_sign();
  doc["encoder_calibrated"] = snapshot.encoder_calibrated;
  doc["gyro_z_rad_s"] = snapshot.gyro_z_rad_s;
  doc["target_error_m"] = snapshot.target_error_m;
  doc["fault"] = snapshot.fault;
  doc["traj_i"] = snapshot.traj_index;
  doc["traj_n"] = snapshot.traj_count;
  doc["w1"] = snapshot.omega_meas[0];
  doc["w2"] = snapshot.omega_meas[1];
  doc["w3"] = snapshot.omega_meas[2];
  int64_t ticks[3];
  encoders_get_raw_counts(ticks);
  JsonArray tick_values = doc["ticks"].to<JsonArray>();
  JsonArray cpr_values = doc["cpr"].to<JsonArray>();
  for (int i = 0; i < 3; i++) {
    tick_values.add(ticks[i]);
    cpr_values.add(snapshot.cpr[i]);
  }
}

static String status_json(const char* error = nullptr) {
  const ControlSnapshot snapshot = control_get_snapshot();
  JsonDocument doc;
  doc["type"] = "status";
  fill_snapshot(doc, snapshot);
  if (error != nullptr) doc["error"] = error;
  String out;
  serializeJson(doc, out);
  return out;
}

static void send_status_to(uint8_t num, const char* error = nullptr) {
  String out = status_json(error);
  g_ws.sendTXT(num, out);
}

static bool read_number(JsonVariantConst value, double& result) {
  if (!value.is<double>()) return false;
  result = value.as<double>();
  return isfinite(result);
}

static void report_command_result(uint8_t num, bool accepted) {
  if (!accepted) {
    const auto snapshot = control_get_snapshot();
    send_status_to(num, snapshot.fault && snapshot.fault[0] ? snapshot.fault :
                   "command not allowed: check mode/state, encoder CPR and IMU readiness");
    return;
  }
  String out = status_json();
  g_ws.broadcastTXT(out);
}

static void handle_ws_message(uint8_t num, uint8_t* payload, size_t len) {
  if (len > MAX_WS_COMMAND_BYTES) {
    send_status_to(num, "message too large");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, len, DeserializationOption::NestingLimit(4));
  if (err || !doc.is<JsonObject>()) {
    send_status_to(num, "invalid JSON message");
    return;
  }

  const char* type = doc["type"] | "";
  if (strcmp(type, "joystick") == 0) {
    double x, y;
    if (!read_number(doc["x"], x) || !read_number(doc["y"], y) ||
        fabs(x) > 1.0 || fabs(y) > 1.0) {
      send_status_to(num, "joystick x and y must be numbers in [-1, 1]");
      return;
    }
    if (control_set_joystick(x, y)) {
      g_controller = num;
      g_has_controller = true;
    } else {
      send_status_to(num, "joystick requires MANUAL / RUNNING; select mode and START first");
    }
  } else if (strcmp(type, "calibration") == 0) {
    JsonArrayConst values = doc["cpr"].as<JsonArrayConst>();
    double cpr[3];
    if (values.isNull() || values.size() != 3 ||
        !read_number(values[0], cpr[0]) || !read_number(values[1], cpr[1]) ||
        !read_number(values[2], cpr[2])) {
      send_status_to(num, "calibration requires cpr: [wheel1, wheel2, wheel3]");
      return;
    }
    report_command_result(num, control_set_encoder_cpr(cpr[0], cpr[1], cpr[2]));
  } else if (strcmp(type, "mode") == 0) {
    const char* value = doc["value"] | "";
    ControlMode mode;
    if (strcasecmp(value, "MANUAL") == 0) {
      mode = MODE_MANUAL;
    } else if (strcasecmp(value, "AUTO") == 0) {
      mode = MODE_AUTO;
    } else if (strcasecmp(value, "STOP") == 0) {
      mode = MODE_STOP;
    } else {
      send_status_to(num, "unknown mode");
      return;
    }
    report_command_result(num, control_set_mode(mode));
  } else if (strcmp(type, "command") == 0) {
    // Keep the legacy command keys for clients that use them.
    const char* cmd = doc["value"] | "";
    if (cmd[0] == '\0') cmd = doc["command"] | "";
    if (cmd[0] == '\0') cmd = doc["cmd"] | "";
    bool accepted = true;
    if (strcasecmp(cmd, "START") == 0) {
      accepted = control_command_start();
    } else if (strcasecmp(cmd, "STOP") == 0) {
      control_command_stop();
      g_has_controller = false;
    } else if (strcasecmp(cmd, "PAUSE") == 0) {
      accepted = control_command_pause();
    } else if (strcasecmp(cmd, "RESUME") == 0) {
      accepted = control_command_resume();
    } else if (strcasecmp(cmd, "RESET_ODOMETRY") == 0) {
      accepted = control_reset_odometry();
    } else if (strcasecmp(cmd, "CALIBRATE_IMU") == 0) {
      accepted = control_recalibrate_imu();
    } else {
      send_status_to(num, "unknown command");
      return;
    }
    if (accepted && (strcasecmp(cmd, "START") == 0 || strcasecmp(cmd, "RESUME") == 0)) {
      g_controller = num;
      g_has_controller = true;
    }
    report_command_result(num, accepted);
  } else {
    send_status_to(num, "unknown message type");
  }
}

static void on_ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      send_status_to(num);
      break;
    case WStype_TEXT:
      handle_ws_message(num, payload, length);
      break;
    case WStype_DISCONNECTED:
      if (g_has_controller && g_controller == num) {
        control_command_stop();
        g_has_controller = false;
      }
      break;
    default:
      break;
  }
}

static void handle_get_trajectory(void) {
  JsonDocument doc;
  doc["type"] = "trajectory";
  JsonArray pts = doc["points"].to<JsonArray>();
  TrajPoint buf[TRAJ_MAX_POINTS];
  int n = control_get_trajectory(buf, TRAJ_MAX_POINTS);
  for (int i = 0; i < n; i++) {
    JsonObject p = pts.add<JsonObject>();
    p["x"] = buf[i].x;
    p["y"] = buf[i].y;
  }
  String out;
  serializeJson(doc, out);
  g_http.send(200, "application/json", out);
}

static void send_trajectory_error(int status, const char* error) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["err"] = error;
  String out;
  serializeJson(doc, out);
  g_http.send(status, "application/json", out);
}

static void handle_post_trajectory(void) {
  if (!g_http.hasArg("plain")) {
    send_trajectory_error(400, "no body");
    return;
  }
  const String body = g_http.arg("plain");
  if (body.length() > MAX_TRAJECTORY_BODY_BYTES) {
    send_trajectory_error(413, "body must be at most 8 KiB");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body, DeserializationOption::NestingLimit(4));
  if (err || !doc.is<JsonObject>()) {
    send_trajectory_error(400, "invalid JSON object");
    return;
  }
  if (!doc["type"].isUnbound() && strcmp(doc["type"] | "", "trajectory") != 0) {
    send_trajectory_error(400, "type must be trajectory when provided");
    return;
  }
  JsonArrayConst pts = doc["points"].as<JsonArrayConst>();
  if (pts.isNull() || pts.size() == 0 || pts.size() > TRAJ_MAX_POINTS) {
    send_trajectory_error(400, "points must be an array with 1 to 64 elements");
    return;
  }
  TrajPoint buf[TRAJ_MAX_POINTS];
  int n = 0;
  for (JsonVariantConst p : pts) {
    double x, y;
    if (!p.is<JsonObjectConst>() || !read_number(p["x"], x) || !read_number(p["y"], y) ||
        fabs(x) > WORLD_COORD_LIMIT_M || fabs(y) > WORLD_COORD_LIMIT_M) {
      send_trajectory_error(400, "each point needs numeric x and y in [-1000, 1000] m");
      return;
    }
    buf[n++] = {static_cast<float>(x), static_cast<float>(y)};
  }
  const TrajectoryResult result = control_set_trajectory(buf, n);
  if (result == TrajectoryResult::Busy) {
    send_trajectory_error(409, "stop the current run before uploading");
    return;
  }
  if (result != TrajectoryResult::Ok) {
    send_trajectory_error(400, "invalid trajectory");
    return;
  }
  g_http.send(200, "application/json", "{\"ok\":true}");
}

static void handle_root(void) {
  String page = FPSTR(CONTROL_PAGE);
  page.replace("__WS_PORT__", String(WS_PORT));
  g_http.send(200, "text/html; charset=utf-8", page);
}

static void setup_wifi(void) {
#if WIFI_USE_STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("STA IP: "));
    Serial.println(WiFi.localIP());
    return;
  }
  Serial.println(F("STA failed, falling back to SoftAP"));
#endif
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const IPAddress ap_ip(192, 168, 4, 1);
  bool ok = WiFi.softAPConfig(ap_ip, ap_ip, IPAddress(255, 255, 255, 0)) &&
            WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
  Serial.printf("SoftAP %s (%s) IP=%s\n",
                ok ? "OK" : "FAIL",
                WIFI_AP_SSID,
                WiFi.softAPIP().toString().c_str());
  if (ok) Serial.println(F("Open http://192.168.4.1/ on the PC connected to this AP"));
}

void net_server_init(void) {
  setup_wifi();

  g_http.on("/", HTTP_GET, handle_root);
  g_http.on("/status", HTTP_GET, []() {
    g_http.sendHeader("Cache-Control", "no-store");
    g_http.send(200, "application/json", status_json());
  });
  g_http.on("/trajectory", HTTP_GET, handle_get_trajectory);
  g_http.on("/trajectory", HTTP_POST, handle_post_trajectory);
  g_http.begin();

  g_ws.begin();
  g_ws.onEvent(on_ws_event);
  // Detect lost controllers even when the network disappears without a close frame.
  g_ws.enableHeartbeat(1000, 1000, 2);

  Serial.printf("HTTP :%d  WS :%d\n", HTTP_PORT, WS_PORT);
}

void net_server_poll_http(void) {
  g_http.handleClient();
}

void net_server_poll_ws(void) {
  g_ws.loop();
}

void net_server_send_telemetry(void) {
  const ControlSnapshot snapshot = control_get_snapshot();
  JsonDocument doc;
  doc["type"] = "telemetry";
  fill_snapshot(doc, snapshot);
  doc["battery"] = -1;
  String out;
  serializeJson(doc, out);
  g_ws.broadcastTXT(out);
}
