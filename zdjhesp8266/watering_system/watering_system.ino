/*
 * ESP8266 自动浇水系统
 * 硬件：NodeMCU/ESP8266 + 电容式湿度传感器 v2.0 + 5V水泵 + 光耦继电器模块
 *
 * 依赖库（Arduino库管理器安装）：
 *   - ESP8266WiFi / ESP8266WebServer / LittleFS  (ESP8266 core自带)
 *   - ArduinoJson (v6.x)
 *   - NTPClient (Fabrice Weinberg)
 *
 * 烧录前：
 *   1. 工具 -> Flash Size 选择 "4MB (FS:1MB)" 或更大，留出LittleFS空间
 *   2. 用 LittleFS 上传插件，把 data/ 目录（index.html等）上传到LittleFS
 *   3. 不需要再手动填WiFi账号密码了！首次开机如果没有保存过WiFi，
 *      设备会自动开启一个配置热点，见下方"WiFi连接说明"
 *
 * WiFi连接说明：
 *   - 开机后设备会先尝试连接之前保存过的WiFi
 *   - 如果没保存过，或者连接失败（密码错/信号差/路由器不兼容等），
 *     设备会自动切换成热点模式，热点名称见 AP_SSID，密码见 AP_PASSWORD
 *   - 用手机/电脑连接这个热点后，浏览器访问 192.168.4.1，
 *     在"WiFi管理"卡片里扫描并选择你家的WiFi、填密码、保存
 *   - 保存后设备会自动重启并尝试用新配置连接
 *   - 处于热点模式时，设备仍会在后台每30秒自动重试一次已保存的WiFi
 *     （如果路由器只是临时故障，不需要你手动做任何操作就能自愈，
 *      短暂掉线/连接失败绝不会清空已保存的WiFi配置，只有下面的恢复出厂设置才会清空）
 *
 * 恢复出厂设置：
 *   - D6 (GPIO12) 接一个按键，另一端接GND（或者直接用杜邦线短接D6和GND也可以测试）
 *   - 按住(短接)满10秒，设备会清空WiFi配置、浇水配置、定时任务、日志，然后自动重启
 *   - 重启后会重新进入配置热点模式，就像全新设备一样
 *   - 按下期间板载LED会快速闪烁提示倒计时，松开则取消操作
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <uri/UriBraces.h>

// ---------------- 引脚定义 ----------------
#define PIN_MOISTURE   A0   // 电容式湿度传感器 AOUT -> A0 (唯一ADC)
#define PIN_RELAY      13    // D7 (GPIO13) -> 继电器模块 IN
#define PIN_STATUS_LED 2    // D4 (GPIO2) -> 板载LED，WiFi/运行状态指示
#define PIN_FACTORY_RESET 12 // D6 (GPIO12) -> 恢复出厂设置按键，接地(短接GND)持续10秒触发
#define RELAY_ACTIVE_LOW true  // 大多数光耦继电器模块为低电平触发，按模块实测调整

// ---------------- WiFi配置热点参数 ----------------
const char* AP_SSID = "ESP8266-Watering-Setup"; // 连不上家里WiFi时，设备自己开的热点名字
const char* AP_PASSWORD = "12345678";           // 热点密码，至少8位(WPA2要求)
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000; // 开机时尝试连WiFi的超时时间
const unsigned long WIFI_BG_RETRY_INTERVAL_MS = 30000; // AP模式下后台重试已保存WiFi的间隔

// ---------------- 网络连接质量冗余参数 ----------------
const int RSSI_ROAM_CHECK_THRESHOLD = -70;  // 信号弱于此值时，才触发一次"是否有更强同名AP"的评估
const int RSSI_ROAM_HYSTERESIS = 8;         // 候选AP必须比当前AP强至少这么多dB才切换，防止两个AP之间来回跳动
const unsigned long ROAM_CHECK_INTERVAL_MS = 60000; // 每隔多久评估一次漫游(仅在信号弱时才会真正扫描)
const unsigned long RECONNECT_BACKOFF_BASE_MS = 5000;   // 断线重连的起始退避时间
const unsigned long RECONNECT_BACKOFF_MAX_MS  = 300000; // 退避时间上限(5分钟)

// ---------------- 全局对象 ----------------
ESP8266WebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com", 8 * 3600, 60000); // 东八区
const char* LOG_TZ_SUFFIX = "+08:00"; // 与上面NTPClient的时区偏移保持一致，仅用于日志上送的ISO8601时间戳

const char* CONFIG_PATH      = "/config.json";
const char* TASKS_PATH       = "/tasks.json";
const char* WIFI_CONFIG_PATH = "/wifi.json";
const char* LOGSERVER_CONFIG_PATH = "/logserver.json";
const char* LOG_PATH         = "/log.txt";
const size_t LOG_MAX_BYTES = 40 * 1024; // 日志超过40KB自动截断，保护flash寿命

// ---------------- 运行时状态 ----------------
struct Config {
  int moistureThresholdOn;   // 低于此值(0-100%)自动开始浇水
  int moistureThresholdOff;  // 高于此值提前停止浇水（仅对自动触发的浇水生效）
  int autoWaterSeconds;      // 自动浇水单次最长时长(秒)，作为安全上限
  int autoCooldownMinutes;   // 两次自动浇水最小间隔(分钟)
  bool autoModeEnabled;      // 是否启用湿度自动浇水
  int adcDry;                // 传感器在空气中(干燥)的ADC读数，用于百分比换算
  int adcWet;                // 传感器在水中(湿润)的ADC读数
} config;

struct Task {
  String id;
  bool enabled;
  int hour;
  int minute;
  int durationSeconds;
  bool days[7]; // 周日=0 ... 周六=6
};

#define MAX_TASKS 20
Task tasks[MAX_TASKS];
int taskCount = 0;

enum PumpSource { SRC_NONE, SRC_MANUAL, SRC_SCHEDULE, SRC_AUTO };
PumpSource pumpSource = SRC_NONE;

bool ntpSynced = false;
bool pumpState = false;
unsigned long pumpStartMillis = 0;
unsigned long pumpDurationMs = 0;
bool everAutoWatered = false; // 是否已触发过至少一次自动浇水，避免开机瞬间被冷却时间误判卡住
unsigned long lastAutoWaterMillis = 0;
int lastTriggeredMinuteOfDay = -1; // 防止同一分钟内定时任务重复触发

String wifiSsid = "";
String wifiPass = "";
bool apMode = false; // 当前是否处于"配置热点"模式

const unsigned long FACTORY_RESET_HOLD_MS = 10000; // 按住多久触发恢复出厂设置
unsigned long resetHoldStartMillis = 0;
bool resetHoldActive = false;

int reconnectAttempts = 0;              // 连续重连失败次数，用于计算退避时间
unsigned long nextReconnectAllowedAt = 0; // 下一次允许尝试重连的时间点
unsigned long lastRoamCheckMillis = 0;    // 上一次漫游评估的时间

// ---------------- 日志上送(日志服务器) ----------------
struct LogServerConfig {
  bool enabled;
  String type;   // "syslog"(UDP, RFC5424格式) 或 "http"(HTTP POST, JSON批量)
  String host;   // IP或域名
  int port;
  String path;   // 仅http使用，如 /log
} logServer = {false, "syslog", "", 514, "/log"};

#define LOG_QUEUE_SIZE 16  // 待上送队列长度(内存环形队列，满了丢弃最旧的)
#define LOG_BATCH_MAX 8    // 每次最多上送几条
String logQTs[LOG_QUEUE_SIZE];   // ISO8601时间戳，NTP未同步时为空
String logQMsg[LOG_QUEUE_SIZE];
int logQHead = 0;
int logQCount = 0;
unsigned long logSentCount = 0;
unsigned long logFailCount = 0;
unsigned long logDroppedCount = 0;
int logConsecutiveFails = 0;
unsigned long logNextSendAllowedAt = 0;
String logLastError = "";
WiFiUDP logUdp;

// ---------------- 水泵控制 ----------------
void setPumpRaw(bool on) {
  pumpState = on;
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_RELAY, level ? HIGH : LOW);
}

void appendLog(const String& msg);

void startPump(unsigned long durationMs, PumpSource src) {
  setPumpRaw(true);
  pumpStartMillis = millis();
  pumpDurationMs = durationMs;
  pumpSource = src;
}

void stopPump(const String& reason) {
  setPumpRaw(false);
  pumpDurationMs = 0;
  pumpSource = SRC_NONE;
  appendLog(reason);
}

// ---------------- 日志/湿度工具函数 ----------------
// 生成日志时间戳：plain为写入文件的"年-月-日 时:分:秒"，iso为上送日志服务器的ISO8601格式
// 返回false表示NTP尚未同步(此时plain用"开机后秒数"代替，iso为空)
bool buildTimestamp(String& plain, String& iso) {
  time_t t = timeClient.getEpochTime(); // NTPClient返回的epoch已含时区偏移，直接按gmtime拆分即为本地时间
  struct tm tmv;
  gmtime_r(&t, &tmv);
  if (!ntpSynced || tmv.tm_year + 1900 < 2020) {
    plain = "[开机+" + String(millis() / 1000) + "s]";
    iso = "";
    return false;
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  plain = buf;
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  iso = String(buf) + LOG_TZ_SUFFIX;
  return true;
}

void enqueueLog(const String& iso, const String& msg) {
  if (logQCount >= LOG_QUEUE_SIZE) { // 队列满，丢弃最旧一条
    logQTs[logQHead] = "";
    logQMsg[logQHead] = "";
    logQHead = (logQHead + 1) % LOG_QUEUE_SIZE;
    logQCount--;
    logDroppedCount++;
  }
  int tail = (logQHead + logQCount) % LOG_QUEUE_SIZE;
  logQTs[tail] = iso;
  logQMsg[tail] = msg;
  logQCount++;
}

void appendLog(const String& msg) {
  String plain, iso;
  buildTimestamp(plain, iso);

  // 先入上送队列(实际发送在loop里异步进行，不会阻塞这里的调用方)
  if (logServer.enabled && logServer.host.length() > 0) enqueueLog(iso, msg);

  File f = LittleFS.open(LOG_PATH, "a");
  if (!f) return;
  if (f.size() > LOG_MAX_BYTES) {
    f.close();
    f = LittleFS.open(LOG_PATH, "w"); // 简单截断策略：超限清空重开
  }
  f.print(plain + " " + msg + "\n");
  f.close();
}

int readMoisturePercent() {
  if (config.adcDry == config.adcWet) return 0; // 防止两个基准相同导致map()除零崩溃
  int raw = analogRead(PIN_MOISTURE); // 0-1023
  int pct = map(raw, config.adcDry, config.adcWet, 0, 100);
  pct = constrain(pct, 0, 100);
  return pct;
}

// ---------------- 配置持久化 ----------------
void loadConfig() {
  config = {50, 65, 5, 30, true, 800, 300}; // 默认值
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) return;
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, f) == DeserializationError::Ok) {
    config.moistureThresholdOn  = doc["moistureThresholdOn"]  | config.moistureThresholdOn;
    config.moistureThresholdOff = doc["moistureThresholdOff"] | config.moistureThresholdOff;
    config.autoWaterSeconds     = doc["autoWaterSeconds"]     | config.autoWaterSeconds;
    config.autoCooldownMinutes  = doc["autoCooldownMinutes"]  | config.autoCooldownMinutes;
    config.autoModeEnabled      = doc["autoModeEnabled"]      | config.autoModeEnabled;
    config.adcDry                = doc["adcDry"]                | config.adcDry;
    config.adcWet                = doc["adcWet"]                | config.adcWet;
  }
  f.close();
}

void saveConfig() {
  StaticJsonDocument<512> doc;
  doc["moistureThresholdOn"]  = config.moistureThresholdOn;
  doc["moistureThresholdOff"] = config.moistureThresholdOff;
  doc["autoWaterSeconds"]     = config.autoWaterSeconds;
  doc["autoCooldownMinutes"]  = config.autoCooldownMinutes;
  doc["autoModeEnabled"]      = config.autoModeEnabled;
  doc["adcDry"]                = config.adcDry;
  doc["adcWet"]                = config.adcWet;
  File f = LittleFS.open(CONFIG_PATH, "w");
  serializeJson(doc, f);
  f.close();
}

void loadTasks() {
  taskCount = 0;
  File f = LittleFS.open(TASKS_PATH, "r");
  if (!f) return;
  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, f) == DeserializationError::Ok) {
    JsonArray arr = doc.as<JsonArray>();
    for (JsonObject o : arr) {
      if (taskCount >= MAX_TASKS) break;
      Task& t = tasks[taskCount];
      t.id = o["id"].as<String>();
      t.enabled = o["enabled"] | true;
      t.hour = o["hour"] | 6;
      t.minute = o["minute"] | 0;
      t.durationSeconds = o["durationSeconds"] | 10;
      JsonArray d = o["days"];
      for (int i = 0; i < 7; i++) t.days[i] = (i < (int)d.size()) ? (bool)d[i] : false;
      taskCount++;
    }
  }
  f.close();
}

void saveTasks() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < taskCount; i++) {
    JsonObject o = arr.createNestedObject();
    o["id"] = tasks[i].id;
    o["enabled"] = tasks[i].enabled;
    o["hour"] = tasks[i].hour;
    o["minute"] = tasks[i].minute;
    o["durationSeconds"] = tasks[i].durationSeconds;
    JsonArray d = o.createNestedArray("days");
    for (int j = 0; j < 7; j++) d.add(tasks[i].days[j]);
  }
  File f = LittleFS.open(TASKS_PATH, "w");
  serializeJson(doc, f);
  f.close();
}

// ---------------- WiFi凭证持久化 ----------------
bool loadWifiConfig() {
  File f = LittleFS.open(WIFI_CONFIG_PATH, "r");
  if (!f) return false;
  StaticJsonDocument<256> doc;
  bool ok = (deserializeJson(doc, f) == DeserializationError::Ok);
  f.close();
  if (!ok) return false;
  wifiSsid = doc["ssid"].as<String>();
  wifiPass = doc["password"].as<String>();
  return wifiSsid.length() > 0;
}

void saveWifiConfig(const String& ssid, const String& pass) {
  StaticJsonDocument<256> doc;
  doc["ssid"] = ssid;
  doc["password"] = pass;
  File f = LittleFS.open(WIFI_CONFIG_PATH, "w");
  serializeJson(doc, f);
  f.close();
}

// 尝试以STA模式连接指定WiFi，成功返回true。timeoutMs内阻塞等待（仅用于开机及AP模式下的后台重试）。
bool tryConnectWifi(const String& ssid, const String& pass, unsigned long timeoutMs) {
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.print("正在连接WiFi: "); Serial.println(ssid);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(300);
    Serial.print(".");
    digitalWrite(PIN_STATUS_LED, (millis() / 300) % 2);
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

// ---------------- 恢复出厂设置 ----------------
void doFactoryReset() {
  Serial.println(F("!! 检测到恢复出厂设置操作，正在清空所有配置... !!"));
  if (LittleFS.exists(WIFI_CONFIG_PATH)) LittleFS.remove(WIFI_CONFIG_PATH);
  if (LittleFS.exists(CONFIG_PATH))      LittleFS.remove(CONFIG_PATH);
  if (LittleFS.exists(TASKS_PATH))       LittleFS.remove(TASKS_PATH);
  if (LittleFS.exists(LOG_PATH))         LittleFS.remove(LOG_PATH);
  if (LittleFS.exists(LOGSERVER_CONFIG_PATH)) LittleFS.remove(LOGSERVER_CONFIG_PATH);
  Serial.println(F("已清空，3秒后重启进入初始配置模式..."));
  delay(3000);
  ESP.restart();
}

// 每次loop调用：检测复位按键是否被持续按住(接地)达到指定时长
void checkFactoryResetButton() {
  bool pressed = (digitalRead(PIN_FACTORY_RESET) == LOW); // 按下/短接GND时为低电平(内部上拉)
  if (pressed) {
    if (!resetHoldActive) {
      resetHoldActive = true;
      resetHoldStartMillis = millis();
      Serial.println(F("检测到复位按键按下，持续按住10秒将恢复出厂设置..."));
    } else {
      unsigned long held = millis() - resetHoldStartMillis;
      // 按住期间LED快闪提示，越接近10秒闪得越快
      unsigned long blinkPeriod = held < 5000 ? 400 : 150;
      digitalWrite(PIN_STATUS_LED, (millis() / blinkPeriod) % 2);
      if (held >= FACTORY_RESET_HOLD_MS) {
        doFactoryReset(); // 内部会重启，不会返回
      }
    }
  } else {
    if (resetHoldActive) {
      Serial.println(F("复位按键已松开，取消恢复出厂设置"));
    }
    resetHoldActive = false;
  }
}

// 评估是否存在同SSID、信号明显更强的其他AP，如果有则主动切换过去(软件模拟漫游)
void evaluateRoaming() {
  if (apMode || WiFi.status() != WL_CONNECTED) return;

  int currentRssi = WiFi.RSSI();
  if (currentRssi > RSSI_ROAM_CHECK_THRESHOLD) return; // 信号本来就不错，不用折腾

  Serial.print(F("[漫游评估] 当前信号较弱("));
  Serial.print(currentRssi);
  Serial.println(F("dBm)，扫描是否有更强的同名AP..."));

  uint8_t currentBssid[6];
  memcpy(currentBssid, WiFi.BSSID(), 6);

  int n = WiFi.scanNetworks(false, false); // 前台扫描：会短暂打断几百毫秒的数据收发，但不会掉线
  int bestRssi = currentRssi;
  int bestIndex = -1;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) != wifiSsid) continue;
    const uint8_t* candBssid = WiFi.BSSID(i);
    if (memcmp(candBssid, currentBssid, 6) == 0) continue; // 就是当前这台AP，跳过
    if (WiFi.RSSI(i) > bestRssi + RSSI_ROAM_HYSTERESIS) {
      bestRssi = WiFi.RSSI(i);
      bestIndex = i;
    }
  }

  if (bestIndex >= 0) {
    uint8_t targetBssid[6];
    memcpy(targetBssid, WiFi.BSSID(bestIndex), 6);
    int32_t targetChannel = WiFi.channel(bestIndex);
    Serial.print(F("[漫游评估] 发现更强AP, 信号:"));
    Serial.print(bestRssi);
    Serial.println(F("dBm，主动切换中..."));
    WiFi.disconnect();
    delay(100);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str(), targetChannel, targetBssid);
    appendLog("主动漫游: 信号" + String(currentRssi) + "->切换目标" + String(bestRssi) + "dBm");
  } else {
    Serial.println(F("[漫游评估] 未发现更强的同名AP，继续保持当前连接"));
  }
  WiFi.scanDelete();
}

void startApMode() {
  apMode = true;
  WiFi.mode(WIFI_AP_STA); // AP+STA共存：既能提供配置热点，也能扫描周边WiFi
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.println(F("========================================"));
  Serial.println(F("未能连接到已保存的WiFi，已启动配置热点"));
  Serial.print(F("热点名称: ")); Serial.println(AP_SSID);
  Serial.print(F("热点密码: ")); Serial.println(AP_PASSWORD);
  Serial.print(F("配置地址: http://")); Serial.println(WiFi.softAPIP());
  Serial.println(F("请用手机/电脑连接该热点，浏览器打开上面的地址进行WiFi配置"));
  Serial.println(F("========================================"));
}

// ---------------- 日志上送实现 ----------------
String deviceId() {
  String m = WiFi.macAddress();
  m.replace(":", "");
  m.toLowerCase();
  return "watering-" + m.substring(6);
}

bool resolveLogHost(IPAddress& ip) {
  if (ip.fromString(logServer.host)) return true; // 已经是IP就不走DNS
  return WiFi.hostByName(logServer.host.c_str(), ip) == 1;
}

void dequeueLog(int n) {
  for (int i = 0; i < n && logQCount > 0; i++) {
    logQTs[logQHead] = "";
    logQMsg[logQHead] = "";
    logQHead = (logQHead + 1) % LOG_QUEUE_SIZE;
    logQCount--;
    logSentCount++;
  }
}

// Syslog(UDP)，RFC5424格式：<PRI>1 时间 主机名 应用名 - - - 内容
bool sendLogsSyslog() {
  IPAddress ip;
  if (!resolveLogHost(ip)) { logLastError = "DNS解析失败"; return false; }
  String id = deviceId();
  int sent = 0;
  while (logQCount > 0 && sent < LOG_BATCH_MAX) {
    String ts = logQTs[logQHead].length() > 0 ? logQTs[logQHead] : String("-");
    String pkt = "<14>1 " + ts + " " + id + " watering - - - " + logQMsg[logQHead];
    logUdp.beginPacket(ip, logServer.port);
    logUdp.write((const uint8_t*)pkt.c_str(), pkt.length());
    if (!logUdp.endPacket()) { logLastError = "UDP发送失败"; return false; }
    dequeueLog(1);
    sent++;
    delay(2);
  }
  return true;
}

// HTTP POST(明文HTTP，不支持HTTPS)，body: {"device":"...","logs":[{"time":"...","msg":"..."},...]}
bool sendLogsHttp() {
  int n = logQCount < LOG_BATCH_MAX ? logQCount : LOG_BATCH_MAX;
  DynamicJsonDocument doc(3072);
  doc["device"] = deviceId();
  JsonArray arr = doc.createNestedArray("logs");
  for (int i = 0; i < n; i++) {
    int idx = (logQHead + i) % LOG_QUEUE_SIZE;
    JsonObject o = arr.createNestedObject();
    o["time"] = logQTs[idx];
    o["msg"] = logQMsg[idx];
  }
  String body;
  serializeJson(doc, body);

  WiFiClient client;
  client.setTimeout(2000);
  if (!client.connect(logServer.host.c_str(), (uint16_t)logServer.port)) { // 连接超时由上面的setTimeout(2000)控制
    logLastError = "连接日志服务器失败";
    return false;
  }
  client.print(String("POST ") + logServer.path + " HTTP/1.1\r\nHost: " + logServer.host +
               "\r\nContent-Type: application/json\r\nContent-Length: " + String(body.length()) +
               "\r\nConnection: close\r\n\r\n");
  client.print(body);

  unsigned long start = millis();
  while (!client.available() && client.connected() && millis() - start < 2000) delay(10);
  String status = client.readStringUntil('\n');
  client.stop();
  int sp = status.indexOf(' ');
  int code = sp > 0 ? status.substring(sp + 1).toInt() : 0;
  if (code < 200 || code >= 300) {
    logLastError = "服务器返回HTTP " + String(code);
    return false;
  }
  dequeueLog(n);
  return true;
}

// 在loop中调用：有待上送日志且网络正常时发送；失败按指数退避(5s,10s,20s...上限60s)，避免反复卡住主循环
void flushLogQueue() {
  if (!logServer.enabled || logServer.host.length() == 0 || logQCount == 0) return;
  if (apMode || WiFi.status() != WL_CONNECTED) return;
  if (millis() < logNextSendAllowedAt) return;

  bool ok = (logServer.type == "http") ? sendLogsHttp() : sendLogsSyslog();
  if (ok) {
    logConsecutiveFails = 0;
    logLastError = "";
    logNextSendAllowedAt = millis() + 300;
  } else {
    logFailCount++;
    logConsecutiveFails++;
    unsigned long backoff = 5000UL << (logConsecutiveFails - 1 > 4 ? 4 : logConsecutiveFails - 1);
    if (backoff > 60000UL) backoff = 60000UL;
    logNextSendAllowedAt = millis() + backoff;
    Serial.print(F("[日志上送] 失败: ")); Serial.print(logLastError);
    Serial.print(F("，")); Serial.print(backoff / 1000); Serial.println(F("秒后重试"));
  }
}

void loadLogServerConfig() {
  File f = LittleFS.open(LOGSERVER_CONFIG_PATH, "r");
  if (!f) return;
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, f) == DeserializationError::Ok) {
    logServer.enabled = doc["enabled"] | false;
    logServer.type = (const char*)(doc["type"] | "syslog");
    logServer.host = (const char*)(doc["host"] | "");
    logServer.port = doc["port"] | 514;
    logServer.path = (const char*)(doc["path"] | "/log");
  }
  f.close();
}

void saveLogServerConfig() {
  StaticJsonDocument<384> doc;
  doc["enabled"] = logServer.enabled;
  doc["type"] = logServer.type;
  doc["host"] = logServer.host;
  doc["port"] = logServer.port;
  doc["path"] = logServer.path;
  File f = LittleFS.open(LOGSERVER_CONFIG_PATH, "w");
  serializeJson(doc, f);
  f.close();
}

void handleGetLogServer() {
  StaticJsonDocument<512> doc;
  doc["enabled"] = logServer.enabled;
  doc["type"] = logServer.type;
  doc["host"] = logServer.host;
  doc["port"] = logServer.port;
  doc["path"] = logServer.path;
  doc["queued"] = logQCount;
  doc["sent"] = logSentCount;
  doc["failed"] = logFailCount;
  doc["dropped"] = logDroppedCount;
  doc["lastError"] = logLastError;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handlePostLogServer() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "text/plain", "bad json"); return;
  }
  bool en = doc["enabled"] | logServer.enabled;
  String type = (const char*)(doc["type"] | logServer.type.c_str());
  String host = (const char*)(doc["host"] | logServer.host.c_str());
  int port = doc["port"] | logServer.port;
  String path = (const char*)(doc["path"] | logServer.path.c_str());
  host.trim();
  path.trim();

  if (type != "syslog" && type != "http") { server.send(400, "text/plain", "type must be syslog or http"); return; }
  if (port < 1 || port > 65535) { server.send(400, "text/plain", "bad port"); return; }
  if (host.length() > 63) { server.send(400, "text/plain", "host too long"); return; }
  if (en && host.length() == 0) { server.send(400, "text/plain", "host empty"); return; }
  if (path.length() == 0) path = "/log";
  if (!path.startsWith("/")) path = "/" + path;
  if (path.length() > 63) { server.send(400, "text/plain", "path too long"); return; }

  logServer.enabled = en;
  logServer.type = type;
  logServer.host = host;
  logServer.port = port;
  logServer.path = path;
  saveLogServerConfig();

  // 服务器变了，之前排队的旧日志不再发送；同时清空失败计数让新配置立刻生效
  for (int i = 0; i < LOG_QUEUE_SIZE; i++) { logQTs[i] = ""; logQMsg[i] = ""; }
  logQHead = 0;
  logQCount = 0;
  logConsecutiveFails = 0;
  logNextSendAllowedAt = 0;
  logLastError = "";

  appendLog("日志服务器配置已更新: " + String(en ? "启用 " : "停用 ") + type + " " + host + ":" + String(port));
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTestLogServer() {
  if (!logServer.enabled || logServer.host.length() == 0) { server.send(400, "text/plain", "log server not enabled"); return; }
  appendLog("日志上送测试消息");
  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------- API 处理：状态/配置 ----------------
void handleStatus() {
  StaticJsonDocument<384> doc;
  doc["moisture"] = readMoisturePercent();
  doc["rawAdc"] = analogRead(PIN_MOISTURE);
  doc["pump"] = pumpState;
  doc["autoModeEnabled"] = config.autoModeEnabled;
  doc["wifiConnected"] = WiFi.status() == WL_CONNECTED;
  doc["apMode"] = apMode;
  doc["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  if (WiFi.status() == WL_CONNECTED) {
    doc["bssid"] = WiFi.BSSIDstr();
    doc["channel"] = WiFi.channel();
  }
  doc["time"] = ntpSynced ? timeClient.getFormattedTime() : "--:--:--";
  doc["ntpSynced"] = ntpSynced;
  doc["uptimeSec"] = millis() / 1000;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// 连续采样16次取平均，减少单次读数抖动，供网页校准"自动获取"使用
void handleRawSample() {
  const int N = 16;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(PIN_MOISTURE);
    delay(5);
  }
  server.send(200, "application/json", "{\"raw\":" + String((int)(sum / N)) + "}");
}

void handleGetConfig() {
  StaticJsonDocument<512> doc;
  doc["moistureThresholdOn"] = config.moistureThresholdOn;
  doc["moistureThresholdOff"] = config.moistureThresholdOff;
  doc["autoWaterSeconds"] = config.autoWaterSeconds;
  doc["autoCooldownMinutes"] = config.autoCooldownMinutes;
  doc["autoModeEnabled"] = config.autoModeEnabled;
  doc["adcDry"] = config.adcDry;
  doc["adcWet"] = config.adcWet;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handlePostConfig() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "text/plain", "bad json"); return;
  }
  Config backup = config; // 校验失败时整体回滚，避免只改了一半
  config.moistureThresholdOn  = doc["moistureThresholdOn"]  | config.moistureThresholdOn;
  config.moistureThresholdOff = doc["moistureThresholdOff"] | config.moistureThresholdOff;
  config.autoWaterSeconds     = doc["autoWaterSeconds"]     | config.autoWaterSeconds;
  config.autoCooldownMinutes  = doc["autoCooldownMinutes"]  | config.autoCooldownMinutes;
  config.autoModeEnabled      = doc["autoModeEnabled"]      | config.autoModeEnabled;
  config.adcDry                = doc["adcDry"]                | config.adcDry;
  config.adcWet                = doc["adcWet"]                | config.adcWet;
  // 校准基准值合法性校验：范围0-1023，且干燥/湿润基准不能相同
  if (config.adcDry < 0 || config.adcDry > 1023 || config.adcWet < 0 || config.adcWet > 1023) {
    config = backup;
    server.send(400, "text/plain", "adc value out of range (0-1023)"); return;
  }
  if (config.adcDry == config.adcWet) {
    config = backup;
    server.send(400, "text/plain", "adcDry and adcWet must differ"); return;
  }
  // 简单合法性校验：避免用户填反导致自动浇水/停止逻辑永远不触发
  if (config.moistureThresholdOff <= config.moistureThresholdOn) {
    config.moistureThresholdOff = config.moistureThresholdOn + 5;
  }
  saveConfig();
  appendLog("配置已更新");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleGetTasks() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < taskCount; i++) {
    JsonObject o = arr.createNestedObject();
    o["id"] = tasks[i].id;
    o["enabled"] = tasks[i].enabled;
    o["hour"] = tasks[i].hour;
    o["minute"] = tasks[i].minute;
    o["durationSeconds"] = tasks[i].durationSeconds;
    JsonArray d = o.createNestedArray("days");
    for (int j = 0; j < 7; j++) d.add(tasks[i].days[j]);
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handlePostTask() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "text/plain", "bad json"); return;
  }
  String id = doc["id"] | "";
  int idx = -1;
  for (int i = 0; i < taskCount; i++) if (tasks[i].id == id) { idx = i; break; }
  if (idx < 0) {
    if (taskCount >= MAX_TASKS) { server.send(400, "text/plain", "task limit reached"); return; }
    idx = taskCount++;
    tasks[idx].id = String(millis());
  }
  Task& t = tasks[idx];
  t.enabled = doc["enabled"] | true;
  t.hour = doc["hour"] | 6;
  t.minute = doc["minute"] | 0;
  t.durationSeconds = doc["durationSeconds"] | 10;
  JsonArray d = doc["days"];
  for (int i = 0; i < 7; i++) t.days[i] = (i < (int)d.size()) ? (bool)d[i] : false;
  saveTasks();
  appendLog("定时任务已保存: " + t.id);
  server.send(200, "application/json", "{\"ok\":true,\"id\":\"" + t.id + "\"}");
}

void handleDeleteTask() {
  String id = server.pathArg(0);
  int idx = -1;
  for (int i = 0; i < taskCount; i++) if (tasks[i].id == id) { idx = i; break; }
  if (idx < 0) { server.send(404, "text/plain", "not found"); return; }
  for (int i = idx; i < taskCount - 1; i++) tasks[i] = tasks[i + 1];
  taskCount--;
  saveTasks();
  appendLog("定时任务已删除: " + id);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleManualPump() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
  StaticJsonDocument<128> doc;
  deserializeJson(doc, server.arg("plain"));
  String action = doc["action"] | "off";
  int duration = doc["durationSeconds"] | 10;
  if (action == "on") {
    startPump((unsigned long)duration * 1000UL, SRC_MANUAL);
    appendLog("手动浇水开始, 时长=" + String(duration) + "s");
  } else {
    stopPump("手动浇水停止");
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleGetLogs() {
  File f = LittleFS.open(LOG_PATH, "r");
  if (!f) { server.send(200, "text/plain", ""); return; }
  server.streamFile(f, "text/plain");
  f.close();
}

// ---------------- API 处理：WiFi管理 ----------------
void handleWifiStatus() {
  StaticJsonDocument<256> doc;
  doc["apMode"] = apMode;
  doc["connected"] = WiFi.status() == WL_CONNECTED;
  doc["ssid"] = apMode ? String(AP_SSID) : wifiSsid;
  doc["ip"] = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  doc["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  doc["savedSsid"] = wifiSsid; // 即使当前在AP模式，也告诉前端之前保存过哪个SSID
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleWifiScan() {
  int n = WiFi.scanNetworks();
  DynamicJsonDocument doc(2048);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < n && i < 30; i++) {
    JsonObject o = arr.createNestedObject();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
    o["secure"] = WiFi.encryptionType(i) != ENC_TYPE_NONE;
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
  WiFi.scanDelete();
}

void handleWifiConfig() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
    server.send(400, "text/plain", "bad json"); return;
  }
  String ssid = doc["ssid"] | "";
  String pass = doc["password"] | "";
  if (ssid.length() == 0) { server.send(400, "text/plain", "ssid empty"); return; }
  saveWifiConfig(ssid, pass);
  appendLog("WiFi配置已更新，准备重启连接: " + ssid);
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"配置已保存，设备将在1秒后重启并尝试连接\"}");
  delay(1000); // 确保响应先发送给浏览器
  ESP.restart();
}

void handleNotFound() {
  String path = server.uri();
  if (path == "/") path = "/index.html";
  String contentType = "text/plain";
  if (path.endsWith(".html")) contentType = "text/html";
  else if (path.endsWith(".css")) contentType = "text/css";
  else if (path.endsWith(".js")) contentType = "application/javascript";
  if (LittleFS.exists(path)) {
    File f = LittleFS.open(path, "r");
    server.streamFile(f, contentType);
    f.close();
  } else {
    server.send(404, "text/plain", "Not found");
  }
}

// ---------------- 定时任务检查 ----------------
void checkScheduledTasks() {
  if (!ntpSynced) return; // 没有可靠时间，不做定时判断，避免用错误时间误触发
  time_t rawtime = timeClient.getEpochTime();
  struct tm* ti = localtime(&rawtime);
  int minuteOfDay = ti->tm_hour * 60 + ti->tm_min;
  if (minuteOfDay == lastTriggeredMinuteOfDay) return; // 本分钟已检查过
  lastTriggeredMinuteOfDay = minuteOfDay;

  for (int i = 0; i < taskCount; i++) {
    Task& t = tasks[i];
    if (!t.enabled) continue;
    if (!t.days[ti->tm_wday]) continue;
    if (t.hour == ti->tm_hour && t.minute == ti->tm_min) {
      startPump((unsigned long)t.durationSeconds * 1000UL, SRC_SCHEDULE);
      appendLog("定时任务触发: " + t.id);
    }
  }
}

// ---------------- 湿度自动浇水 ----------------
void checkAutoWater() {
  if (!config.autoModeEnabled) return;
  if (pumpState) return; // 正在浇水（无论来源）不重复触发

  if (everAutoWatered) { // 只有触发过至少一次后才检查冷却时间，避免开机瞬间被冷却卡住
    unsigned long cooldownMs = (unsigned long)config.autoCooldownMinutes * 60000UL;
    if (millis() - lastAutoWaterMillis < cooldownMs) return;
  }

  int moisture = readMoisturePercent();
  if (moisture < config.moistureThresholdOn) {
    startPump((unsigned long)config.autoWaterSeconds * 1000UL, SRC_AUTO);
    lastAutoWaterMillis = millis();
    everAutoWatered = true;
    appendLog("自动浇水触发, 湿度=" + String(moisture) + "%");
  }
}

// ---------------- 水泵停止判断（超时兜底 + 自动模式下达到停止阈值提前结束）----------------
void checkPumpTimeout() {
  if (!pumpState) return;
  bool timeUp = pumpDurationMs > 0 && (millis() - pumpStartMillis >= pumpDurationMs);
  bool reachedOff = false;
  if (pumpSource == SRC_AUTO) {
    reachedOff = readMoisturePercent() >= config.moistureThresholdOff;
  }
  if (timeUp) {
    stopPump("浇水结束(达到时长上限)");
  } else if (reachedOff) {
    stopPump("浇水结束(湿度已达到停止阈值)");
  }
}

// ---------------- 系统信息打印 ----------------
void printSystemInfo() {
  Serial.println();
  Serial.println(F("======== 系统信息 ========"));

  Serial.println(F("-- WiFi --"));
  Serial.print(F("模式: ")); Serial.println(apMode ? F("配置热点(AP)模式") : F("正常连接(STA)模式"));
  if (apMode) {
    Serial.print(F("热点名称: ")); Serial.println(AP_SSID);
    Serial.print(F("热点密码: ")); Serial.println(AP_PASSWORD);
    Serial.print(F("配置地址: http://")); Serial.println(WiFi.softAPIP());
    if (wifiSsid.length() > 0) {
      Serial.print(F("之前保存的WiFi: ")); Serial.println(wifiSsid);
      Serial.println(F("(设备会每30秒在后台自动重试一次)"));
    } else {
      Serial.println(F("尚未保存过任何WiFi，请通过配置地址设置"));
    }
  } else {
    Serial.print(F("SSID: ")); Serial.println(wifiSsid);
    Serial.print(F("IP地址: ")); Serial.println(WiFi.localIP());
    Serial.print(F("MAC地址: ")); Serial.println(WiFi.macAddress());
    Serial.print(F("当前连接AP(BSSID): ")); Serial.println(WiFi.BSSIDstr());
    Serial.print(F("信道: ")); Serial.println(WiFi.channel());
    Serial.print(F("信号强度: ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));
    Serial.print(F("网关: ")); Serial.println(WiFi.gatewayIP());
    Serial.print(F("漫游触发阈值: 弱于")); Serial.print(RSSI_ROAM_CHECK_THRESHOLD);
    Serial.print(F("dBm时评估，需强"));Serial.print(RSSI_ROAM_HYSTERESIS);Serial.println(F("dB以上才切换"));
  }

  Serial.println(F("-- 时间(NTP) --"));
  Serial.print(F("同步状态: ")); Serial.println(ntpSynced ? F("已同步") : F("未同步"));
  if (ntpSynced) { Serial.print(F("当前时间: ")); Serial.println(timeClient.getFormattedTime()); }

  Serial.println(F("-- 文件系统(LittleFS) --"));
  FSInfo fsInfo;
  if (LittleFS.info(fsInfo)) {
    Serial.print(F("总空间: ")); Serial.print(fsInfo.totalBytes); Serial.println(F(" bytes"));
    Serial.print(F("已用空间: ")); Serial.print(fsInfo.usedBytes); Serial.println(F(" bytes"));
    Serial.print(F("index.html存在: ")); Serial.println(LittleFS.exists("/index.html") ? F("是") : F("否(网页文件未上传!)"));
  } else {
    Serial.println(F("LittleFS未挂载成功!"));
  }

  Serial.println(F("-- 引脚配置 --"));
  Serial.println(F("湿度传感器AOUT -> A0"));
  Serial.print(F("继电器IN -> GPIO: ")); Serial.println(PIN_RELAY);
  Serial.print(F("状态LED -> GPIO: ")); Serial.println(PIN_STATUS_LED);
  Serial.print(F("继电器触发方式: ")); Serial.println(RELAY_ACTIVE_LOW ? F("低电平触发") : F("高电平触发"));
  Serial.print(F("恢复出厂设置按键 -> GPIO: ")); Serial.print(PIN_FACTORY_RESET);
  Serial.println(F(" (接地持续10秒触发，会清空WiFi/浇水配置/定时任务/日志)"));

  Serial.println(F("-- 浇水配置 --"));
  Serial.print(F("自动浇水: ")); Serial.println(config.autoModeEnabled ? F("已启用") : F("已禁用"));
  Serial.print(F("低于阈值(开始): ")); Serial.print(config.moistureThresholdOn); Serial.println(F("%"));
  Serial.print(F("达到阈值(提前停止): ")); Serial.print(config.moistureThresholdOff); Serial.println(F("%"));
  Serial.print(F("单次浇水最长时长: ")); Serial.print(config.autoWaterSeconds); Serial.println(F("秒"));
  Serial.print(F("冷却间隔: ")); Serial.print(config.autoCooldownMinutes); Serial.println(F("分钟"));
  Serial.print(F("干燥基准ADC: ")); Serial.println(config.adcDry);
  Serial.print(F("湿润基准ADC: ")); Serial.println(config.adcWet);
  Serial.print(F("当前原始ADC读数: ")); Serial.println(analogRead(PIN_MOISTURE));
  Serial.print(F("当前湿度: ")); Serial.print(readMoisturePercent()); Serial.println(F("%"));

  Serial.println(F("-- 日志上送 --"));
  if (logServer.enabled && logServer.host.length() > 0) {
    Serial.print(F("状态: 已启用  协议: ")); Serial.println(logServer.type);
    Serial.print(F("服务器: ")); Serial.print(logServer.host); Serial.print(F(":")); Serial.println(logServer.port);
    if (logServer.type == "http") { Serial.print(F("路径: ")); Serial.println(logServer.path); }
  } else {
    Serial.println(F("状态: 未启用"));
  }

  Serial.println(F("-- 定时任务 --"));
  Serial.print(F("任务数量: ")); Serial.println(taskCount);
  for (int i = 0; i < taskCount; i++) {
    Serial.print(F("  [")); Serial.print(i); Serial.print(F("] "));
    Serial.print(tasks[i].enabled ? F("启用") : F("禁用"));
    Serial.print(F(" 时间:")); Serial.print(tasks[i].hour); Serial.print(F(":")); Serial.print(tasks[i].minute);
    Serial.print(F(" 时长:")); Serial.print(tasks[i].durationSeconds); Serial.println(F("秒"));
  }

  Serial.println(F("=========================="));
  Serial.println();
}

// ---------------- setup / loop ----------------
void registerRoutes() {
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/raw", HTTP_GET, handleRawSample);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/tasks", HTTP_GET, handleGetTasks);
  server.on("/api/tasks", HTTP_POST, handlePostTask);
  server.on(UriBraces("/api/tasks/{}"), HTTP_DELETE, handleDeleteTask);
  server.on("/api/pump", HTTP_POST, handleManualPump);
  server.on("/api/logs", HTTP_GET, handleGetLogs);
  server.on("/api/wifi/status", HTTP_GET, handleWifiStatus);
  server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/api/wifi/config", HTTP_POST, handleWifiConfig);
  server.on("/api/logserver", HTTP_GET, handleGetLogServer);
  server.on("/api/logserver", HTTP_POST, handlePostLogServer);
  server.on("/api/logserver/test", HTTP_POST, handleTestLogServer);
  server.onNotFound(handleNotFound);
  server.begin();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_FACTORY_RESET, INPUT_PULLUP); // 按键一端接此引脚，另一端接GND，短接即触发
  setPumpRaw(false); // 上电先确保泵关闭

  if (!LittleFS.begin()) {
    Serial.println(F("LittleFS挂载失败！请检查Flash Size分区设置"));
  }
  loadConfig();
  loadTasks();
  loadWifiConfig();
  loadLogServerConfig();

  WiFi.mode(WIFI_STA);
  if (wifiSsid.length() > 0 && tryConnectWifi(wifiSsid, wifiPass, WIFI_CONNECT_TIMEOUT_MS)) {
    apMode = false;
    digitalWrite(PIN_STATUS_LED, LOW); // 板载LED低电平点亮，表示连接成功
  } else {
    startApMode();
  }

  if (!apMode) {
    timeClient.begin();
    Serial.println(F("正在同步NTP时间..."));
    int ntpRetry = 0;
    bool ntpOk = false;
    while (ntpRetry < 10) {
      if (timeClient.forceUpdate()) { ntpOk = true; break; }
      delay(500);
      ntpRetry++;
    }
    ntpSynced = ntpOk;
    if (ntpOk) {
      Serial.print(F("NTP同步成功, 当前时间: "));
      Serial.println(timeClient.getFormattedTime());
    } else {
      Serial.println(F("NTP同步失败！设备时间将不准确，定时任务会暂停直到同步成功。"));
    }
  } else {
    Serial.println(F("当前为配置热点模式，跳过NTP同步（定时任务暂不生效，浇水控制和自动湿度检测不受影响）"));
  }

  registerRoutes();
  appendLog("系统启动完成");
  printSystemInfo();
}

unsigned long lastNtpResync = 0;     // NTP真正对时的周期（较慢，减少网络请求）
unsigned long lastScheduleCheck = 0; // 定时任务检查周期（较快，避免错过整分钟）
unsigned long lastWifiBgRetry = 0;   // AP模式下后台重试已保存WiFi的周期

void loop() {
  server.handleClient();
  checkFactoryResetButton(); // 恢复出厂设置检测，不分模式始终生效
  flushLogQueue();           // 日志上送(内部自带条件判断，无待发日志时几乎无开销)

  static unsigned long lastMoistureCheck = 0;
  if (millis() - lastMoistureCheck > 5000) {
    lastMoistureCheck = millis();
    checkAutoWater();
  }

  checkPumpTimeout();

  if (!apMode) {
    // 正常联网模式：定期真正对时(30分钟一次，减少网络请求)，
    // 但每15秒单独检查一次定时任务，避免因为对时间隔太长而错过整分钟触发点
    if (millis() - lastNtpResync > 30UL * 60UL * 1000UL) {
      lastNtpResync = millis();
      if (timeClient.update()) ntpSynced = true;
    }
    if (millis() - lastScheduleCheck > 15000) {
      lastScheduleCheck = millis();
      checkScheduledTasks();
    }

    // 断网检测：指数退避重连，避免弱信号环境下无脑高频重试加重信道拥堵
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(PIN_STATUS_LED, millis() % 1000 < 500 ? HIGH : LOW); // 闪烁提示掉线
      if (millis() >= nextReconnectAllowedAt) {
        reconnectAttempts++;
        unsigned long backoff = RECONNECT_BACKOFF_BASE_MS * (1UL << min(reconnectAttempts, 6));
        if (backoff > RECONNECT_BACKOFF_MAX_MS) backoff = RECONNECT_BACKOFF_MAX_MS;
        nextReconnectAllowedAt = millis() + backoff;
        Serial.print(F("WiFi已断开，尝试重连(第"));
        Serial.print(reconnectAttempts);
        Serial.print(F("次)，下次重试间隔"));
        Serial.print(backoff / 1000);
        Serial.println(F("秒"));
        WiFi.reconnect();
      }
    } else {
      if (reconnectAttempts > 0) {
        appendLog("WiFi重连成功(共重试" + String(reconnectAttempts) + "次)");
        reconnectAttempts = 0; // 连接恢复后清零退避计数
      }
      digitalWrite(PIN_STATUS_LED, LOW);

      // 信号质量冗余：周期性评估是否有更强的同名AP可以主动切换过去
      if (millis() - lastRoamCheckMillis > ROAM_CHECK_INTERVAL_MS) {
        lastRoamCheckMillis = millis();
        evaluateRoaming();
      }
    }
  } else {
    // 配置热点模式：状态LED持续闪烁提示"需要配置"
    digitalWrite(PIN_STATUS_LED, millis() % 500 < 250 ? HIGH : LOW);

    // 如果之前保存过WiFi(只是暂时连不上)，每隔一段时间在后台悄悄重试，
    // 一旦成功就自动退出AP模式，不需要用户手动干预
    if (wifiSsid.length() > 0 && millis() - lastWifiBgRetry > WIFI_BG_RETRY_INTERVAL_MS) {
      lastWifiBgRetry = millis();
      Serial.println(F("[AP模式] 后台尝试重连已保存WiFi..."));
      if (tryConnectWifi(wifiSsid, wifiPass, 8000)) {
        Serial.println(F("后台重连成功，退出配置热点模式"));
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);
        apMode = false;
        digitalWrite(PIN_STATUS_LED, LOW);
        timeClient.begin();
        if (timeClient.forceUpdate()) ntpSynced = true;
        appendLog("后台自动重连WiFi成功");
      }
    }
  }
}
