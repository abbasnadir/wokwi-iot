#include <PubSubClient.h>
#include <TM1637Display.h>
#include <WiFi.h>
#include <cstdio>
#include <cstring>

namespace
{

  constexpr int N_GREEN = 1;
  constexpr int N_RED = 2;
  constexpr int E_GREEN = 3;
  constexpr int E_RED = 6;
  constexpr int S_GREEN = 7;
  constexpr int S_RED = 10;
  constexpr int W_GREEN = 11;
  constexpr int W_RED = 18;

  constexpr int IDX_N = 0;
  constexpr int IDX_E = 1;
  constexpr int IDX_S = 2;
  constexpr int IDX_W = 3;

  constexpr unsigned long NORMAL_PHASE_MS = 10000;
  constexpr unsigned long WARNING_MS = 5000;
  constexpr unsigned long COOLDOWN_MS = 5000;
  constexpr unsigned long RECORD_STALE_MS = 20000;
  constexpr unsigned long SYNC_INTERVAL_MS = 500;
  constexpr unsigned long EXPIRE_INTERVAL_MS = 2000;

  // ↓ Key reductions to fit in Wokwi ESP32-C6 heap
  constexpr int MAX_QUEUE = 4;          // was 8
  constexpr int CODE_BUFFER_SIZE = 12;  // was 20, "AMB-0001" = 8 chars
  constexpr int MQTT_BUFFER_SIZE = 256; // was 512

  constexpr int APPROACH_PRIORITY_DISTANCE_M = 300;
  constexpr int DEPARTURE_PRIORITY_DISTANCE_M = 100;

  const char *WIFI_SSID = "Wokwi-GUEST";
  const char *WIFI_PASSWORD = "";
  const char *MQTT_BROKER = "broker.emqx.io";
  const int MQTT_PORT = 1883;
  const char *MQTT_TOPIC = "iot/ambulance-priority/corridor";

  const int GREEN_PINS[4] = {N_GREEN, E_GREEN, S_GREEN, W_GREEN};
  const int RED_PINS[4] = {N_RED, E_RED, S_RED, W_RED};
  const char *DIR_NAMES[4] = {"NORTH", "EAST", "SOUTH", "WEST"};

  // ── record struct: kept minimal ────────────────────────────────────────────
  struct AmbulanceRecord
  {
    bool inUse = false;
    char code[CODE_BUFFER_SIZE] = {};
    int8_t originIdx = -1; // int8_t saves 3 bytes each vs int
    int8_t targetIdx = -1;
    int positionMeters = 0;
    unsigned long firstSeenMs = 0;
    unsigned long lastUpdateMs = 0;
  };

  WiFiClient wifiClient;
  PubSubClient mqttClient(wifiClient);

  TM1637Display dispN(19, 20);
  TM1637Display dispE(21, 22);
  TM1637Display dispS(23, 0);
  TM1637Display dispW(5, 15);
  TM1637Display *displays[4] = {&dispN, &dispE, &dispS, &dispW};

  AmbulanceRecord records[MAX_QUEUE];

  enum State
  {
    NORMAL,
    AMBUL_WARNING,
    AMBUL_ACTIVE,
    AMBUL_COOLDOWN
  };

  State state = NORMAL;
  int normalPhase = IDX_N;
  int targetDirection = IDX_N;
  int latestDistanceMeters = 999;
  unsigned long phaseStartMs = 0;
  unsigned long stateStartMs = 0;
  unsigned long lastSyncMs = 0;
  unsigned long lastExpireMs = 0;
  bool ambulanceNear = false;
  bool queueHoldForCooldown = false;
  char activeCode[CODE_BUFFER_SIZE] = {};
  int activeDirection = IDX_N;

  // ── lights & displays ──────────────────────────────────────────────────────

  void setOnlyGreen(int idx)
  {
    for (int i = 0; i < 4; ++i)
    {
      digitalWrite(GREEN_PINS[i], i == idx ? HIGH : LOW);
      digitalWrite(RED_PINS[i], i == idx ? LOW : HIGH);
    }
  }

  void allRed()
  {
    for (int i = 0; i < 4; ++i)
    {
      digitalWrite(GREEN_PINS[i], LOW);
      digitalWrite(RED_PINS[i], HIGH);
    }
  }

  void showDashes(TM1637Display *d)
  {
    uint8_t seg[] = {SEG_G, SEG_G, SEG_G, SEG_G};
    d->setSegments(seg);
  }

  void showSeconds(TM1637Display *d, int s)
  {
    if (s < 0)
      s = 0;
    d->showNumberDec(s, false);
  }

  void updateDisplays()
  {
    const unsigned long now = millis();
    const unsigned long elapsed = now - phaseStartMs;

    switch (state)
    {
    case NORMAL:
    {
      int phaseLeft = 0;
      if (elapsed < NORMAL_PHASE_MS)
        phaseLeft = (int)((NORMAL_PHASE_MS - elapsed) / 1000);
      for (int i = 0; i < 4; ++i)
      {
        if (i == normalPhase)
        {
          showSeconds(displays[i], phaseLeft);
        }
        else
        {
          int phasesAway = (i - normalPhase + 4) % 4;
          int waitSecs = phaseLeft + (phasesAway - 1) * (int)(NORMAL_PHASE_MS / 1000);
          showSeconds(displays[i], waitSecs);
        }
      }
      break;
    }
    case AMBUL_WARNING:
    {
      int s = (int)((WARNING_MS - (now - stateStartMs)) / 1000);
      for (int i = 0; i < 4; ++i)
        i == targetDirection ? showSeconds(displays[i], s) : showDashes(displays[i]);
      break;
    }
    case AMBUL_ACTIVE:
      for (int i = 0; i < 4; ++i)
        i == targetDirection ? showSeconds(displays[i], 0) : showDashes(displays[i]);
      break;
    case AMBUL_COOLDOWN:
    {
      int s = (int)((COOLDOWN_MS - (now - stateStartMs)) / 1000);
      for (int i = 0; i < 4; ++i)
        i == targetDirection ? showSeconds(displays[i], s) : showDashes(displays[i]);
      break;
    }
    }
  }

  // ── JSON helpers ───────────────────────────────────────────────────────────

  int directionToIndex(const char *d)
  {
    if (strcasecmp(d, "north") == 0)
      return IDX_N;
    if (strcasecmp(d, "east") == 0)
      return IDX_E;
    if (strcasecmp(d, "south") == 0)
      return IDX_S;
    if (strcasecmp(d, "west") == 0)
      return IDX_W;
    return -1;
  }

  bool extractJsonString(const char *json, const char *key,
                         char *out, size_t outSize)
  {
    char pattern[24];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *start = strstr(json, pattern);
    if (!start)
      return false;
    start += strlen(pattern);
    const char *end = strchr(start, '"');
    if (!end)
      return false;
    size_t len = (size_t)(end - start);
    if (len >= outSize)
      return false;
    memcpy(out, start, len);
    out[len] = '\0';
    return true;
  }

  bool extractJsonInt(const char *json, const char *key, int *out)
  {
    char pattern[24];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *start = strstr(json, pattern);
    if (!start)
      return false;
    start += strlen(pattern);
    while (*start == ' ')
      ++start;
    return sscanf(start, "%d", out) == 1;
  }

  // ── networking ─────────────────────────────────────────────────────────────

  void connectToWifi()
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected. Traffic controller IP: ");
    Serial.println(WiFi.localIP());
  }

  // ── debug ──────────────────────────────────────────────────────────────────

  const char *stateName(State v)
  {
    switch (v)
    {
    case NORMAL:
      return "NORMAL";
    case AMBUL_WARNING:
      return "AMBUL_WARNING";
    case AMBUL_ACTIVE:
      return "AMBUL_ACTIVE";
    case AMBUL_COOLDOWN:
      return "AMBUL_COOLDOWN";
    }
    return "UNKNOWN";
  }

  void printSummary(const char *reason)
  {
    int q = 0;
    for (int i = 0; i < MAX_QUEUE; ++i)
      if (records[i].inUse)
        ++q;

    // Single snprintf into stack buffer — no heap use
    char buf[96];
    snprintf(buf, sizeof(buf), "[%s] state=%s, active=%s, dir=%s, dist=%dm, queue=%d",
             reason, stateName(state),
             activeCode[0] ? activeCode : "none",
             DIR_NAMES[targetDirection],
             latestDistanceMeters, q);
    Serial.println(buf);
  }

  // ── record management ──────────────────────────────────────────────────────

  int findRecordByCode(const char *code)
  {
    for (int i = 0; i < MAX_QUEUE; ++i)
      if (records[i].inUse && strcmp(records[i].code, code) == 0)
        return i;
    return -1;
  }

  int findFreeSlot()
  {
    for (int i = 0; i < MAX_QUEUE; ++i)
      if (!records[i].inUse)
        return i;
    return -1;
  }

  void removeRecord(int i)
  {
    if (i >= 0 && i < MAX_QUEUE)
      records[i] = AmbulanceRecord();
  }

  bool isApproaching(const AmbulanceRecord &r)
  {
    if (r.originIdx == IDX_N && r.targetIdx == IDX_S)
      return r.positionMeters <= 0;
    if (r.originIdx == IDX_S && r.targetIdx == IDX_N)
      return r.positionMeters >= 0;
    return true;
  }

  bool inPriorityWindow(const AmbulanceRecord &r)
  {
    int d = abs(r.positionMeters);
    return d <= (isApproaching(r) ? APPROACH_PRIORITY_DISTANCE_M
                                  : DEPARTURE_PRIORITY_DISTANCE_M);
  }

  int findHead()
  {
    int head = -1;
    for (int i = 0; i < MAX_QUEUE; ++i)
    {
      if (!records[i].inUse || !inPriorityWindow(records[i]))
        continue;
      if (head < 0 || records[i].firstSeenMs < records[head].firstSeenMs)
        head = i;
    }
    return head;
  }

  void syncActiveAmbulance()
  {
    if (queueHoldForCooldown)
    {
      ambulanceNear = false;
      latestDistanceMeters = 999;
      return;
    }
    if (activeCode[0])
    {
      int idx = findRecordByCode(activeCode);
      if (idx >= 0 && inPriorityWindow(records[idx]))
      {
        ambulanceNear = true;
        latestDistanceMeters = abs(records[idx].positionMeters);
        targetDirection = activeDirection;
        return;
      }
      activeCode[0] = '\0';
    }
    int head = findHead();
    if (head < 0)
    {
      ambulanceNear = false;
      latestDistanceMeters = 999;
      targetDirection = IDX_N;
      return;
    }
    strncpy(activeCode, records[head].code, CODE_BUFFER_SIZE - 1);
    activeCode[CODE_BUFFER_SIZE - 1] = '\0';
    activeDirection = records[head].originIdx;
    targetDirection = activeDirection;
    latestDistanceMeters = abs(records[head].positionMeters);
    ambulanceNear = true;
  }

  void upsertRecord(const char *code, int originIdx, int targetIdx, int pos)
  {
    int idx = findRecordByCode(code);
    if (idx < 0)
    {
      idx = findFreeSlot();
      if (idx < 0)
        return;
      records[idx].inUse = true;
      strncpy(records[idx].code, code, CODE_BUFFER_SIZE - 1);
      records[idx].code[CODE_BUFFER_SIZE - 1] = '\0';
      records[idx].originIdx = (int8_t)originIdx;
      records[idx].targetIdx = (int8_t)targetIdx;
      records[idx].firstSeenMs = millis();

      // Minimal log — avoid long Serial.print chains
      char buf[48];
      snprintf(buf, sizeof(buf), "Enqueued %s from %s to %s",
               code, DIR_NAMES[originIdx], DIR_NAMES[targetIdx]);
      Serial.println(buf);
    }
    records[idx].positionMeters = pos;
    records[idx].lastUpdateMs = millis();
  }

  void completeRecord(const char *code)
  {
    int idx = findRecordByCode(code);
    if (idx < 0)
      return;
    if (strcmp(code, activeCode) == 0)
    {
      queueHoldForCooldown = true;
      activeCode[0] = '\0';
    }
    removeRecord(idx);
  }

  void expireStaleRecords()
  {
    unsigned long now = millis();
    if (now - lastExpireMs < EXPIRE_INTERVAL_MS)
      return;
    lastExpireMs = now;
    for (int i = 0; i < MAX_QUEUE; ++i)
    {
      if (records[i].inUse && now - records[i].lastUpdateMs > RECORD_STALE_MS)
      {
        Serial.print("Expiring: ");
        Serial.println(records[i].code);
        removeRecord(i);
      }
    }
  }

  // ── MQTT ───────────────────────────────────────────────────────────────────

  void mqttCallback(char *topic, byte *payload, unsigned int length)
  {
    if (length == 0 || length >= MQTT_BUFFER_SIZE)
      return;

    // Stack-allocated — no heap use at all
    char message[MQTT_BUFFER_SIZE];
    memcpy(message, payload, length);
    message[length] = '\0';

    // Only print code+event, not the full payload — saves Serial heap pressure
    char code[CODE_BUFFER_SIZE];
    char eventName[16];
    char origin[8];
    char target_str[8];
    int positionMeters = 0;

    if (!extractJsonString(message, "code", code, sizeof(code)) ||
        !extractJsonString(message, "event", eventName, sizeof(eventName)) ||
        !extractJsonString(message, "origin", origin, sizeof(origin)) ||
        !extractJsonString(message, "target", target_str, sizeof(target_str)) ||
        !extractJsonInt(message, "position_m", &positionMeters))
    {
      Serial.println("Bad MQTT payload");
      return;
    }

    int originIdx = directionToIndex(origin);
    int targetIdx = directionToIndex(target_str);
    if (originIdx < 0 || targetIdx < 0)
      return;

    if (strcmp(eventName, "completed") == 0)
    {
      completeRecord(code);
    }
    else
    {
      upsertRecord(code, originIdx, targetIdx, positionMeters);
    }

    syncActiveAmbulance();
    printSummary("MQTT");
  }

  void ensureMqttConnection()
  {
    while (!mqttClient.connected())
    {
      Serial.print("Connecting to MQTT broker...");
      char clientId[40];
      snprintf(clientId, sizeof(clientId), "proj1-ctrl-%08lx",
               (unsigned long)ESP.getEfuseMac());
      if (mqttClient.connect(clientId))
      {
        Serial.println("connected");
        mqttClient.subscribe(MQTT_TOPIC);
        Serial.print("Subscribed to topic: ");
        Serial.println(MQTT_TOPIC);
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" retrying in 2s");
        delay(2000);
      }
    }
  }

  // ── state machine ──────────────────────────────────────────────────────────

  void handleStateMachine()
  {
    const unsigned long now = millis();
    switch (state)
    {
    case NORMAL:
      if (now - phaseStartMs >= NORMAL_PHASE_MS)
      {
        normalPhase = (normalPhase + 1) % 4;
        phaseStartMs = now;
        setOnlyGreen(normalPhase);
        Serial.print("[NORMAL] ");
        Serial.print(DIR_NAMES[normalPhase]);
        Serial.println(" GREEN");
      }
      if (ambulanceNear)
      {
        allRed();
        state = AMBUL_WARNING;
        stateStartMs = now;
        printSummary("WARNING");
      }
      break;

    case AMBUL_WARNING:
      if (!ambulanceNear)
      {
        state = NORMAL;
        phaseStartMs = now;
        setOnlyGreen(normalPhase);
        break;
      }
      if (now - stateStartMs >= WARNING_MS)
      {
        setOnlyGreen(targetDirection);
        state = AMBUL_ACTIVE;
        stateStartMs = now;
        printSummary("ACTIVE");
      }
      break;

    case AMBUL_ACTIVE:
      if (queueHoldForCooldown || !ambulanceNear)
      {
        allRed();
        state = AMBUL_COOLDOWN;
        stateStartMs = now;
        queueHoldForCooldown = false;
        printSummary("COOLDOWN");
      }
      break;

    case AMBUL_COOLDOWN:
      if (ambulanceNear)
      {
        allRed();
        state = AMBUL_WARNING;
        stateStartMs = now;
        printSummary("WARNING");
        break;
      }
      if (now - stateStartMs >= COOLDOWN_MS)
      {
        normalPhase = (targetDirection + 1) % 4;
        phaseStartMs = now;
        setOnlyGreen(normalPhase);
        state = NORMAL;
      }
      break;
    }
  }

} // namespace

// ── Arduino entry points ───────────────────────────────────────────────────

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < 4; ++i)
  {
    pinMode(GREEN_PINS[i], OUTPUT);
    pinMode(RED_PINS[i], OUTPUT);
  }
  for (int i = 0; i < 4; ++i)
  {
    displays[i]->setBrightness(5);
    showSeconds(displays[i], 0);
  }

  allRed();
  delay(500);

  connectToWifi();

  // setBufferSize BEFORE setServer — mandatory
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  phaseStartMs = millis();
  lastExpireMs = millis();
  lastSyncMs = millis();
  setOnlyGreen(IDX_N);
  state = NORMAL;

  Serial.println("=== 4-Way Intersection — Corridor Queue Mode ===");
  Serial.print("Listening on topic ");
  Serial.println(MQTT_TOPIC);
  Serial.println("Displays restored to signal countdown mode.");
  Serial.println("Priority window: 300m while approaching, 100m after passing.");
  printSummary("START");
}

void loop()
{
  ensureMqttConnection();
  mqttClient.loop();
  expireStaleRecords();

  const unsigned long now = millis();
  if (now - lastSyncMs >= SYNC_INTERVAL_MS)
  {
    lastSyncMs = now;
    syncActiveAmbulance();
  }

  handleStateMachine();
  updateDisplays();
  delay(50);
}