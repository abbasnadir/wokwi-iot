#include <PubSubClient.h>
#include <WiFi.h>

namespace {

constexpr int POT_PIN = 4;
constexpr int RESET_BUTTON_PIN = 8;

constexpr int POSITION_MIN_METERS = -400;
constexpr int POSITION_MAX_METERS = 400;
constexpr int START_ZONE_METERS = 350;
constexpr int MOVE_START_DELTA_METERS = 20;
constexpr int READ_INTERVAL_MS = 100;
constexpr int PUBLISH_INTERVAL_MS = 800;

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const char* MQTT_BROKER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "iot/ambulance-priority/corridor";

enum Side { SIDE_NONE = -1, SIDE_NORTH = 0, SIDE_SOUTH = 1 };

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int currentAdc = 0;
int rawPositionMeters = POSITION_MIN_METERS;
int publishedPositionMeters = POSITION_MIN_METERS;
int previousPublishedPositionMeters = POSITION_MIN_METERS;
int resetReferencePositionMeters = POSITION_MIN_METERS;
bool resetOverrideActive = true;
bool lastButtonPressed = false;

bool tripActive = false;
Side readySide = SIDE_NORTH;
Side originSide = SIDE_NONE;
Side targetSide = SIDE_NONE;
String activeCode;
unsigned long nextAmbulanceCode = 1;
unsigned long lastSensorUpdateMs = 0;
unsigned long lastPublishMs = 0;
unsigned long lastSerialLogMs = 0;
String pendingEvent = "reset";

const char* sideName(Side side) {
  switch (side) {
    case SIDE_NORTH:
      return "north";
    case SIDE_SOUTH:
      return "south";
    case SIDE_NONE:
      return "none";
  }
  return "none";
}

const char* routeName(Side origin, Side target) {
  if (origin == SIDE_NORTH && target == SIDE_SOUTH) return "north_to_south";
  if (origin == SIDE_SOUTH && target == SIDE_NORTH) return "south_to_north";
  return "idle";
}

int readStableAdc() {
  long total = 0;
  for (int i = 0; i < 8; ++i) {
    total += analogRead(POT_PIN);
    delay(2);
  }
  return static_cast<int>(total / 8);
}

String makeCode() {
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "AMB-%04lu", nextAmbulanceCode++);
  return String(buffer);
}

void connectToWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. Node IP: ");
  Serial.println(WiFi.localIP());
}

void ensureMqttConnection() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker...");
    char clientId[40];
    snprintf(clientId, sizeof(clientId), "proj2-corridor-%08lx", (unsigned long)ESP.getEfuseMac());
    if (mqttClient.connect(clientId)) {
      Serial.println("connected");
      Serial.print("Publishing corridor data to topic: ");
      Serial.println(MQTT_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 2s");
      delay(2000);
    }
  }
}

void resetSimulationToNorth() {
  resetOverrideActive = true;
  resetReferencePositionMeters = rawPositionMeters;
  publishedPositionMeters = POSITION_MIN_METERS;
  previousPublishedPositionMeters = POSITION_MIN_METERS;
  readySide = SIDE_NORTH;
  tripActive = false;
  originSide = SIDE_NONE;
  targetSide = SIDE_NONE;
  activeCode = "";
  pendingEvent = "reset";

  Serial.println("Reset button pressed. Published position forced to 400m north of the center.");
}

void startTrip(Side origin, Side target) {
  tripActive = true;
  originSide = origin;
  targetSide = target;
  activeCode = makeCode();
  pendingEvent = "trip_started";

  Serial.print("New ambulance trip -> code=");
  Serial.print(activeCode);
  Serial.print(", route=");
  Serial.println(routeName(originSide, targetSide));
}

void updateReadySide() {
  if (publishedPositionMeters <= -START_ZONE_METERS) {
    readySide = SIDE_NORTH;
  } else if (publishedPositionMeters >= START_ZONE_METERS) {
    readySide = SIDE_SOUTH;
  }
}

void maybeStartTrip() {
  if (tripActive) {
    return;
  }

  if (readySide == SIDE_NORTH &&
      previousPublishedPositionMeters <= -START_ZONE_METERS &&
      publishedPositionMeters > previousPublishedPositionMeters + MOVE_START_DELTA_METERS) {
    startTrip(SIDE_NORTH, SIDE_SOUTH);
    return;
  }

  if (readySide == SIDE_SOUTH &&
      previousPublishedPositionMeters >= START_ZONE_METERS &&
      publishedPositionMeters < previousPublishedPositionMeters - MOVE_START_DELTA_METERS) {
    startTrip(SIDE_SOUTH, SIDE_NORTH);
  }
}

void maybeCompleteTrip() {
  if (!tripActive) {
    return;
  }

  if (targetSide == SIDE_SOUTH && publishedPositionMeters >= START_ZONE_METERS) {
    pendingEvent = "completed";
    readySide = SIDE_SOUTH;
    tripActive = false;
    Serial.print("Ambulance completed route at south node -> ");
    Serial.println(activeCode);
    return;
  }

  if (targetSide == SIDE_NORTH && publishedPositionMeters <= -START_ZONE_METERS) {
    pendingEvent = "completed";
    readySide = SIDE_NORTH;
    tripActive = false;
    Serial.print("Ambulance completed route at north node -> ");
    Serial.println(activeCode);
  }
}

void refreshCorridorPosition(bool forceLog = false) {
  const unsigned long now = millis();
  if (!forceLog && now - lastSensorUpdateMs < READ_INTERVAL_MS) {
    return;
  }

  lastSensorUpdateMs = now;
  currentAdc = readStableAdc();
  rawPositionMeters = map(currentAdc, 0, 4095, POSITION_MIN_METERS, POSITION_MAX_METERS);
  rawPositionMeters = constrain(rawPositionMeters, POSITION_MIN_METERS, POSITION_MAX_METERS);

  const bool buttonPressed = digitalRead(RESET_BUTTON_PIN) == LOW;
  if (buttonPressed && !lastButtonPressed) {
    resetSimulationToNorth();
  }
  lastButtonPressed = buttonPressed;

  if (resetOverrideActive && abs(rawPositionMeters - resetReferencePositionMeters) > MOVE_START_DELTA_METERS) {
    resetOverrideActive = false;
    pendingEvent = "override_released";
    Serial.println("Slider moved after reset. Releasing north-side override.");
  }

  previousPublishedPositionMeters = publishedPositionMeters;
  publishedPositionMeters = resetOverrideActive ? POSITION_MIN_METERS : rawPositionMeters;

  updateReadySide();
  maybeStartTrip();
  maybeCompleteTrip();

  if (forceLog || now - lastSerialLogMs >= 1000) {
    lastSerialLogMs = now;
    Serial.print("Position update -> raw=");
    Serial.print(rawPositionMeters);
    Serial.print("m, published=");
    Serial.print(publishedPositionMeters);
    Serial.print("m, code=");
    Serial.println(activeCode.length() ? activeCode : "none");
  }
}

String buildJsonPayload(const String& eventName) {
  String payload = "{";
  payload += "\"code\":\"";
  payload += activeCode;
  payload += "\",\"event\":\"";
  payload += eventName;
  payload += "\",\"origin\":\"";
  payload += sideName(originSide);
  payload += "\",\"target\":\"";
  payload += sideName(targetSide);
  payload += "\",\"route\":\"";
  payload += routeName(originSide, targetSide);
  payload += "\",\"position_m\":";
  payload += publishedPositionMeters;
  payload += ",\"abs_distance_m\":";
  payload += abs(publishedPositionMeters);
  payload += ",\"adc\":";
  payload += currentAdc;
  payload += ",\"updated_at_ms\":";
  payload += lastSensorUpdateMs;
  payload += "}";
  return payload;
}

void publishTelemetry() {
  const unsigned long now = millis();
  const bool hasPendingEvent = pendingEvent.length() > 0;
  if (!hasPendingEvent && now - lastPublishMs < PUBLISH_INTERVAL_MS) {
    return;
  }

  lastPublishMs = now;
  refreshCorridorPosition();
  ensureMqttConnection();

  const String eventName = hasPendingEvent ? pendingEvent : "update";
  const String payload = buildJsonPayload(eventName);
  mqttClient.publish(MQTT_TOPIC, payload.c_str(), false);

  Serial.print("Published -> ");
  Serial.println(payload);

  if (eventName == "completed") {
    activeCode = "";
    originSide = SIDE_NONE;
    targetSide = SIDE_NONE;
  }
  pendingEvent = "";
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(POT_PIN, INPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  connectToWifi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  refreshCorridorPosition(true);

  Serial.println("=== Ambulance Corridor Publisher ===");
  Serial.println("Slider start  -> 400m north of center");
  Serial.println("Slider middle -> 0m at center");
  Serial.println("Slider full   -> 400m south of center");
  Serial.println("Press reset to force the published position back to the north side.");
}

void loop() {
  refreshCorridorPosition();
  ensureMqttConnection();
  mqttClient.loop();
  publishTelemetry();
  delay(10);
}
