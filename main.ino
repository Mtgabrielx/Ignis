#include <DHT.h>
#include <math.h>
#include <WiFi.h>
#include <DHT_U.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>

#define DHTTYPE    DHT11
#define DHTPIN     32
#define Dsm501aPIN 36

// -------------------- DSM501A / Fumaça --------------------
static unsigned long duration;
static unsigned long starttime;
static unsigned long sampletime_ms     = 5000;   // 5 segundos
static unsigned long lowpulseoccupancy = 0;
static float ratio = 0.0f;

// -------------------- WiFi / MQTT --------------------
const char* ssid     = "";
const char* password = "";

const char* mqtt_server = "";
const int   mqtt_port   = 1883;

const char* mqtt_user   = "";
const char* mqtt_pass   = "";

// tópicos principais
const char* topic_temp   = "sensor.temperatura";
const char* topic_umid   = "sensor.umidade";
const char* topic_fuma   = "sensor.fumaca";

// tópicos de alerta
const char* topic_alerta_temp = "sensor.alertaTemp";
const char* topic_alerta_fuma = "sensor.alertaFuma";

// =====================================================
// DETECÇÃO ROBUSTA: Mediana + MAD + Confirmação + Cooldown
// =====================================================
#define K_TEMP  4.0f
#define K_FUMA  4.5f

#define CONFIRM_M 3
#define CONFIRM_N 2

#define BASELINE_MIN 6

#define COOLDOWN_TEMP_MS 30000UL
#define COOLDOWN_FUMA_MS 30000UL

// =====================================================
// Objetos
// =====================================================
WiFiClient espClient;
PubSubClient client(espClient);
DHT_Unified dht(DHTPIN, DHTTYPE);

// =====================================================
// Estado do detector (SEM struct): buffers + índices + flags
// =====================================================

// Temperatura
float tempBuf[10];
int   tempIdx = 0;
int   tempCount = 0;
int   tempFlags[CONFIRM_M] = {0};
int   tempFidx = 0;
unsigned long lastAlertTempMs = 0;

// Fumaça
float fumaBuf[10];
int   fumaIdx = 0;
int   fumaCount = 0;
int   fumaFlags[CONFIRM_M] = {0};
int   fumaFidx = 0;
unsigned long lastAlertFumaMs = 0;

// =====================================================
// Utilitários (ordenar, mediana, MAD)
// =====================================================
void sortArray(float *a, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (a[j] < a[i]) {
        float t = a[i]; a[i] = a[j]; a[j] = t;
      }
    }
  }
}

float medianOf(const float *arr, int n) {
  if (n <= 0) return 0.0f;
  float tmp[10];
  for (int i = 0; i < n; i++) tmp[i] = arr[i];
  sortArray(tmp, n);
  if (n % 2 == 1) return tmp[n / 2];
  return (tmp[n / 2 - 1] + tmp[n / 2]) / 2.0f;
}

float madOf(const float *arr, int n) {
  if (n <= 1) return 0.0f;
  float med = medianOf(arr, n);
  float dev[10];
  for (int i = 0; i < n; i++) dev[i] = fabs(arr[i] - med);
  return medianOf(dev, n);
}

// =====================================================
// Operações básicas do “detector” (SEM struct)
// =====================================================
void pushValue(float *buf, int &idx, int &count, float v) {
  buf[idx] = v;
  idx = (idx + 1) % 10;
  if (count < 10) count++;
}

int pushFlag(int *flags, int &fidx, bool isAnom) {
  flags[fidx] = isAnom ? 1 : 0;
  fidx = (fidx + 1) % CONFIRM_M;

  int s = 0;
  for (int i = 0; i < CONFIRM_M; i++) s += flags[i];
  return s;
}

void clearFlags(int *flags, int &fidx) {
  for (int i = 0; i < CONFIRM_M; i++) flags[i] = 0;
  fidx = 0;
}

bool isAnomaly(const float *buf, int count, float v, float K) {
  if (count < BASELINE_MIN) return false;

  float med = medianOf(buf, count);
  float mad = madOf(buf, count);

  const float eps = 1e-6f;
  float score = fabs(v - med) / (mad + eps);
  return (score > K);
}

// =====================================================
// processAlert genérico (SEM struct) retorna 0/1
// - valor que gera alerta ENTRA no buffer
// =====================================================
int processAlertGeneric(float value,
                        float *buf, int &idx, int &count,
                        int *flags, int &fidx,
                        float K,
                        unsigned long cooldownMs,
                        unsigned long &lastAlertMs) {

  // baseline fraca: só alimenta o buffer
  if (count < BASELINE_MIN) {
    pushValue(buf, idx, count, value);
    return 0;
  }

  bool anom = isAnomaly(buf, count, value, K);
  int anomCount = pushFlag(flags, fidx, anom);

  bool confirmed = (anomCount >= CONFIRM_N);

  if (!confirmed) {
    pushValue(buf, idx, count, value);
    return 0;
  }

  unsigned long now = millis();
  if (now - lastAlertMs < cooldownMs) {
    // em cooldown: não dispara alerta, mas salva no baseline
    pushValue(buf, idx, count, value);
    clearFlags(flags, fidx);
    return 0;
  }

  // alerta válido
  lastAlertMs = now;
  clearFlags(flags, fidx);

  // ajuste solicitado: salva o valor que gerou alerta
  pushValue(buf, idx, count, value);

  return 1;
}

// =====================================================
// WiFi / MQTT
// =====================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnectMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    String clientId = "esp32-client-";
    clientId += String(random(0xffff), HEX);
    if (!client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) delay(2000);
  }
}

void disconnectWIFI() {
  if (client.connected()) client.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

// =====================================================
// Leitura sensores
// =====================================================
float ler_temperatura() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) return -1;
  return event.temperature;
}

float ler_umidade() {
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) return -1;
  return event.relative_humidity;
}

float ler_fumaca() {
  lowpulseoccupancy = 0;
  starttime = millis();

  while ((millis() - starttime) < sampletime_ms) {
    duration = pulseIn(Dsm501aPIN, LOW, sampletime_ms * 1000UL);
    lowpulseoccupancy += duration;
  }

  ratio = lowpulseoccupancy / (sampletime_ms * 10.0f);
  return ratio;
}

// =====================================================
// setup / loop
// =====================================================
void setup() {
  Serial.begin(115200);
  pinMode(Dsm501aPIN, INPUT);
  client.setServer(mqtt_server, mqtt_port);
  dht.begin();

  for (int i = 0; i < 10; i++) {
    tempBuf[i] = 0.0f;
    fumaBuf[i] = 0.0f;
  }
}

void loop() {
  char buffer[32];

  float umidade     = ler_umidade();
  float temperatura = ler_temperatura();
  float fumaca      = ler_fumaca();


  int alertaTemp = 0;
  int alertaFuma = 0;

  if (temperatura >= 0) {
    alertaTemp = processAlertGeneric(
      temperatura,
      tempBuf, tempIdx, tempCount,
      tempFlags, tempFidx,
      K_TEMP,
      COOLDOWN_TEMP_MS,
      lastAlertTempMs
    );
  }

  if (fumaca >= 0) {
    alertaFuma = processAlertGeneric(
      fumaca,
      fumaBuf, fumaIdx, fumaCount,
      fumaFlags, fumaFidx,
      K_FUMA,
      COOLDOWN_FUMA_MS,
      lastAlertFumaMs
    );
  }

  connectWiFi();
  reconnectMQTT();
  client.loop();
  client.publish(topic_alerta_temp, (alertaTemp ? "1" : "0"), true);
  client.publish(topic_alerta_fuma, (alertaFuma ? "1" : "0"), true);

  // 5) Publica telemetria
  dtostrf(temperatura, 0, 2, buffer);
  client.publish(topic_temp, buffer, true);

  dtostrf(umidade, 0, 2, buffer);
  client.publish(topic_umid, buffer, true);

  dtostrf(fumaca, 0, 2, buffer);
  client.publish(topic_fuma, buffer, true);

  Serial.print("[PUB] temp="); Serial.print(temperatura);
  Serial.print(" umid=");      Serial.print(umidade);
  Serial.print(" fuma=");      Serial.print(fumaca);
  Serial.print(" alertaTemp=");Serial.print(alertaTemp);
  Serial.print(" alertaFuma=");Serial.println(alertaFuma);

  // 6) Desliga Wi-Fi
  disconnectWIFI();

}
