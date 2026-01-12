#include <DHT.h>
#include <math.h>
#include <WiFi.h>
#include <DHT_U.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>

/* =====================================================
   CONFIGURAÇÕES DE HARDWARE
   ===================================================== */
#define DHTTYPE    DHT11
#define DHTPIN     32
#define Dsm501aPIN 36 // DSM501A (saída digital) para LOW pulse occupancy

/* =====================================================
   DSM501A / “Fumaça” (na prática: particulado/poeira -> ratio)
   ===================================================== */
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms     = 5000;   // 5s de janela de amostragem
unsigned long lowpulseoccupancy = 0;
float ratio = 0.0f;

/* =====================================================
   Wi-Fi / MQTT
   ===================================================== */
const char* ssid     = "";
const char* password = "";

const char* mqtt_server = "";
const int   mqtt_port   = 1883;

const char* mqtt_user   = "";
const char* mqtt_pass   = "";

// ClientID FIXO (não muda)
static const char* MQTT_CLIENT_ID = "esp32-dc01";

// tópicos principais
const char* topic_temp   = "sensor.temperatura";
const char* topic_umid   = "sensor.umidade";
const char* topic_fuma   = "sensor.fumaca";

// tópicos de alerta
const char* topic_alerta_temp = "sensor.alertaTemp";
const char* topic_alerta_fuma = "sensor.alertaFuma";

// status (online/offline) + LWT
const char* topic_status = "sensor.status";

/* =====================================================
   DETECÇÃO ROBUSTA: Mediana + MAD (sigma robusta) + Confirmação + Cooldown + Delta mínimo
   ===================================================== */
#define K_TEMP  4.0f
#define K_FUMA  4.5f

#define CONFIRM_M 3
#define CONFIRM_N 2

// (1) baseline maior (reduz falso positivo com baseline fraca)
#define BASELINE_MIN 8

#define COOLDOWN_TEMP_MS 30000UL
#define COOLDOWN_FUMA_MS 30000UL

// Delta mínimo (evita “anomalia” por variação muito pequena / ruído / quantização)
#define DELTA_MIN_TEMP 1.0f
#define DELTA_MIN_FUMA 2.0f

// (2) sigma robusta = 1.4826 * MAD, com piso (evita score explodir quando MAD ~ 0)
#define SIGMA_MIN_TEMP 0.20f
#define SIGMA_MIN_FUMA 0.30f

/* =====================================================
   Objetos
   ===================================================== */
WiFiClient espClient;
PubSubClient client(espClient);
DHT_Unified dht(DHTPIN, DHTTYPE);

/* =====================================================
   Estado do detector (sem struct): buffers + índices + flags
   ===================================================== */
// Temperatura
float tempBuf[10];
int   tempIdx = 0;
int   tempCount = 0;
int   tempFlags[CONFIRM_M] = {0};
int   tempFidx = 0;
unsigned long lastAlertTempMs = 0;
float prevTemp = 0.0f;

// “Fumaça” (ratio DSM501A)
float fumaBuf[10];
int   fumaIdx = 0;
int   fumaCount = 0;
int   fumaFlags[CONFIRM_M] = {0};
int   fumaFidx = 0;
unsigned long lastAlertFumaMs = 0;
float prevFuma = 0.0f;

// Contador do loop
unsigned long loopCounter = 0;

/* =====================================================
   Utilitários estatísticos: ordenar, mediana, MAD
   ===================================================== */
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

  // buffer local (máximo 10, pois nossos buffers têm tamanho 10)
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

/* =====================================================
   Operações básicas do detector: inserir valores e flags
   ===================================================== */
void pushValue(float *buf, int &idx, int &count, float v) {
  buf[idx] = v;
  idx = (idx + 1) % 10;
  if (count < 10) count++;
}

// armazena 1/0 em uma janela circular de tamanho CONFIRM_M e retorna a soma da janela
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

/* =====================================================
   Critério de anomalia:
   - baseline mínimo (BASELINE_MIN)
   - delta mínimo (evita ruído)
   - MAD escalado (sigma robusta) com piso
   ===================================================== */
bool isAnomaly(const float *buf, int count, float v, float prev,
               float K, float deltaMin, float sigmaMin) {
  if (count < BASELINE_MIN) return false;

  // Exige variação mínima entre leituras (bloqueia falsos positivos por ruído/quantização)
  if (fabs(v - prev) < deltaMin) return false;

  float med = medianOf(buf, count);
  float mad = madOf(buf, count);

  // sigma robusta aproximando desvio padrão (para dados ~normais)
  float sigma = 1.4826f * mad;

  // piso para impedir sigma ~ 0
  if (sigma < sigmaMin) sigma = sigmaMin;

  float z = fabs(v - med) / sigma;
  return (z > K);
}

/* =====================================================
   Processo de alerta genérico:
   - calcula anomalia (z > K) com delta mínimo
   - confirma usando CONFIRM_M/CONFIRM_N
   - aplica cooldown para reduzir repetição de alertas
   ===================================================== */
int processAlertGeneric(float value,
                        float *buf, int &idx, int &count,
                        int *flags, int &fidx,
                        float K,
                        float &prevValue,
                        float deltaMin,
                        float sigmaMin,
                        unsigned long cooldownMs,
                        unsigned long &lastAlertMs) {

  // Primeira amostra válida: inicializa prev e baseline
  if (count == 0) {
    prevValue = value;
    pushValue(buf, idx, count, value);
    return 0;
  }

  // Ainda formando baseline: só alimenta buffer
  if (count < BASELINE_MIN) {
    prevValue = value;
    pushValue(buf, idx, count, value);
    return 0;
  }

  // Decide anomalia com base no valor anterior + baseline
  bool anom = isAnomaly(buf, count, value, prevValue, K, deltaMin, sigmaMin);

  // Atualiza janela de confirmação
  int anomCount = pushFlag(flags, fidx, anom);
  bool confirmed = (anomCount >= CONFIRM_N);

  // Atualiza prev após avaliar
  prevValue = value;

  // Se não confirmou, apenas atualiza baseline e segue
  if (!confirmed) {
    pushValue(buf, idx, count, value);
    return 0;
  }

  // Se confirmou, aplica cooldown (evita alertas repetidos)
  unsigned long now = millis();
  if (now - lastAlertMs < cooldownMs) {
    pushValue(buf, idx, count, value);
    clearFlags(flags, fidx);
    return 0;
  }

  // Alerta válido
  lastAlertMs = now;
  clearFlags(flags, fidx);

  // Mantém o valor que gerou alerta no baseline (comportamento atual do seu código)
  pushValue(buf, idx, count, value);

  return 1;
}

/* =====================================================
   Publicação MQTT com “confirmação” (apenas prints de publicação)
   - imprime OK/FAIL para cada tópico
   - tenta reconectar e repete algumas vezes
   ===================================================== */
bool publishWithConfirm(const char* topic, const char* payload, bool retained, uint8_t attempts = 3) {
  for (uint8_t i = 1; i <= attempts; i++) {

    // Garante conexão antes de publicar
    if (!client.connected()) {
      reconnectMQTT();
      client.loop();
    }

    bool ok = client.publish(topic, payload, retained);

    if (ok) return true;

    // pequeno intervalo entre tentativas
    client.loop();
    delay(150);
  }
  return false;
}

/* =====================================================
   Conexão Wi-Fi
   ===================================================== */
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Timeout para não travar indefinidamente
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - t0 > 20000UL) {
      // Sem “debug extra”: apenas sai para não bloquear o loop
      return;
    }
  }
}

/* =====================================================
   Conexão MQTT com:
   - ClientID fixo
   - KeepAlive e socket timeout
   - LWT = "offline" (retained)
   - status online assim que conecta
   ===================================================== */
void reconnectMQTT() {
  client.setServer(mqtt_server, mqtt_port);

  client.setKeepAlive(20);
  client.setSocketTimeout(5);

  while (!client.connected()) {
    bool ok = client.connect(
      MQTT_CLIENT_ID,
      mqtt_user,
      mqtt_pass,
      topic_status, // LWT topic
      1,            // LWT QoS
      true,         // LWT retained
      "offline",    // LWT message
      true          // cleanSession
    );

    if (!ok) {
      delay(2000);
    }
  }

  // Ao conectar, sinaliza online (retained)
  client.publish(topic_status, "online", true);
  client.loop();
}

/* =====================================================
   Desconexão “graciosa”:
   - publica "offline" explicitamente
   - envia DISCONNECT MQTT
   - drena TCP
   - desliga Wi-Fi
   ===================================================== */
void disconnectWIFI() {
  if (client.connected()) {
    // Offline explícito (retained)
    client.publish(topic_status, "offline", true);
    client.loop();
    delay(150);

    // DISCONNECT MQTT
    client.disconnect();

    // Janela para drenar buffers TCP
    unsigned long t0 = millis();
    while (millis() - t0 < 300) {
      client.loop();
      delay(10);
    }
  }

  // Encerra socket TCP e desliga Wi-Fi
  espClient.stop();
  delay(50);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

/* =====================================================
   Leitura de sensores
   ===================================================== */
float ler_temperatura() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) return -1.0f;
  return event.temperature;
}

float ler_umidade() {
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) return -1.0f;
  return event.relative_humidity;
}

// DSM501A: ratio baseado em low pulse occupancy
float ler_fumaca() {
  lowpulseoccupancy = 0;
  starttime = millis();

  // Amostra durante sampletime_ms
  while ((millis() - starttime) < sampletime_ms) {
    duration = pulseIn(Dsm501aPIN, LOW, sampletime_ms * 1000UL);
    lowpulseoccupancy += duration;
  }

  // Converte para “ratio”
  ratio = lowpulseoccupancy / (sampletime_ms * 10.0f);
  return ratio;
}

/* =====================================================
   setup / loop
   ===================================================== */
void setup() {
  Serial.begin(115200);

  pinMode(Dsm501aPIN, INPUT);
  dht.begin();

  client.setServer(mqtt_server, mqtt_port);

  // Zera buffers
  for (int i = 0; i < 10; i++) {
    tempBuf[i] = 0.0f;
    fumaBuf[i] = 0.0f;
  }

  // Prevs inicializados para evitar delta com valores “lixo”
  prevTemp = 0.0f;
  prevFuma = 0.0f;
}

void loop() {
  loopCounter++;

  char buffer[32];

  // 1) Leituras
  float umidade     = ler_umidade();
  float temperatura = ler_temperatura();
  float fumaca      = ler_fumaca();

  // 2) Detecção (alertas 0/1)
  int alertaTemp = 0;
  int alertaFuma = 0;

  if (temperatura >= 0) {
    alertaTemp = processAlertGeneric(
      temperatura,
      tempBuf, tempIdx, tempCount,
      tempFlags, tempFidx,
      K_TEMP,
      prevTemp,
      DELTA_MIN_TEMP,
      SIGMA_MIN_TEMP,
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
      prevFuma,
      DELTA_MIN_FUMA,
      SIGMA_MIN_FUMA,
      COOLDOWN_FUMA_MS,
      lastAlertFumaMs
    );
  }

  // 3) Conecta e publica
  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    reconnectMQTT();
    client.loop();

    // 3.1) Publica alertas (retained)
    publishWithConfirm(topic_alerta_temp, (alertaTemp ? "1" : "0"), true, 3);
    publishWithConfirm(topic_alerta_fuma, (alertaFuma ? "1" : "0"), true, 3);

    // 3.2) Publica telemetria (retained)
    if (temperatura >= 0) {
      dtostrf(temperatura, 0, 2, buffer);
      publishWithConfirm(topic_temp, buffer, true, 3);
    }

    if (umidade >= 0) {
      dtostrf(umidade, 0, 2, buffer);
      publishWithConfirm(topic_umid, buffer, true, 3);
    }

    if (fumaca >= 0) {
      dtostrf(fumaca, 0, 2, buffer);
      publishWithConfirm(topic_fuma, buffer, true, 3);
    }

    // Janela curta para drenar publicações antes de desconectar
    unsigned long t0 = millis();
    while (millis() - t0 < 200) {
      client.loop();
      delay(10);
    }

    // 4) Print consolidado (mantém o que você já exibia)
    Serial.print("[LOOP] #"); Serial.print(loopCounter);
    Serial.print(" temp=");        Serial.print(temperatura);
    Serial.print(" umid=");        Serial.print(umidade);
    Serial.print(" fuma=");        Serial.print(fumaca);
    Serial.print(" alertaTemp=");  Serial.print(alertaTemp);
    Serial.print(" alertaFuma=");  Serial.println(alertaFuma);

    // 5) Desconexão graciosa (envia offline + DISCONNECT)
    disconnectWIFI();
  }

  // Intervalo entre ciclos (ajuste conforme necessário)
  delay(5000);
}
