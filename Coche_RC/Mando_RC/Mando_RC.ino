#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// =====================
// Pines
// =====================
const int pinVolante     = 34;
const int pinAcelerador  = 35;
const int pinGearUp      = 32;
const int pinGearDown    = 33;

const int pinButtonMode  = 19;  // Botón (INPUT con resistencia externa)
const int pinLedMode     = 16;  // LED modo

const int pinLedRD = 25; // LED1
const int pinLedRI = 26; // LED2
const int pinLedAD = 27; // LED3
const int pinLedAI = 13; // LED4

// =====================
// ESP-NOW (MACs)
// =====================
uint8_t macCoche[] = {0x44, 0x1D, 0x64, 0xE2, 0xE8, 0xE8, 0x98}; // (no usado aquí)
uint8_t macBase[]  = {0x6C, 0xC8, 0x40, 0x5D, 0x23, 0x1C};       // destino actual

// =====================
// Datos a enviar
// =====================
typedef struct __attribute__((packed)) {
  uint8_t volante;
  uint8_t acelerador;
  uint8_t marcha;
} ControlData;

ControlData controlData;

// =====================
// Estado
// =====================
bool rcMode = true;     // POR DEFECTO: RC
bool gearHold = false;

int revolutions    = 0;
int revolutionsMax = 4095; // por defecto hasta que Unity mande el valor

bool espNowReady = false;

// Lecturas actuales
int inputVolante     = 0;
int inputAcelerador  = 0;
int inputGearUp      = 0;
int inputGearDown    = 0;
int inputButtonMode  = 0;

// =====================
// Callback ESP-NOW
// =====================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  (void)status;
}

// =====================
// Prototipos
// =====================
void initPins();

bool enableEspNow();
void disableEspNow();
void applyMode();

void readInputs();
void updateModeButton();

void receiveRevolutionsFromSerial();

void updateGear();
void updateLedsFromRevs();
void updateControlData();

void processOutputs();

void clearBarLeds();
void setAllBarLeds(bool state);

// =====================
// Setup / Loop
// =====================
void setup() {
  Serial.begin(115200);

  initPins();

  controlData.volante = 0;
  controlData.acelerador = 0;
  controlData.marcha = 1;

  applyMode(); // arranca en rcMode=true -> LED modo HIGH y ESP-NOW ON
}

void loop() {
  readInputs();
  updateModeButton();

  receiveRevolutionsFromSerial();
  updateGear();
  updateLedsFromRevs();
  updateControlData();

  processOutputs();
}

// =====================
// Implementación
// =====================
void initPins() {
  pinMode(pinVolante, INPUT);
  pinMode(pinAcelerador, INPUT);

  pinMode(pinGearUp, INPUT);
  pinMode(pinGearDown, INPUT);

  pinMode(pinButtonMode, INPUT); // resistencia externa, sin PULLUP interno
  pinMode(pinLedMode, OUTPUT);

  pinMode(pinLedRD, OUTPUT);
  pinMode(pinLedRI, OUTPUT);
  pinMode(pinLedAD, OUTPUT);
  pinMode(pinLedAI, OUTPUT);

  digitalWrite(pinLedMode, LOW);
  clearBarLeds();
}

bool enableEspNow() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return false;

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macBase, 6); // destino
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    esp_now_deinit();
    return false;
  }

  espNowReady = true;
  return true;
}

void disableEspNow() {
  if (espNowReady) {
    esp_now_deinit();
    espNowReady = false;
  }
  WiFi.mode(WIFI_OFF);
}

void applyMode() {
  gearHold = false;

  if (rcMode) {
    // MODO RC
    digitalWrite(pinLedMode, HIGH);
    clearBarLeds();

    controlData.marcha = 1; // marchas deshabilitadas

    if (!espNowReady) {
      enableEspNow(); // si falla, simplemente no enviará
    }
  } else {
    // MODO UNITY
    digitalWrite(pinLedMode, LOW);
    disableEspNow(); // ESP-NOW OFF
  }
}

void readInputs() {
  inputVolante    = analogRead(pinVolante);
  inputAcelerador = analogRead(pinAcelerador);

  inputGearUp     = digitalRead(pinGearUp);
  inputGearDown   = digitalRead(pinGearDown);

  inputButtonMode = digitalRead(pinButtonMode);
}

// Alterna rcMode en cada pulsación (debounce + flanco)
// OJO: aquí asumo que la pulsación da HIGH (pull-down externo).
// Si tu pulsación da LOW (pull-up externo), se cambia a (stable == 0).
void updateModeButton() {
  static int lastRaw = 0;
  static int stable = 0;
  static unsigned long lastChangeMs = 0;
  const unsigned long DEBOUNCE_MS = 30;

  int raw = inputButtonMode;

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = millis();
  }

  if (millis() - lastChangeMs >= DEBOUNCE_MS) {
    if (stable != raw) {
      stable = raw;

      // Detecta pulsación (flanco a HIGH)
      if (stable == 1) {
        rcMode = !rcMode;
        applyMode();
      }
    }
  }
}

// Espera líneas: "3500,12000\n"
// rev -> revolutions, max -> revolutionsMax
// Solo en modo Unity
void receiveRevolutionsFromSerial() {
  if (rcMode) return;

  static char buf[48];
  static uint8_t idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      idx = 0;

      char *comma = strchr(buf, ',');
      if (!comma) return;

      *comma = '\0';
      char *a = buf;
      char *b = comma + 1;

      long rev = strtol(a, nullptr, 10);
      long mx  = strtol(b, nullptr, 10);

      if (mx < 1) mx = 1;
      if (rev < 0) rev = 0;
      if (rev > mx) rev = mx;

      revolutions = (int)rev;
      revolutionsMax = (int)mx;

      return;
    }

    if (idx < sizeof(buf) - 1) buf[idx++] = c;
    else idx = 0; // overflow -> resetea
  }
}

// Marchas solo en modo Unity
void updateGear() {
  if (rcMode) {
    gearHold = false;
    controlData.marcha = 1;
    return;
  }

  if (!gearHold) {
    gearHold = true;

    if (inputGearUp == 1 && controlData.marcha < 9) {
      controlData.marcha++;
    }
    if (inputGearDown == 1 && controlData.marcha > 0) {
      controlData.marcha--;
    }
  }

  if (inputGearUp == 0 && inputGearDown == 0) {
    gearHold = false;
  }
}

// LEDs de revoluciones solo en modo Unity
void updateLedsFromRevs() {
  if (rcMode) {
    clearBarLeds();
    return;
  }

  int v = (revolutionsMax > 0) ? map(revolutions, 0, revolutionsMax, 0, 9) : 0;
  v = constrain(v, 0, 8);

  static unsigned long lastBlink = 0;
  static bool blinkOn = false;

  if (v == 8) {
    if (millis() - lastBlink >= 100) {
      lastBlink = millis();
      blinkOn = !blinkOn;
    }
    setAllBarLeds(blinkOn);
    return;
  }

  clearBarLeds();

  switch (v) {
    case 7: digitalWrite(pinLedRD, HIGH); // LED1 (>=7)
    case 6: digitalWrite(pinLedRI, HIGH); // LED2 (>=6)
    case 5: digitalWrite(pinLedAD, HIGH); // LED3 (>=5)
    case 4: digitalWrite(pinLedAI, HIGH); // LED4 (>=4)
            break;
    default:
            break; // 0..3: ninguno
  }
}

void updateControlData() {
  controlData.volante    = map(inputVolante,    0, 4095, 255, 0);
  controlData.acelerador = map(inputAcelerador, 0, 4095, 255, 0);
}

// RC: envía por ESP-NOW y NO imprime Serial
// Unity: NO envía ESP-NOW y SÍ imprime Serial
void processOutputs() {
  if (rcMode) {
    if (espNowReady) {
      esp_now_send(macBase, (uint8_t *)&controlData, sizeof(controlData));
    }
    return;
  }

  Serial.print(controlData.volante);
  Serial.print(",");
  Serial.print(controlData.acelerador);
  Serial.print(",");
  Serial.println(controlData.marcha);
}

void clearBarLeds() {
  digitalWrite(pinLedRD, LOW);
  digitalWrite(pinLedRI, LOW);
  digitalWrite(pinLedAD, LOW);
  digitalWrite(pinLedAI, LOW);
}

void setAllBarLeds(bool state) {
  digitalWrite(pinLedRD, state);
  digitalWrite(pinLedRI, state);
  digitalWrite(pinLedAD, state);
  digitalWrite(pinLedAI, state);
}
