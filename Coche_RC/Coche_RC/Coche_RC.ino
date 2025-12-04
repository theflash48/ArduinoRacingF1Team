#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>   // importante con el core nuevo

// MAC del mando (por si luego quieres responderle)
uint8_t macMando[] = {0x44, 0x1D, 0x64, 0xE3, 0xBE, 0xA0};

// *** NUEVO *** MAC de la base (receptor adicional)
uint8_t macBase[]  = {0x6C, 0xC8, 0x40, 0x5D, 0x23, 0x1C};

// Estructura recibida
typedef struct __attribute__((packed)) {
  uint8_t volante;
  uint8_t acelerador;
} ControlData;

volatile ControlData ultimo;
volatile bool nuevoDato = false;

const uint8_t pinMotor = 25;

// Callback de recepción (MANDO -> COCHE)
void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlData)) {
    memcpy((void*)&ultimo, incomingData, sizeof(ControlData));
    nuevoDato = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Coche ESP32 arrancando...");

  pinMode(pinMotor, OUTPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  // *** NUEVO *** Registrar a la BASE como peer (destino)
  esp_now_peer_info_t peerInfoBase = {};
  memcpy(peerInfoBase.peer_addr, macBase, 6);
  peerInfoBase.channel = 0;    // mismo canal, 0 = actual
  peerInfoBase.encrypt = false;

  if (esp_now_add_peer(&peerInfoBase) != ESP_OK) {
    Serial.println("Error añadiendo peer BASE");
    return;
  }
}

void loop() {
  if (nuevoDato) {
    nuevoDato = false;

    // *** LIGERO CAMBIO: copiar a variable local para trabajar tranquilos ***
    ControlData dato;
    memcpy(&dato, (const void*)&ultimo, sizeof(ControlData));

    Serial.print("RECIBIDO -> Volante: ");
    Serial.print(dato.volante);
    Serial.print("  Acelerador: ");
    Serial.println(dato.acelerador);

    analogWrite(pinMotor, dato.acelerador);

    // Aquí luego metes el control de motor / dirección

    // *** NUEVO *** Reenviar ese mismo dato a la BASE
    esp_err_t result = esp_now_send(macBase, (uint8_t *)&dato, sizeof(dato));

    // Log opcional (si ves que spamea demasiado se puede comentar)
    if (result != ESP_OK) {
      Serial.println("Error enviando datos a BASE");
    }
  }
}
