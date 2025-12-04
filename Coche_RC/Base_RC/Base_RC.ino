#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Mismo struct que manda el coche
typedef struct __attribute__((packed)) {
  uint8_t volante;
  uint8_t acelerador;
} ControlData;

volatile ControlData ultimo;
volatile bool nuevoDato = false;

// Callback de recepción (COCHE -> BASE)
void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlData)) {
    memcpy((void*)&ultimo, incomingData, sizeof(ControlData));
    nuevoDato = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Base ESP32 arrancando...");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    // Si quisieras, aquí podrías hacer Serial.println("0,0")
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  if (nuevoDato) {
    nuevoDato = false;

    ControlData dato;
    memcpy(&dato, (const void*)&ultimo, sizeof(ControlData));

    // De momento solo mando los 2 primeros campos
    Serial.print(dato.volante);
    Serial.print(",");
    Serial.println(dato.acelerador);
  }else{
    Serial.print(0);
    Serial.print(",");
    Serial.println(0);
  }
}
