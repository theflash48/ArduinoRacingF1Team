# CONTROLADOR DE VEHICULOS VOLADORES EN UNITY USANDO ARDUINO

## OBJETIVO DEL PROYECTO
El objetivo de este proyecto es realizar un prototipo de un Formula 1 + Mando + Base conectada a Unity 6 en el que se mostrará la telemetría.

Este es un proyecto individual.

## SPRINTS

Sprint 1:
7/11/2025
Primer prototipo consistente en tinkercad.

10/11/2025
Recepción de todos los componentes de Aliexpress.

11/11/2025
Mando montado, completamente funcional y conectividad inalambria via ESP-NOW lograda entre los 2 ESP32, casi preparado para enviarlo al 3o conectado a Unity

13 y 18/11/2025
Sesiones dedicadas a soldar componentes.

20/11/2025
Intentar hacer que funcione la pantalla del mando IRL durante más de 10h, objetivo postpuesto a otro sprint.

24/11/2025
Compra de las baterias y de la protoboard del coche, montaje general del coche pero con problemas con los transistores que llevan a un atraso considerable para el siguiente sprint.

25/11/2025
Esquema de tinkercad reiniciado debido a problemas encontrados en el proyecto fisico, intento de montar un puente H, pero no se ha probado en tinkercad, no se sabe si funciona

## BOCETO DEL PROYECTO

<img width="903" height="721" alt="Captura de pantalla 2025-11-25 165246" src="https://github.com/user-attachments/assets/4d2ff88a-d0bf-4b85-bbd5-df9be9d64182" />

<img width="1033" height="816" alt="Captura de pantalla 2025-11-25 165141" src="https://github.com/user-attachments/assets/40e408a6-dea5-401b-8802-bc7ac39f0e16" />

## CÓDIGO DE ARDUINO (el codigo de tinkercad y el que se pondrá en el proyecto no tienen relación pues se va a trabajar en 3 ESP32 y no en 1 Arduino UNO)


---MANDO:

#include <WiFi.h>

#include <esp_now.h>

#include <esp_wifi.h>   // importante con el core nuevo


// MAC del mando (por si luego quieres responderle)

uint8_t macMando[] = {0x44, 0x1D, 0x64, 0xE3, 0xBE, 0xA0};


// Estructura recibida

typedef struct __attribute__((packed)) {

  uint8_t volante;

  uint8_t acelerador;

} ControlData;


volatile ControlData ultimo;

volatile bool nuevoDato = false;


// NUEVA firma del callback con el core actual:

//  info -> datos del envío (no la usamos ahora)

//  incomingData -> puntero al payload

//  len -> longitud del payload

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {

  if (len == sizeof(ControlData)) {

    memcpy((void*)&ultimo, incomingData, sizeof(ControlData));

    nuevoDato = true;

  }

}


void setup() {

  Serial.begin(115200);

  Serial.println("Coche ESP32 arrancando...");


  // Modo estación obligatorio para ESP-NOW

  WiFi.mode(WIFI_STA);


  // Inicializar ESP-NOW

  if (esp_now_init() != ESP_OK) {

    Serial.println("Error inicializando ESP-NOW");

    return;

  }


  // Registrar callback de recepción con la nueva firma

  esp_now_register_recv_cb(onDataRecv);

}


void loop() {

  if (nuevoDato) {

    nuevoDato = false;

    Serial.print("RECIBIDO -> Volante: ");

    Serial.print(ultimo.volante);

    Serial.print("  Acelerador: ");

    Serial.println(ultimo.acelerador);

  }

}

---COCHE:

#include <esp_now.h>

#include <WiFi.h>

#include <esp_wifi.h>  


// Pines Inputs

const int pinVolante    = 34;

const int pinAcelerador = 35;


// MAC del coche (receptor)

uint8_t macCoche[] = {0x44, 0x1D, 0x64, 0xE2, 0xE8, 0x98};


// Estructura de datos a enviar

typedef struct __attribute__((packed)) {

  uint8_t volante;

  uint8_t acelerador;

} ControlData;


ControlData controlData;


  // Callback opcional para saber si se ha enviado bien

  void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {

  Serial.print("\r\nLast Packet Send Status:\t");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}


void setup() {

  Serial.begin(115200);

  Serial.println("Mando ESP32 arrancando...");


  pinMode(pinVolante, INPUT);

  pinMode(pinAcelerador, INPUT);


  // Modo estación obligatorio para ESP-NOW

  WiFi.mode(WIFI_STA);


  // Inicializar ESP-NOW

  if (esp_now_init() != ESP_OK) {

    Serial.println("Error inicializando ESP-NOW");

    return;

  }


  esp_now_register_send_cb(onDataSent);


  // Registrar al coche como peer (destino)

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, macCoche, 6);

  peerInfo.channel = 0;    // mismo canal, 0 = actual

  peerInfo.encrypt = false;


  if (esp_now_add_peer(&peerInfo) != ESP_OK) {

    Serial.println("Error añadiendo peer");

    return;

  }

}


void loop() {

  // Leer potenciometros

  int inputVolante    = analogRead(pinVolante);

  int inputAcelerador = analogRead(pinAcelerador);


  // Mapear a 0-255

  controlData.volante    = map(inputVolante,    0, 4095, 255, 0);

  controlData.acelerador = map(inputAcelerador, 0, 4095, 255, 0);


  // Enviar paquete al coche

  esp_err_t result = esp_now_send(macCoche, (uint8_t *)&controlData, sizeof(controlData));


  // Log opcional por Serial

  if (result == ESP_OK) {

    Serial.print("Enviado -> Volante: ");

    Serial.print(controlData.volante);

    Serial.print("  Acelerador: ");

    Serial.println(controlData.acelerador);

  } else {

    Serial.println("Error enviando datos");

  }

}
