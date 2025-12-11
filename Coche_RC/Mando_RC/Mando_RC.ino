#include <esp_now.h>

#include <WiFi.h>

#include <esp_wifi.h> 


// Pines Inputs

const int pinVolante  = 34;

const int pinAcelerador = 35;

const int pinGearUp = 32;

const int pinGearDown = 33;

bool gearHold = false;

// MAC del coche (receptor)

uint8_t macCoche[] = {0x44, 0x1D, 0x64, 0xE2, 0xE8, 0x98};

// *** NUEVO *** MAC de la base (receptor adicional)
uint8_t macBase[]  = {0x6C, 0xC8, 0x40, 0x5D, 0x23, 0x1C};

// Estructura de datos a enviar

typedef struct __attribute__((packed)) {

 uint8_t volante;

 uint8_t acelerador;

 uint8_t marcha;

} ControlData;


ControlData controlData;


// Callback opcional para saber si se ha enviado bien

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {

 // De momento paso del parámetro 'info', solo miro el estado

 //Serial.print("\r\nLast Packet Send Status:\t");

 //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}


void setup() {

 Serial.begin(115200);

 //Serial.println("Mando ESP32 arrancando...");


 pinMode(pinVolante, INPUT);

 pinMode(pinAcelerador, INPUT);

 pinMode(pinGearUp, INPUT);

 pinMode(pinGearDown, INPUT);


 // Modo estación obligatorio para ESP-NOW

 WiFi.mode(WIFI_STA);


 // Inicializar ESP-NOW

 if (esp_now_init() != ESP_OK) {

  //Serial.println("Error inicializando ESP-NOW");

  return;

 }


  controlData.acelerador = 0;
  controlData.volante = 0;
  controlData.marcha = 0;

  esp_now_register_send_cb(onDataSent);


 // Registrar al coche como peer (destino)

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, macBase, 6);

  peerInfo.channel = 0;  // mismo canal, 0 = actual

  peerInfo.encrypt = false;




 if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //Serial.println("Error añadiendo peer");
  return;
 }



}


void loop() {

 // Leer potenciometros (0-4095)

  int inputVolante  = analogRead(pinVolante);

  int inputAcelerador = analogRead(pinAcelerador);

  int inputGearUp = digitalRead(pinGearUp);

  int inputGearDown = digitalRead(pinGearDown);

  if(!gearHold)
  {
    gearHold = true;
    if(inputGearUp == 1 && controlData.marcha < 8)
    {
      controlData.marcha++;
    }
    if(inputGearDown == 1 && controlData.marcha > -1)
    {
      controlData.marcha--;
    }
  }
  if(inputGearUp == 0 && inputGearDown == 0)
  {
    gearHold = false;
  }


 // Mapear a 0-255 (como ya hacías, invertido)

  controlData.volante  = map(inputVolante,  0, 4095, 255, 0);

  controlData.acelerador = map(inputAcelerador, 0, 4095, 255, 0);


 // Enviar paquete al coche

  esp_err_t result = esp_now_send(macBase, (uint8_t *)&controlData, sizeof(controlData));


 // Log opcional por Serial

  if (result == ESP_OK) 
  {

    Serial.print(controlData.volante);

    Serial.print(",");

    Serial.print(controlData.acelerador);

    Serial.print(",");

    Serial.print(inputGearUp);

    Serial.print(",");

    Serial.print(inputGearDown);

    Serial.print(",");

    Serial.println(controlData.marcha);
    

  }

}