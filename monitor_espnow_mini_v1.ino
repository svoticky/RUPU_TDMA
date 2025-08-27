#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

uint8_t mac_formateada[6];

int peer_registrado = 1;

#define MAX_LISTAS 4

typedef struct struct_message {
    char tipo_mensaje;
    int enviados[MAX_LISTAS];
    int recibidos_m[MAX_LISTAS];
    int recibidos_v[MAX_LISTAS];
    char para[3];
    long atiempo[MAX_LISTAS];
    double vel_1[MAX_LISTAS];
    double vel_2[MAX_LISTAS];
    double curva[MAX_LISTAS];
    long time_stamp[MAX_LISTAS];
    long datos_TDMA[4];
} struct_message;

//struct_message myData;
//struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

QueueHandle_t qMsg;

void guardar_mac_lider(String mac_lider){
  int values[6];
  const char* macCStr = mac_lider.c_str();
  //Serial.println(macCStr);
  if (sscanf(macCStr, "%x:%x:%x:%x:%x:%x",
            &values[0], &values[1], &values[2],
            &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; ++i) {
      mac_formateada[i] = (uint8_t)values[i];
    }
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  struct_message incomingReadings;
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));            // copia rápida
  xQueueSend(qMsg, &incomingReadings, 0);                                       // encola sin bloquear
  
}

void consumerTask(void *pv) {
  struct_message incomingReadings;
  for(;;) {
    if (xQueueReceive(qMsg, &incomingReadings, portMAX_DELAY) == pdTRUE) {
      // Procesa tranquilo aquí (orden FIFO garantizado)
      if (incomingReadings.tipo_mensaje == 'M' || incomingReadings.tipo_mensaje == 'V'){
        if (peer_registrado == 0){
          guardar_mac_lider(incomingReadings.para);
          // Register peer
          memcpy(peerInfo.peer_addr, mac_formateada, 6);
          peerInfo.channel = 0;  
          peerInfo.encrypt = false;
          // Add peer        
          if (esp_now_add_peer(&peerInfo) != ESP_OK){
            Serial.println("Failed to add peer");
            return;
          } else {
            peer_registrado = 1;
          }
        } else {
          //Serial.print(incomingReadings.para);
          int i = 0;
          while (i < MAX_LISTAS && incomingReadings.vel_1[i] != -1) {
            //Serial.print("Datos[");
            //Serial.print(i);
            //Serial.print("] = ");
            Serial.printf("Datos[%d]=%lu;%d;%d;%d;%ld;%ld;%.3f;%.3f;%.3f;%ld\n", i, micros()
                           ,incomingReadings.enviados[i],incomingReadings.recibidos_m[i],incomingReadings.recibidos_v[i]
                           ,incomingReadings.atiempo[i],incomingReadings.datos_TDMA[2],incomingReadings.vel_1[i]
                           ,incomingReadings.vel_2[i],incomingReadings.curva[i],incomingReadings.time_stamp[i]);
            //Serial.printf(" %d ;",incomingReadings.enviados[i]);
            //Serial.printf(" %d ;",incomingReadings.recibidos_m[i]);
            //Serial.printf(" %d ;",incomingReadings.recibidos_v[i]);
            //Serial.printf(" %ld ;",incomingReadings.atiempo[i]);
            //Serial.printf(" %d ;",incomingReadings.datos_TDMA[2]);
            //Serial.printf(" %f ;",incomingReadings.vel_1[i]);
            //Serial.printf(" %f ;",incomingReadings.vel_2[i]);
            //Serial.printf(" %f ;",incomingReadings.curva[i]);
            //Serial.printf(" %ld \n",incomingReadings.time_stamp[i]);
            i++;
          }
          //Serial.printf(" %d  ",incomingReadings.datos_TDMA[0]);
          //Serial.printf(" %d  ",incomingReadings.datos_TDMA[1]);
          //Serial.printf(" %d  ",incomingReadings.datos_TDMA[2]);
          //Serial.printf(" %d  ",incomingReadings.datos_TDMA[3]);
          //Serial.println();
        }
      }
    }
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //esp_now_register_send_cb(OnDataSent);
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  qMsg = xQueueCreate(10, sizeof(struct_message));
  xTaskCreatePinnedToCore(consumerTask, "consumer", 4096, NULL, 4, NULL, 1);
}

void loop(){
  delay(1000);
  //Serial.println(WiFi.macAddress());
}







