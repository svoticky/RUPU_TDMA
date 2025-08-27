TaskHandle_t udp_sync_handle = NULL;

long time_now = 0;
long time_past = 0; 
long time_other_1;
long time_other_2;
long round_trip_1 = 0;
long round_trip_2 = 0;
//long desfase_2 = 0;
long time_prediction;
int contador_mensajes = 0; 

int estado_seguidor = 0;
int estado_sync = 0;
int contador_sincronizados = 0;
int sync_iniciada = 0;
int digito_ip_seguidor = 2;
char ip_seguidor[13];

void udp_sync_task(void * parameters){
  for(;;){

    if (conectado == 1){                                  //Revisa que haya una conexion con el lider

      if (rango_robot == 1){                     /////Si es Lider//////

        int packetSize = udp.parsePacket();

        if (packetSize) {                                  //Espera a recibir mensajes del seguidor
          int len = udp.read(paquete_entrante, 64);
          if (len > 0) paquete_entrante[len - 1] = 0;

          if (paquete_entrante[0] == 'O'){                 //Revisa si es el mensaje de termino o no
            //Serial.println("Sincronizado listo");
            estado_sync = 1;
            vTaskSuspend(NULL);
          }
          
          time_past = micros();                            //Guarda el tiempo en el que recibio el mensaje
          udp.beginPacket(ip_seguidor, localPort);         //Le contesta al seguidor con el tiempo de recepcion
          char buf[20];
          sprintf(buf, "%lu", time_past);
          udp.printf("%s\r\n", buf);
          udp.endPacket();
        }

        if (micros() - time_now >= 20000){                 //Envia cada 20ms un mensaje con el tiempo en el que lo envio y espera a recibir respuesta del seguidor
          time_now = micros();
          udp.beginPacket(ip_seguidor, localPort);
          char buf[20];
          sprintf(buf, "%lu", time_now);
          udp.printf("%s\r\n", buf);
          udp.endPacket();

        }
        vTaskDelay(1 / portTICK_PERIOD_MS);

      } else{                                   //////Si es Seguidor/////

        int packetSize = udp.parsePacket();

        if (packetSize) {                                    //Espera a recibir mensaje del Lider al comienzo del intervalo de los 20ms
          int len = udp.read(paquete_entrante, 64);
          if (len > 0) paquete_entrante[len - 1] = 0;

          if (estado_seguidor == 0){                         //Si es el primer mensaje del intervalo entonces guarda el tiempo recibido y el recepcion
            time_past = micros();
            time_other_1 = atol(paquete_entrante);
            time_adjusted = time_past + desfase + round_trip_1/2;

            udp.beginPacket(udp.remoteIP(), udp.remotePort()); //Responde al lider para que este mande otro mensaje con su tiempo de recepcion
            char buf[20];
            sprintf(buf, "%lu", time_adjusted);
            udp.printf("%s\r\n", buf);
            udp.endPacket();

            estado_seguidor = 1;

          } else if (estado_seguidor == 1){                   //Si ya recibio el primer mensaje entonces entra aqui para procesar la informacion del primer y segundo mensaje
            time_now = micros();
            time_other_2 = atol(paquete_entrante);

            round_trip_1 = time_other_2 - time_other_1;        //Calcula los round trips con los tiempos de los dos mensajes 
            round_trip_2 = time_now - time_past;
            int promedio = (round_trip_1 + round_trip_2)/4;

            desfase = time_other_2 - time_now + (round_trip_2/2);  //Calcula el desfase 
            time_prediction = time_now + desfase - promedio;       //Hace una euristica con el promedio de los round trips
            contador_mensajes += 1;

            if (abs(time_prediction - time_other_2)<= 100 && contador_mensajes >= 30){  //Verifica que la diferencia entre el tiempo pasado y la euristica sea menor a 100 microsegundos
              //Serial.println("Sincronizado listo");
              for (int i = 0; i <= 3 ; i++) {                        //Le confirma al lider que se llegó a la sincronizacion
                udp.beginPacket(udp.remoteIP(), udp.remotePort());
                udp.printf("OK \r\n");
                udp.endPacket();
              }
              
              estado_sync = 1;
              vTaskSuspend(NULL);
            }

            estado_seguidor = 0;

          }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }

    }

  }
}


void sincronizar() {
  while (sincronizado == 0){

    if (rango_robot == 1){                          //////Si es lider//////
      
      if (sync_iniciada == 1){                                   //Revisa si ya esta en el proceso de sincronizacion con una esp32 o no

        if (estado_sync == 1){                                   //Revisa si ya completo la sincronizacion con la esp32 actual
          vTaskDelete(udp_sync_handle);
          contador_sincronizados += 1;
          digito_ip_seguidor += 1;
          if (contador_sincronizados == WiFi.AP.stationCount()){ //Revisa si ya se sincronizo con todas o no
            //Serial.println("Todos estan sincronizados");
            sincronizado = 1;
          }else {                                                //Si no estan sincronizadas todas entonces inicia otra ronda con la siguiente IP
            sync_iniciada = 0;
            estado_sync = 0;
          }
        }
        vTaskDelay(25 / portTICK_PERIOD_MS);

      } else {                                                  //Envia un mensaje por UDP para iniciar la sincronizacion con la IP del sguidor que toca
        sprintf(ip_seguidor, "192.168.1.%d", digito_ip_seguidor);
        udp.beginPacket(ip_seguidor, localPort);
        udp.printf("Sincronizamos?");
        udp.endPacket();

        int packetSize = udp.parsePacket();
        if (packetSize) {
          int len = udp.read(paquete_entrante, 64);
          if (len > 0) paquete_entrante[len - 1] = 0;

          if (paquete_entrante[0] == 'B'){                     //Si el sguidor responde a la solicitud de sincronizacion entonces incia la tarea para sincronizar
            Serial.println("Task sync iniciada");
            vTaskDelay(25 / portTICK_PERIOD_MS);
            xTaskCreatePinnedToCore(
              udp_sync_task,    //Function Name
              "Sync UDP",       //Task Name
              2048,           //Stack Size   
              NULL,           //Task Parameters
              1,              //Task Priority
              &udp_sync_handle,     //Task Handle
              1               //Task Core 
            );

            sync_iniciada = 1;
          }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }

    } else{                                         //////Si es Seguidor//////

      if (sync_iniciada == 1){                               //Revisa si esta en proceso de sincronizacion con el lider o no

        if (estado_sync == 1){                               //Revisa si ya termino de sincronizar
          sincronizado = 1;
          vTaskDelete(udp_sync_handle);
        }
        vTaskDelay(25 / portTICK_PERIOD_MS);

      } else {                                               //Si es que no esta en proceso de sync entonces espera a recibir el mensaje de iniciacion

        int packetSize = udp.parsePacket();
        if (packetSize) {
          int len = udp.read(paquete_entrante, 64);
          if (len > 0) paquete_entrante[len - 1] = 0;
          if (paquete_entrante[0] == 'S'){

            for (int i = 0; i <= 2 ; i++) {                  // Si recibe el mensaje de iniciacion entonces contesta para iniciar el proceso
              udp.beginPacket(udp.remoteIP(), udp.remotePort());
              udp.printf("Bueno\r\n");
              udp.endPacket();
            }
            //Serial.println("Task sync iniciada");
            xTaskCreatePinnedToCore(
              udp_sync_task,    //Function Name
              "Sync UDP",       //Task Name
              2048,           //Stack Size   
              NULL,           //Task Parameters
              1,              //Task Priority
              &udp_sync_handle,     //Task Handle
              1               //Task Core 
             );

            sync_iniciada = 1;
          }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }

    }

  }
}


