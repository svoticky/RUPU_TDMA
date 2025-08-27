// Variables Secuencia de Inicio Peloton
int estoy_ordenado = 0;   //Variable para saber si conozco mi lugar o no
int tarea_iniciada = 0;

String mymac;

int estado_movimiento = 0;
TaskHandle_t movimiento_handle = NULL;

bool macExists(const String& mac) {
  for (const auto& m : macList) {
    if (m.equalsIgnoreCase(mac)) {
      return true;
    }
  }
  return false;
}

int findMacIndex(const String& macToFind) {
  for (size_t i = 0; i < macList.size(); i++) {
    if (macList[i].equalsIgnoreCase(macToFind)) {
      return i;  // Retorna el índice si la encuentra
    }
  }
  return -1;  // Retorna -1 si no está en el vector
}

void patron_movimiento_task(void * parameters){
  for(;;){

    if (estado_movimiento == 0){
      motor(0, 0);
      estado_movimiento = 1;
    } else if (estado_movimiento == 1){
      motor(200, 200);
      estado_movimiento = 2;
    } else if (estado_movimiento == 2){
      motor(-200, -200);
      estado_movimiento = 0;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void secuencia_inicio_peloton(){
  //Serial.println("Estoy iniciando el peloton!");
  int distancia_original = distancia();
  long tiempo_anterior = millis();

  if (rango_robot == 1) delay(1000);                 //Delay del lider para esperar a que todos se conecten

  while (ordenar_lugar == 1){

    if (rango_robot == 1){                           ///////////////// Si es lider ///////////////////

      if (estoy_ordenado == 0){                      // Si no fue reconocido por el segundo

        if (tarea_iniciada == 0){
          xTaskCreatePinnedToCore(
            patron_movimiento_task,  //Function Name
            "Secuencia movimiento",  //Task Name
            1024,                    //Stack Size   
            NULL,                    //Task Parameters
            1,                       //Task Priority
            &movimiento_handle,      //Task Handle
            1                        //Task Core 
          );
          tarea_iniciada = 1;
          mymac = WiFi.macAddress();                  // Manda la MAC de su STA para comunicarse por ESP-NOW por ahi
          mymac.trim();  // Quitar espacios
          macList.push_back(mymac);

        }                                             // Si ya inicio la tarea entonces escucha hasta que el segundo responda
                                                      // y manda su MAC para que el segundo la guarde
      }
      int packetSize = udp.parsePacket();
      if (packetSize) {
        int len = udp.read(paquete_entrante, 64);
        if (len > 0) paquete_entrante[len] = 0;
        if (estoy_ordenado == 0){
          estoy_ordenado = 1;
          vTaskDelete(movimiento_handle);
          motor(0,0);
        }
        contador_estaciones += 1;
        Serial.println(paquete_entrante);
        String mac = String(paquete_entrante);
        mac.trim();  // Quitar espacios
        macList.push_back(mac);
        //Serial.println("MAC válida agregada.");
        delay(10);
      }
      macList[contador_estaciones].toCharArray(msg, macList[contador_estaciones].length() + 1);
      udp.beginPacket("192.168.1.255", localPort);
      udp.printf(msg);
      udp.endPacket();
      delay(50);
                                                  // Si el segundo ya lo reconocio, escucha a los demas y cuando todos esten listos termina
      if (contador_estaciones == WiFi.AP.stationCount()){
        udp.beginPacket("192.168.1.255", localPort);
        udp.printf("Todos listos\n");
        udp.endPacket();
        ordenar_lugar = 0;
        // Mostrar todas las MAC guardadas hasta ahora
        //Serial.println("Lista actual de MACs:");
        //for (const auto& m : macList) {
          //Serial.println(m);
        //}
      }  
      

    } else {                                                      //////////// Si es seguidor //////////////

      if (estoy_ordenado == 0){                                  // Si todavia no ve a alguien moverse entonces vigila el sensor de distancia
        delay(200);
        if (abs(distancia_original - distancia()) >= 5){
          udp.beginPacket("192.168.1.255", localPort);
          mymac = WiFi.softAPmacAddress();                      // Envia la MAC de su AP para comunicarse por ESP-NOW por ahi
          mymac.toCharArray(msg, mymac.length() + 1);
          udp.printf(msg);
          udp.endPacket();
          estoy_ordenado = 1;
        }
      } else {                                                  // Si ya vio a alguien entonces inicia su propio movimiento

        if (tarea_iniciada == 0){
          xTaskCreatePinnedToCore(
            patron_movimiento_task,  //Function Name
            "Secuencia movimiento",  //Task Name
            1024,                    //Stack Size   
            NULL,                    //Task Parameters
            1,                       //Task Priority
            &movimiento_handle,      //Task Handle
            1                        //Task Core 
          );
          tarea_iniciada = 1;

        } 
      }

      int packetSize = udp.parsePacket();
      if (packetSize) {                                  // Escucha y espera a asignar MAC de sucesor
        int len = udp.read(paquete_entrante, 64);
        if (len > 0) paquete_entrante[len] = 0;
        if (paquete_entrante[0] != 'T' && ordenar_lugar == 1) { //Si ya esta ordenado y el mensaje es una MAC entonces la guarda en la lista
          String mac = String(paquete_entrante);
          mac.trim();                                   
          if (!macExists(mac)) {
            contador_estaciones += 1;
            macList.push_back(mac);
            //Serial.println("agregue mac");
          }
        }
        if (udp.remoteIP() != WiFi.gatewayIP() || paquete_entrante[0] == 'T'){ //Si el mensaje viene de alguien que no sea el lider o es el mensaje de termino entonces deja de moverse
          if (movimiento_handle != NULL) vTaskDelete(movimiento_handle);
          movimiento_handle = NULL;
          motor(0,0);
          delay(10);
          if (paquete_entrante[0] == 'T'){                //Si llega el mensaje de que Todos estan sincronizados entonces guarda los datos recolectados y vuelve a la maquina de estados
            ordenar_lugar = 0;
            mymac = WiFi.softAPmacAddress();
            contador_estaciones = findMacIndex(mymac) + 1;
            cantidad_peloton = macList.size();
          }
        }
      }
      delay(10);

    }
  }
}


