void udp_recep() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(paquete_entrante, 64);
    if (len > 0) {
      paquete_entrante[len] = 0;
    }
    if (paquete_entrante[0] == 'C'){
      estado = Calibracion;
    }
    else if (paquete_entrante[0] == 'E') {          //ESCRITURA DE PARAMETROS
      cadena = configuracion_remota(len);
      cadena.toCharArray(msg, cadena.length() + 1);
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.printf(msg);
      udp.endPacket();
    }
    else if (paquete_entrante[0] == 'T'){
      lectura_TDMA(len);
    }
  }
}

void lectura_TDMA(int len) {
  String CANTIDAD;
  int i = 2;
  while (paquete_entrante[i] != '/' && i < len) {
    char nuevo = paquete_entrante[i];
    CANTIDAD.concat(nuevo);
    i++;
  }
  String DURACION;
  int j = i + 1;
  i++;
  while (paquete_entrante[i] != '/' && i < len) {
    char nuevo = paquete_entrante[i];
    DURACION.concat(nuevo);
    i++;
  }
  String T_INICIO;
  int k = i + 1;
  i++;
  while (paquete_entrante[i] != '/' && i < len) {
    char nuevo = paquete_entrante[i];
    T_INICIO.concat(nuevo);
    i++;
  }
  cantidad_peloton = CANTIDAD.toInt();
  duracion_ronda_TDMA = DURACION.toInt();
  inicio_ronda_TDMA = T_INICIO.toInt();
}


void envio_TDMA(long inicio_siguiente_ronda){

  //cadena = "T/" + String(cantidad_peloton) + "/" + String(duracion_ronda_TDMA) + "/" + String(inicio_siguiente_ronda) + "/" + String(micros());
  //cadena.toCharArray(msg, cadena.length() + 1);
  //udp.beginPacket("192.168.1.255", localPort);
  //udp.printf(msg);
  //udp.endPacket();
  mensajes_enviados += 1;
  if (contador_mensajes_TDMA == 0) myData.tipo_mensaje = 'M';
  parar.toCharArray(myData.para, parar.length()+1);
  myData.vel_1[0] = Input_vel;
  myData.vel_2[0] = vel_ref;
  myData.curva[0] = curvatura;
  myData.enviados[0] = mensajes_enviados;
  //myData.atiempo[0] = mi_inicio_TDMA;
  //myData.time_stamp[3] = micros();
  myData.datos_TDMA[0] = cantidad_peloton;
  myData.datos_TDMA[1] = duracion_ronda_TDMA;
  myData.datos_TDMA[2] = inicio_siguiente_ronda;
  myData.datos_TDMA[3] = rondas_tdma;
  esp_now_send(mac_broadcast, (uint8_t *) &myData, sizeof(myData));
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //memcpy(&myData, incomingData, sizeof(myData));

  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));            // copia rápida
  msg.time_stamp[contador_estaciones - 1] = micros() + desfase;
  xQueueSend(qMsg, &msg, 0);                          // encola sin bloquear

}

inline void processEspNowMsg(struct_message &msg) {

  // Índices usados en tu lógica original
  const int i1 = contador_estaciones - 1;
  const int i2 = contador_estaciones - 2;

  // Una sola lectura de tiempo
  time_adjusted= micros() + desfase;

  if (msg.tipo_mensaje == 'V') {
    mensajes_V_recibidos += 1;

    msg.time_stamp[i1]  = msg.time_stamp[i1] - inicio_predecesor_TDMA;
    msg.recibidos_v[i1] = mensajes_V_recibidos;
    msg.atiempo[i1] = myData.atiempo[i1];
    msg.recibidos_m[i1] = myData.recibidos_m[i1];

    //msg.atiempo[i1] = time_adjusted; // si lo quieres activar
      

  } else if (msg.tipo_mensaje == 'M') {
    mensajes_M_recibidos += 1;
    parar = String(msg.para);

    // vel_crucero = msg.vel_1[i2];               // si lo necesitas
    sat_d = msg.vel_2[i2];                        // saturación PID
    curvatura_predecesor = msg.curva[i2];
    
    //if (myData.datos_TDMA[2] < msg.datos_TDMA[2]){
    msg.atiempo[i1] = time_adjusted;
    if (contador_estaciones == cantidad_peloton) {
      msg.time_stamp[i1] = msg.time_stamp[i1] - inicio_predecesor_TDMA;
    }
    //}
    msg.recibidos_m[i1] = mensajes_M_recibidos;

    // Actualiza parámetros TDMA si corresponde
    //if (msg.datos_TDMA[2] >= time_adjusted) {
    cantidad_peloton     = msg.datos_TDMA[0];
    duracion_ronda_TDMA  = msg.datos_TDMA[1];
    inicio_ronda_TDMA    = msg.datos_TDMA[2];
    //}
  }
  myData = msg;  
}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


///////////////////////////////CONFIGURACION ESP-NOW////////////////

void setup_esp_now(){

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  qMsg = xQueueCreate(10, sizeof(struct_message));
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  if (rango_robot == 1){
    
    // Register peer
    memcpy(peerInfo.peer_addr, mac_monitor, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Register peer
    memcpy(peerInfo.peer_addr, mac_broadcast, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }

    //duracion_ronda_TDMA = 40000;
    contador_estaciones = 1;
    inicio_ronda_TDMA = micros() + 10000;
    cantidad_peloton = WiFi.AP.stationCount() + 1;
    envio_TDMA(inicio_ronda_TDMA);

  } else {

    int values[6];
    const char* macCStr = macList[contador_estaciones-2].c_str();
    if (sscanf(macCStr, "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) == 6) {
      for (int i = 0; i < 6; ++i) {
        mac_sucesor[i] = (uint8_t)values[i];
      }
    }

    //esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, mac_sucesor, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Register peer
    //memcpy(peerInfo.peer_addr, mac_monitor, 6);
    //peerInfo.channel = 0;  
    //peerInfo.encrypt = false;
  
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    
  }
}



String configuracion_remota(int len) {
  String variable;
  int i = 1;
  while ((paquete_entrante[i] != '/' || i == 1) && i < len) {
    char nuevo = paquete_entrante[i];
    variable.concat(nuevo);
    i++;
  }
  String dato;
  int j = i + 1;
  while (i < len - 1) {
    char nuevo = paquete_entrante[i + 1];
    dato.concat(nuevo);
    i++;
  }
  //Serial.println(variable);
  //Serial.println(dato);
  //Serial.println(j);
  //Serial.println(paquete_entrante[j]);
  //Serial.println(dato.toFloat());
  //Serial.println(len);
  if ((dato.toFloat() == 0 && (j < len && paquete_entrante[j] != '0')) && variable != "/parar") {
    return "datoIncorrecto";
  }
  if (variable == "/co_p") {
    Kp_theta = dato.toFloat();
    //PID_theta.SetTunings(Kp_theta, Ki_theta, Kd_theta);
  }
  else if (variable == "/co_i") {
    Ki_theta = dato.toFloat();
    //PID_theta.SetTunings(Kp_theta, Ki_theta, Kd_theta);
  }
  else if (variable == "/co_d") {
    Kd_theta = dato.toFloat();
    //PID_theta.SetTunings(Kp_theta, Ki_theta, Kd_theta);
  }
  else if (variable == "/cv_p") {
    Kp_vel = dato.toFloat();
    //PID_vel.SetTunings(Kp_vel, Ki_vel, Kd_vel);
  }
  else if (variable == "/cv_i") {
    Ki_vel = dato.toFloat();
    //PID_vel.SetTunings(Kp_vel, Ki_vel, Kd_vel);
  }
  else if (variable == "/cv_d") {
    Kd_vel = dato.toFloat();
    //PID_vel.SetTunings(Kp_vel, Ki_vel, Kd_vel);
  }
  else if (variable == "/cv_ref") {
    sp_vel = dato.toFloat();
  }
  else if (variable == "/cd_p") {
    Kp_d = dato.toFloat();
    //PID_d.SetTunings(Kp_vel, Ki_vel, Kd_vel);
  }
  else if (variable == "/cd_i") {
    Ki_d = dato.toFloat();
    //PID_d.SetTunings(Kp_vel, Ki_vel, Kd_vel);
  }
  else if (variable == "/cd_d") {
    Kd_d = dato.toFloat();
    //PID_d.SetTunings(Kp_vel, Ki_vel, Kd_vel);
  }
  else if (variable == "/cd_delta") {
    delta = dato.toFloat();
  }
  else if (variable == "/cd_ref") {
    d_ref = dato.toFloat();
  }
  else if (variable == "/calibrar") {
    calibrar = dato.toFloat();
  }
  else if (variable == "/parar") {
    parar = dato;
  }
  else {
    return "incorrecto";
  }
  return "ok";
}
