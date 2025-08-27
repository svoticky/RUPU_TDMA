//Los sensores IR estan en los canales io4-A7-A6-A0-A3-io26 de izquierda a derecha
float sensoresMax[16];
float sensoresMin[16];


void calibrarSensores() {
  for (int i = 0; i <= 15; i++) {
    sensoresMax[i] = 0;
    sensoresMin[i] = 4095;
  }
  int vcal=200;
  encoder_der.write(0);
  encoder_izq.write(0);
  while(abs(radio_rueda*PI*(encoder_der.read()-encoder_izq.read())*N/(CPR*l))< 4*PI){
    motor(-vcal,vcal);
    for(byte i=0;i<=15;i++){
      digitalWrite(25, i & 0x01  );
      digitalWrite(26, i & 0x02  );
      digitalWrite(2, i & 0x04  );
      digitalWrite(4, i & 0x08 );
      setMaxMin(i, analogRead(39));
    }
  }
  motor(0,0);
  digitalWrite(25, 4 & 0x01  );
  digitalWrite(26, 4 & 0x02  );
  digitalWrite(2, 4 & 0x04  );
  digitalWrite(4, 4 & 0x08 );
  while((((1-2*linea)*100*(analogRead(39)-sensoresMin[4])/(sensoresMax[4]-sensoresMin[4])) + 100*linea)<80){
    motor(-vcal/2,vcal/2);
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  motor(0,0);
}

void setMaxMin(int i, float aread) {
  if (sensoresMax[i] < aread) {
    sensoresMax[i] = aread;
  }
  if (sensoresMin[i] > aread) {
    sensoresMin[i] = aread;
  }
}

float getposition(bool direccion){
    // VARIABLES FUNCION getposition
  float sensorNormalizado[8];
  float inte[8];
  float sum1=0;
  float area=0;
  if(direccion){
    for(byte i=0;i<=7;i++){
      digitalWrite(25, i & 0x01  );
      digitalWrite(26, i & 0x02  );
      digitalWrite(2, i & 0x04  );
      digitalWrite(4, i & 0x08 );
      sensorNormalizado[i]=((1-2*linea)*100*(analogRead(39)-sensoresMin[i])/(sensoresMax[i]-sensoresMin[i])) + 100*linea; //MAPEO DE SENSORES
      if(sensorNormalizado[i]<30){
        sensorNormalizado[i]=0;
      }
    }
  }
  else{
    for(byte i=0;i<=7;i++){
      digitalWrite(25, (i+8) & 0x01  );
      digitalWrite(26, (i+8) & 0x02  );
      digitalWrite(2, (i+8) & 0x04  );
      digitalWrite(4, (i+8) & 0x08 );
      sensorNormalizado[i]=((1-2*linea)*100*(analogRead(39)-sensoresMin[i+8])/(sensoresMax[i+8]-sensoresMin[i+8])) + 100*linea; //MAPEO DE SENSORES
      if( sensorNormalizado[i]<10){
        sensorNormalizado[i]=0;
      }
    }
  }

  for(int i=0; i<=7;i++){
    sum1+=sensorNormalizado[i]*(-28+8*i);
    area+=sensorNormalizado[i];
  }//
  if(area==0){
    parar ="si";
    return 0;
  }
  else{
      return sum1/area; //centroide
  }
  
}


void configuracion_sensor_d(){
  // CONFIGURACION SENSOR DE DISTANCIA ////////////////////////////////////                         
  Wire.begin();                                                       ///
  sensor.setTimeout(500);                                             ///
  while (!sensor.init())                                                 ///
  {                                                                   ///
    Serial.println("Failed to detect and initialize sensor!");        ///                                                    ///
    delay(1000); 
  }                                                                   ///
  sensor.setSignalRateLimit(0.25);                                    ///
  sensor.setMeasurementTimingBudget(33000);                           ///
  sensor.startContinuous(); //toma medidas cada {Argumento} ms     ///
/////////////////////////////////////////////////////////////////////////
}

double distancia(){
  double dist = sensor.readReg16Bit(sensor.RESULT_RANGE_STATUS + 10)*0.100 - 2; //Lee el registro que almacena las mediciones del sensor, se realiza una
                                                                                            //operacion matematica para ajustar la medicion y expresarla en centimetros
  if(dist>80){ //Saturación de la medicion en 80[cm]
    cuenta=0;
    dist=80;
  }
  return dist;
}


void velocidades(){
  t_actual = millis() - t_svel;
  if(t_actual>=10){
    v_r[1]=v_r[0];
    v_r[2]=v_r[1];
    v_r[3]=v_r[2];
    v_r[4]=v_r[3];
    v_l[1]=v_l[0];
    v_l[2]=v_l[1];
    v_l[3]=v_l[2];
    v_l[4]=v_l[3];
    v_r[0]=2*PI*radio_rueda*encoder_der.read()*N/(t_actual*0.001*CPR); // ENCODERS:28CPR---CAJA_REDUCTORA:100:1  [cm/s]
    v_l[0]=2*PI*radio_rueda*encoder_izq.read()*N/(t_actual*0.001*CPR); // ENCODERS:28CPR---CAJA_REDUCTORA:100:1  [cm/s]
    encoder_der.write(0);
    encoder_izq.write(0);
    t_svel=millis();
  }
  v_r[0]=(0.2*(v_r[1]+v_r[2]+v_r[3]+v_r[4])/4) + 0.8*v_r[0];
  v_l[0]=(0.2*(v_l[1]+v_l[2]+v_l[3]+v_l[4])/4) + 0.8*v_l[0];
  Input_vel=0.5*(v_r[0]+v_l[0]); // [cm/s]
  ang_vel=0.5*(v_r[0]-v_l[0])/l;
}

void curvatura_pista(){
  t_actual = millis() - t_arco;
  if(t_actual>=20){
    a_sl+=v_l[0]*t_actual*0.001;
    a_sr+=v_r[0]*t_actual*0.001;
    b_sl+=v_l[0]*t_actual*0.001;
    b_sr+=v_r[0]*t_actual*0.001;
    c_sl+=v_l[0]*t_actual*0.001;
    c_sr+=v_r[0]*t_actual*0.001;
    t_arco=millis();
  }
  a_s=0.5*(a_sr+a_sl);
  b_s=0.5*(b_sr+b_sl);
  c_s=0.5*(c_sr+c_sl);
  if(Input_vel<0.1){
    a_curvatura=0;
    b_curvatura=0;
    c_curvatura=0;
    a_sl=0;
    a_sr=0;
    b_sl=0;
    b_sr=0;
    c_sl=0;
    c_sr=0;
  }
  else if (a_s>=3 && b_s>=10){
    b_curvatura=abs((0.5*(b_sr-b_sl))/(l*b_s));
    b_sl=0;
    b_sr=0;
  }
  else if(a_s>=3 && a_s==b_s){
    b_sl=0;
    b_sr=0;
  }
  else if (a_s>=6 && c_s>=10){
    c_curvatura=abs((0.5*(c_sr-c_sl))/(l*c_s));
    c_sl=0;
    c_sr=0;
  }
  else if(a_s>=6 && a_s==c_s){
    c_sl=0;
    c_sr=0;
  }
  else if(a_s>=10){
    a_curvatura=abs((0.5*(a_sr-a_sl))/(l*a_s));
    a_sl=0;
    a_sr=0;
  }
  if(a_curvatura<0.01 && b_curvatura<0.01 && c_curvatura<0.01){
    recta=1;
  }
  else if(a_curvatura>0.02 && b_curvatura>0.02 && c_curvatura>0.02){
    recta=0;
  }
  if(recta){
    if(a_curvatura >= b_curvatura && a_curvatura >= c_curvatura){
      curvatura=a_curvatura;
    }
    else if(b_curvatura >= a_curvatura && b_curvatura >= c_curvatura){
      curvatura=b_curvatura;
    }
    else if(c_curvatura >= a_curvatura && c_curvatura >= b_curvatura){
      curvatura=c_curvatura;
    }
    else{
      curvatura=a_curvatura;
    }
  }
  else{
    if(a_curvatura < b_curvatura && a_curvatura < c_curvatura){
      curvatura=a_curvatura;
    }
    else if(b_curvatura < a_curvatura && b_curvatura < c_curvatura){
      curvatura=b_curvatura;
    }
    else if(c_curvatura < a_curvatura && c_curvatura < b_curvatura){
      curvatura=c_curvatura;
    }
    else{
      curvatura=a_curvatura;
    }
  }
}
