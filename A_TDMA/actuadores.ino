
void analogWriteNormalized(uint8_t pin, uint32_t value, uint32_t valueMaxUser) {
  uint32_t levels = (1 << pwmResolution_bits);
  uint32_t duty = (value > valueMaxUser) ? levels - 1 : ((levels - 1) * value) / valueMaxUser;
  analogWrite(pin, duty);
}

void motor(int Velocidad_motor_izq, int Velocidad_motor_der)
{ 
  /*
   * Función de motor motor(M1, M2).
   * Mueve los motores con velocidad M1 y M2 con un valor entero entre 0 y 1024
   * En caso de introducir un número negativo, el motor se mueve en sentido inverso.
  */

  if (Velocidad_motor_der >= resolucionPWM){
    Velocidad_motor_der=resolucionPWM;  
  }
  else if(Velocidad_motor_der <= -resolucionPWM){
    Velocidad_motor_der=-resolucionPWM;
  }
  if (Velocidad_motor_izq >= resolucionPWM){
    Velocidad_motor_izq=resolucionPWM;  
  }
  else if(Velocidad_motor_izq <= -resolucionPWM){
    Velocidad_motor_izq=-resolucionPWM;
  }
  if (Velocidad_motor_der > 0)
  {
    digitalWrite(ain1, HIGH); 
    digitalWrite(ain2, LOW);
    analogWriteNormalized(pwm_a, Velocidad_motor_der, resolucionPWM);
    //analogWrite(pwm_a, Velocidad_motor_der); 
  }

  else if (Velocidad_motor_der < 0)
  {
    digitalWrite(ain1, LOW); 
    digitalWrite(ain2, HIGH);
    analogWriteNormalized(pwm_a, -Velocidad_motor_der, resolucionPWM);
    //analogWrite(pwm_a, -Velocidad_motor_der); 
  }

  else
  {
    digitalWrite(ain1, LOW); 
    digitalWrite(ain2, LOW);
    analogWriteNormalized(pwm_a, Velocidad_motor_der, resolucionPWM);
    //analogWrite(pwm_a, Velocidad_motor_der); 
  }

  if (Velocidad_motor_izq > 0)
  {
    digitalWrite(bin1, HIGH); 
    digitalWrite(bin2, LOW);
    analogWriteNormalized(pwm_b, Velocidad_motor_izq, resolucionPWM);
    //analogWrite(pwm_b, Velocidad_motor_izq);   
  }

  else if (Velocidad_motor_izq < 0)
  {
    digitalWrite(bin1, LOW); 
    digitalWrite(bin2, HIGH);
    analogWriteNormalized(pwm_b, -Velocidad_motor_izq, resolucionPWM); 
    //analogWrite(pwm_b, -Velocidad_motor_izq);
  }

  else
  {
    digitalWrite(bin1, LOW); 
    digitalWrite(bin2, LOW);
    analogWriteNormalized(pwm_b, Velocidad_motor_izq, resolucionPWM);
    //analogWrite(pwm_b, Velocidad_motor_izq);  
  }
}