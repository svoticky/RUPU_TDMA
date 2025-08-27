double calculoPID(double y,double ref,double &error_ant,double &error_integral,double kp,double ki,double kd,double limite, String MODO, double &out_manual, String direccion){
  if(MODO=="MANUAL"){
    return out_manual;
  }
  else if(MODO=="AUTO"){
    double u;
    double error;
    if (direccion=="DIRECTO"){
      error=ref-y;
    }
    else if (direccion=="INVERSO"){
       error=y-ref;
    }
    error_integral+=error*t_actual*0.001;
    if(ki*error_integral>limite){
      error_integral=limite/ki;      
    }
    else if(ki*error_integral<-limite){
      error_integral=-limite/ki; 
    }
    u=kp*error + kd*(error-error_ant)/(t_actual*0.001) + ki*error_integral;
    error_ant=error;
    if(u>limite){
      //error_integral=limite;
      return limite; 
    }
    else if(u<-limite){
      //error_integral=-limite;
      return -limite;
    }
    else{
      return u;
    }
  }
}

double calculoPIDd(double y,double ref,double &error_ant,double &error_integral,double kp,double ki,double kd,double limite, String MODO, double &out_manual, String direccion){
  int satMax = 0.1;
  if(MODO=="MANUAL"){
    return out_manual;
  }
  else if(MODO=="AUTO"){
    double u;
    double error;
    if (direccion=="DIRECTO"){
      error=ref-y;
    }
    else if (direccion=="INVERSO"){
       error=y-ref;
    }
    error_integral+=error*t_actual*0.001;
    if(ki*error_integral>limite){
      error_integral=limite/ki;      
    }
    else if(ki*error_integral<-limite){
      error_integral=-limite/ki; 
    }
    u=kp*error + kd*(error-error_ant)/(t_actual*0.001) + ki*error_integral;
    error_ant=error;
    if(u > limite*(1+satMax)+1){
      //error_integral=limite;
      return limite*(1+satMax)+1; 
    }
    else if(u < limite*(1-satMax)-1){
      //error_integral=-limite;
      return limite*(1-satMax)-1;
    }
    else{
      return u;
    }
  }
}
