#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h> //I2C
#include <VL53L0X.h>
#include <vector>
#include <esp_now.h>


///     WIFI
///////////////////////////////////////
//     Configuración RED   ///////////
///////////////////////////////////////
#define ssid "esp32"
#define ssid_seguidor "esp32_seguidor"  
#define password "12345678"

#define IP_Lider "192.168.1.1"       //ip del robot lider
#define MAX_LISTAS 4                 //Numero maximo de robots en el peloton (cambiar si es necesario)

////////
#define led 2
#define radio_rueda 2.15 //RADIO EN [CM]
#define l 5.6 // l corresponde a la distancia entre las ruedas en cm
#define N 0.01 // Modulo de REDUCCION 
#define CPR 28 // Cuentas por revolucion encoders
#define d1 52
#define d2 32
//
/*  Pines motor 1 (derecho)*/
#define ain1  16
#define ain2  27
#define pwm_a  17

/*  Pines motor 2 (izquierdo)*/
#define bin1  13
#define bin2  12
#define pwm_b  14

#define resolucionPWM 1023
/////////////////////////////SUPERFICIE
#define linea 1  // valor logico 1 corresponde a una pista con trayectoria color blanco; Valor lógico 0 corresponde a una pista con trayectoria color negro. 



// MAQUINA DE ESTADOS
#define Inicio 0
#define Formacion_de_Peloton 1
#define Sincronizacion 2
#define Calibracion 3
#define Loop_de_Control 4
int estado = 0;
int calibrar = 1;
String parar = "no";     //permite saber si el robot se salió de la pista

int ordenar_lugar = 1;   // Variable para saber si es que hay que seguir ejecutando la secuencia de orden
int sincronizado = 0;
int conectado = 0;
long desfase = 0;
int rango_robot;        // 1 si es lider y 0 si es seguidor
long time_adjusted;     //tiempo ajustado segun el desafe calculado
const uint8_t pwmResolution_bits = 10;         // bits


/////////////////////////////////////////////////////////////// VARIABLES CONTROLADORES PID
//Define Variables we'll be connecting to PID de posicion
double Input_theta, Output_theta;
double theta_ref = 0;
//Define the aggressive and conservative Tuning Parameters
/*PID posición*/
double Kp_theta = 2000;//64.078;//147.657974619537;//60;
double Ki_theta = 4500;//145.3619;//555.734047342947;//150;
double Kd_theta = 60;//7.0616;//8.30454187876464;//1;

//Define Variables we'll be connecting to PID de Velocidad
double Input_vel, Output_vel;
double Output_vel_ant = 0;
double vel_ref = 0;
double sp_vel = 0;
double vel_crucero = 15;

//Define the aggressive and conservative Tuning Parameters
/*PID velocidad*/
double Kp_vel = 20.3;//100;//49.9757241214599;//130;
double Ki_vel = 145.3;//282.271150682254;//130;
double Kd_vel = 0;//0.197722771627088;//0;

//Define Variables we'll be connecting to PID de distancia
double Input_d, Output_d;
double d_ref = 10;
double error_d;
float delta = 0.1;

//Define the aggressive and conservative Tuning Parameters
/*PID velocidad*/
double Kp_d = 4;//1.5;//8.0013;
double Ki_d = 3;//11;//5.6025;
double Kd_d = 0;//0.1;//1.0077;
//Specify the links and initial tuning parameters
String PID_theta="MANUAL";
double sat_theta=1023;
double error_ant_theta=0;
double integral_theta=0;
String PID_vel="MANUAL";
double sat_vel=800;
double error_ant_vel=0;
double integral_vel=0;
String PID_d="MANUAL";
double sat_d=30;
double error_ant_d=0;
double integral_d=0;
/////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////// VARIABLES UDP
WiFiUDP udp;
String cadena;
char msg[128];
char paquete_entrante[64];  // buffer for incoming packets
unsigned int localPort = 5555;
unsigned long t_com_predecesor, t_monitoreo;


//////////////////////////////// VARIABLES ESP-NOW
uint8_t mac_sucesor[6];
uint8_t mac_predecesor[6];
const uint8_t mac_monitor[6] = {0xE0,0x5A,0x1B,0x77,0x7D,0x78};           //Cambiar por la MAC de la ESP32 que vaya a ser usada para monitorear
const uint8_t mac_broadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
esp_now_peer_info_t peerInfo;
std::vector<String> macList;

//////////////////////////////// VARIABLES TDMA

int cantidad_peloton = 0;                   //Cantidad de robots en el peloton
long duracion_ronda_TDMA = 40000;           //Tiempo que dura una ronda TDMA en microsegundos ()
long inicio_ronda_TDMA = 0;                 //Tiempo en el que inicia la ronda general de TDMA
int contador_estaciones = 0;                //Lugar en el que el se encuentra el robot dentro del peloton
int contador_mensajes_TDMA = 0;             //Cuenta cuantos mensajes manda el lider con los datos TDMA
int cantidad_mensajes_por_ronda_TDMA = 2;   //Limita cuantos mensajes puede mandar el lider para no saturar las comunicaciones
int contador_rondas_TDMA = 0;               //Cuenta cada cuantas rondas el Lider envia los mensajes TDMA
int enable_envio_seguidor = 0;              //Verifica si el seguidor ya envio un mensaje exitoso o no (Solo Unicast)

int slot_TDMA;                              //Es el tiempo que tiene cada robot para transmitir 
long mi_inicio_TDMA;                         //El inicio del slot de cada robot que depende de su lugar en el peloton
long mi_final_TDMA;                          //El final del slot de cada robot 
long inicio_predecesor_TDMA;                 //El inicio de la ronda del robot anterior para calcular cuanto tarda en llegar el mensaje

int mensajes_enviados = 0;
int mensajes_M_recibidos = 0;
int mensajes_V_recibidos = 0;
int rondas_tdma = 0;

long ajuste_fase = 0;


////////////////////////////////////////////////////////////

unsigned long t_actual;     //TIMER GENERAL
unsigned long t_controlador;

//////////SENSORES
double ang_vel;
double v_r[] = {0, 0, 0, 0, 0};
double v_l[] = {0, 0, 0, 0, 0};
double a_sl=0;
double a_sr=0;
double b_sl=0;
double b_sr=0;
double c_sl=0;
double c_sr=0;
double a_s=0;
double b_s=0;
double c_s=0;
unsigned long t_arco;
byte recta=0;
double a_curvatura=0;
double b_curvatura=0;
double c_curvatura=0;
double curvatura=0;
double curvatura_predecesor=0;
unsigned long t_svel;
double cuenta=0;

//Structure example to send data
//Must match the receiver structure
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

struct_message myData;

QueueHandle_t qMsg;

VL53L0X sensor;     //SENSOR DE DISTANCIA I2C

Encoder encoder_der(5, 23);
Encoder encoder_izq(18, 19);

byte control=1;

void ICACHE_RAM_ATTR encoder1_ISR() {    //Funcion para que los enconcers se actualicen correctamente
  encoder_der.read();
}

void ICACHE_RAM_ATTR encoder2_ISR() {
  encoder_izq.read();
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup!");

  attachInterrupt(digitalPinToInterrupt(5), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), encoder2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), encoder2_ISR, CHANGE);

  pinMode(ain1, OUTPUT); 
  pinMode(ain2, OUTPUT); 
  pinMode(bin1, OUTPUT); 
  pinMode(bin2, OUTPUT); 

  analogWriteFrequency(pwm_a, 500);
  analogWriteFrequency(pwm_b, 500);
  analogWriteResolution(pwm_a, pwmResolution_bits);
  analogWriteResolution(pwm_b, pwmResolution_bits);

  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);

  configuracion_sensor_d();
  delay(50);

  if (distancia() > 70) rango_robot = 1;                      //Condicion para determinar si es lider o no
  else if (distancia() < 60) rango_robot = 0;

  if (rango_robot == 1){
    IPAddress ap_ip(192, 168, 1, 1);
    IPAddress ap_mask(255, 255, 255, 0);
    IPAddress ap_leaseStart(192, 168, 1, 2);
    IPAddress ap_dns(8, 8, 4, 4);
    
    WiFi.mode(WIFI_AP_STA);
    WiFi.AP.begin();
    WiFi.AP.config(ap_ip, ap_ip, ap_mask, ap_leaseStart, ap_dns);
    WiFi.AP.create(ssid, password);
    if (!WiFi.AP.waitStatusBits(ESP_NETIF_STARTED_BIT, 1000)) {
      Serial.println("Failed to start AP!");
      return;
    }
    Serial.println("Soy Lider!");

  } else if (rango_robot == 0){
    WiFi.mode(WIFI_AP_STA);
    WiFi.AP.begin();
    WiFi.AP.create(ssid_seguidor, password);
    WiFi.begin(ssid, password, 1);
    Serial.println("Soy Seguidor!");
  }
  delay(100);
  udp.begin(localPort);
}

void loop(){
  //Serial.print("ESTADO   :");
  //Serial.println(estado);

  switch (estado) {
    
    case Inicio:
      ciclo_de_inicio();
      if (ordenar_lugar == 1) estado = Formacion_de_Peloton;
      if (calibrar == 0) {
        estado = Loop_de_Control;
        setup_esp_now();
      }
      if (rango_robot == 1) sp_vel = 15;
      break;

    case Formacion_de_Peloton:
      if (WiFi.status() == WL_CONNECTED || WiFi.AP.stationCount() >= 1) conectado = 1;
      if (conectado == 1 && ordenar_lugar == 1) {
        if (rango_robot == 1) vTaskDelay(5000 / portTICK_PERIOD_MS);
        secuencia_inicio_peloton();
      }
      if (ordenar_lugar == 0) estado = Sincronizacion;
      break;

    case Sincronizacion:
      if (sincronizado == 0) sincronizar();
      else if (sincronizado == 1){
        if (rango_robot == 0)  udp_recep();
        if (rango_robot == 1){
          for (int i = 0; i <= 3 ; i++) {
            udp.beginPacket("192.168.1.255", localPort);
            udp.printf("Calibrar \r\n");
            udp.endPacket();
            vTaskDelay(1 / portTICK_PERIOD_MS);
          }
          estado = Calibracion;
        }
        //estado = Calibracion;
      }
      break;

    case Calibracion:
      calibrarSensores();
      calibrar = 0;
      estado = Inicio;
      if (rango_robot == 1) sp_vel = 15;      //Condicion para que avance el lider
      break;

    case Loop_de_Control:

      //if (parar == "si") estado = Inicio;
      ciclo_control_TDMA();
      
      break;

    default:
      ciclo_de_inicio();
      break;
  }
  //vTaskDelay(1 / portTICK_PERIOD_MS);
}

bool pollEspNowQueue(uint32_t max_items = 0, uint32_t time_budget_us = 0) {  //Funcion para leer los mensajes en la cola
  // max_items=0 => sin límite por cantidad
  // time_budget_us=0 => sin límite por tiempo
  uint64_t t0 = micros();
  uint32_t count = 0;
  struct_message msg;
  bool did = false;

  while (xQueueReceive(qMsg, &msg, 0) == pdTRUE) {
    did = true;
    processEspNowMsg(msg);
    count++;
    if (max_items && count >= max_items) break;
    if (time_budget_us && (micros() - t0) >= time_budget_us) break;
  }
  return did;
}

void ciclo_control_TDMA(){

  if (rango_robot == 0){
    slot_TDMA = duracion_ronda_TDMA/cantidad_peloton;
    mi_inicio_TDMA = inicio_ronda_TDMA + (slot_TDMA*(cantidad_peloton - contador_estaciones));
    mi_final_TDMA = mi_inicio_TDMA + slot_TDMA;
    inicio_predecesor_TDMA = mi_inicio_TDMA - slot_TDMA;
    time_adjusted = micros() + desfase;

    if (time_adjusted >= mi_inicio_TDMA && time_adjusted < mi_final_TDMA){
      if (enable_envio_seguidor == 0){
        mensajes_enviados += 1;
        myData.tipo_mensaje = 'V';
        parar.toCharArray(myData.para, parar.length()+1);
        myData.vel_1[contador_estaciones-1] = Input_vel;
        myData.vel_2[contador_estaciones-1] = vel_ref; 
        myData.curva[contador_estaciones-1] = curvatura;
        myData.enviados[contador_estaciones-1] = mensajes_enviados;
        //myData.recibidos_m[contador_estaciones-1] = mensajes_M_recibidos;
        //myData.atiempo[contador_estaciones-1] = mi_inicio_TDMA;
        //myData.time_stamp[contador_estaciones-1] = time_adjusted;
        esp_err_t result = esp_now_send(mac_sucesor, (uint8_t *) &myData, sizeof(myData));
        if (result == ESP_OK) {
          enable_envio_seguidor = 1;
        }
      }
    }
    //else if (time_adjusted >= mi_final_TDMA){
      //inicio_ronda_TDMA += duracion_ronda_TDMA;
    //}
    else {
      pollEspNowQueue(/*max_items=*/4, /*time_budget_us=*/5000);
      enable_envio_seguidor = 0;
      ciclo_de_control();
    }
  } 
  else if (rango_robot == 1){
    //cantidad_peloton = WiFi.AP.stationCount() + 1;
    slot_TDMA = duracion_ronda_TDMA/cantidad_peloton;
    mi_inicio_TDMA = inicio_ronda_TDMA + (slot_TDMA)*(cantidad_peloton - 1);
    mi_final_TDMA = mi_inicio_TDMA + slot_TDMA;
    inicio_predecesor_TDMA = mi_inicio_TDMA - slot_TDMA;
    time_adjusted = micros();

    if (time_adjusted >= mi_inicio_TDMA && time_adjusted < mi_final_TDMA){
      if (contador_mensajes_TDMA < cantidad_mensajes_por_ronda_TDMA){
        int enable_ajuste_fase = 0;
        for (int i = 0; i < MAX_LISTAS-1; i++) {
          if (myData.time_stamp[i] > 6000) {
            enable_ajuste_fase = 1;
            if (myData.time_stamp[i] < 15000) {
              ajuste_fase = myData.time_stamp[i];
            } else{
              ajuste_fase = 10000;
            }
          }
        }
        if (enable_ajuste_fase == 0) ajuste_fase = 0;
        myData.atiempo[0] = ajuste_fase;
        envio_TDMA(mi_final_TDMA + ajuste_fase);
        contador_mensajes_TDMA += 1;
      }
    } 
    else if (time_adjusted >= mi_final_TDMA){
      inicio_ronda_TDMA = mi_final_TDMA + ajuste_fase;
      contador_mensajes_TDMA = 0;
      //contador_rondas_TDMA += 1;
      rondas_tdma += 1;
      //if (contador_rondas_TDMA == 3) contador_rondas_TDMA = 0;
    } 
    else {
      pollEspNowQueue(/*max_items=*/3, /*time_budget_us=*/3000);
      ciclo_de_control();
    }
  }
}

void ciclo_de_inicio(){
  PID_theta="MANUAL";
  PID_vel="MANUAL";
  PID_d="MANUAL";
  error_ant_d=0;
  integral_d=0;
  error_ant_vel=0;
  integral_vel=0;
  error_ant_theta=0;
  integral_theta=0;
  Output_d=0;
  Output_vel=0;
  Output_theta=0;
  if (ordenar_lugar == 1) d_ref = distancia();
  sp_vel = 0;
  Input_vel=0;
  vel_ref=0;
  motor(0,0);
  encoder_der.write(0);
  encoder_izq.write(0);
}


void ciclo_de_control(){

  Input_d = distancia();
  if(rango_robot != 1){
    error_d = Input_d - d_ref;
  }
  else{
    error_d = 0;
  }
  velocidades();
  curvatura_pista();
  //control = ((curvatura_predecesor <= 0.01) && (curvatura <= 0.01));      // HABILITA ALGORITMO DE SWITCHEO CURVATURA
  control = 1;                                                              // DESAVILITA ALGORITMO CURVATURA
  if(Output_vel<0){
    Input_theta=-getposition(0)/d2;
  }
  else{
    Input_theta=getposition(1)/d1;
  }
  if ((abs(error_d) >= delta + 1) || rango_robot == 1) {  // cambia a != 0 para poder ordenar bien al seguidor despues de calibrar
    PID_theta="AUTO";
    PID_vel="AUTO";
    if(control){
      PID_d="AUTO";
    }
    else{
      PID_d="MANUAL";
      Output_d=vel_crucero;
      error_ant_d=0;
      integral_d=vel_crucero/Ki_d - error_d*0.04;
    }
    if(rango_robot == 1){
      PID_d="MANUAL"; 
      Output_d = 0;
      error_ant_d=0;
      integral_d=0;
      error_d =0;
    }
  }
  if (abs(Input_vel) <= 6 && abs(Input_theta) <= 0.015 && ((abs(error_d) < delta) && (sp_vel==0 || rango_robot != 1))) {
    PID_d="MANUAL";
    PID_vel="MANUAL";
    PID_theta="MANUAL";
    Output_d = 0;
    Output_vel = 0;
    Output_theta = 0;
    error_ant_d=0;
    integral_d=0;
    error_ant_vel=0;
    integral_vel=0;
    error_ant_theta=0;
    integral_theta=0;
  }
  
  t_actual=millis()-t_controlador;
  if(t_actual >= 40){
    //Output_d=calculoPID(Input_d, d_ref,error_ant_d, integral_d, Kp_d, Ki_d, Kd_d, sat_d, PID_d, Output_d, "INVERSO");     //HABILITAR PARA CONTROLADOR PID SIN COMUNICACION Y SWITCHEO CURVATURA
    Output_d =calculoPIDd(Input_d, d_ref,error_ant_d, integral_d, Kp_d, Ki_d, Kd_d, sat_d, PID_d, Output_d, "INVERSO"); //HABILITA CALCULO DE PID CON SATURACION PREDECESOR
    vel_ref = Output_d *(rango_robot != 1) + (rango_robot == 1)*sp_vel;
    Output_vel = calculoPID(Input_vel, vel_ref, error_ant_vel, integral_vel, Kp_vel, Ki_vel, Kd_vel, sat_vel, PID_vel, Output_vel, "DIRECTO");
    Output_theta = calculoPID(Input_theta, theta_ref, error_ant_theta, integral_theta, Kp_theta, Ki_theta, Kd_theta, sat_theta, PID_theta, Output_theta, "DIRECTO");
    t_controlador = millis();
  }
  motor(Output_vel - Output_theta, Output_vel + Output_theta);
}
