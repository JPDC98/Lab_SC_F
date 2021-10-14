#include <SoftwareSerial.h>

SoftwareSerial envio(51,50);
//-------------------Asignación de pines modulos----------------
const int llanta_1 = 6;
const int llanta_2 = 7;
const int encoder_1 = 2;
const int encoder_2 = 3;
const int trig = 5;
const int eco_1 = 18;
const int eco_2 = 19;
const int buzzer = 53;
const int led = 52;

//---------------Pines para cambiar giro del motor--------------
const int dir_1_1 = 22;
const int dir_1_2 = 23;
const int dir_2_1 = 24;
const int dir_2_2 = 25;

//-----------------------Constantes PID-------------------------
double kp = 1.5;
double kd = 1000;
double ki = 0.00001;

//-------------Asignación de variables a utilizar---------------

//Variables de captura de tiempo para encoder, sensor de distacia y tiempo de interruciones del PID
volatile unsigned delta_1,delta_2,t_actual_1,t_pasado_1,delta_t_1,t_actual_2,t_pasado_2,delta_t_2 = 0;
volatile unsigned espera_pas_1,espera_act_1,espera_1,espera_pas_2,espera_act_2,espera_2 = 0;
unsigned long t_in,t_out,t_T = 0;
//Variables del sistema PID discreto.
double salida_PID_1,error_1,error_ant_1,integral_1,derivativo_1,total_1 = 0;
double salida_PID_2,error_2,error_ant_2,integral_2,derivativo_2,total_2 = 0;
//Variables de resultados finales de sistemas de control del carro. 
double  pwm_1,pwm_2 = 0; 
int distancia,cm_s_setpoint,cm_s_1,cm_s_2 = 0;


//---------------------Inicio del programa----------------------

//----------------Inicialización de pines y condiciones estaticas-----------
void setup() {
  pinMode(llanta_1,OUTPUT);
  pinMode(llanta_2,OUTPUT);
  pinMode(trig,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(dir_1_1,OUTPUT);
  pinMode(dir_1_2,OUTPUT);
  pinMode(dir_2_1,OUTPUT);
  pinMode(dir_2_2,OUTPUT);
  digitalWrite(dir_1_1,HIGH);
  digitalWrite(dir_1_2,LOW);
  digitalWrite(dir_2_1,HIGH);
  digitalWrite(dir_2_2,LOW);
  analogWrite(trig,127);
  attachInterrupt(digitalPinToInterrupt(encoder_1),t_encoder_1,RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_2),t_encoder_2,RISING);
  attachInterrupt(digitalPinToInterrupt(eco_1),disparo_alto,RISING); 
  attachInterrupt(digitalPinToInterrupt(eco_2),disparo_bajo,FALLING); 
  envio.begin(115200);
 
}
//----------------Control de interrucion de enconder y generadores de distancias----------------
void t_encoder_1(){
  t_actual_1 = millis();
  espera_pas_1 = millis();
  delta_t_1 = t_actual_1 - t_pasado_1;
  t_pasado_1 = t_actual_1;
  cm_s_1 = 1100/delta_t_1;
  if(cm_s_1 > 45){
    cm_s_1 = 45;
  }
}
void t_encoder_2(){
  t_actual_2 = millis();
  espera_pas_2 = millis();
  delta_t_2 = t_actual_2 - t_pasado_2;
  t_pasado_2 = t_actual_2;
  cm_s_2 = 1100/delta_t_2;
  if(cm_s_2 > 45){
    cm_s_2 = 45;
  } 
}
//----------------Controlador de interrupciones del tiempo de la señal eco-------------
void disparo_alto(){
    t_in = micros();
}

void disparo_bajo(){
    t_out = micros();
    t_T = t_out - t_in;
    distancia = 0.017*t_T;
}
//--------------CICLO PRINCIPAL DE TRABAJO-----------------
void loop() {
  espera_act_1 = millis();
  espera_act_2 = millis();
  espera_1 = espera_act_1 - espera_pas_1;
  espera_2 = espera_act_2 - espera_pas_2;
  if (espera_1 > 500){
    cm_s_1 = 0;
  }
  if(espera_2 > 500){
    cm_s_2 = 0;
  }
  if (distancia > 100){
    digitalWrite(buzzer,LOW);
    digitalWrite(led,LOW);
    cm_s_setpoint = min(cm_s_1,cm_s_2);
    pwm_1 = calculoPID_1(cm_s_1);
    pwm_2 = calculoPID_2(cm_s_2);
  }
  else if (distancia > 50 && distancia < 100){
    digitalWrite(buzzer,HIGH);
    digitalWrite(led,LOW);
    cm_s_setpoint = 34;
    pwm_1 = calculoPID_1(cm_s_1);
    pwm_2 = calculoPID_2(cm_s_2);
  }
  else if (distancia > 25 && distancia < 50){
    digitalWrite(buzzer,LOW);
    digitalWrite(led,HIGH);
    cm_s_setpoint = 23;
    pwm_1 = calculoPID_1(cm_s_1);
    pwm_2 = calculoPID_2(cm_s_2);
  }
  else if (distancia < 25){
    digitalWrite(buzzer,LOW);
    digitalWrite(led,LOW);
    cm_s_setpoint = 0;
    pwm_1 = 0;
    pwm_2 = 0;
  }
  envio.write( cm_s_setpoint);
  envio.write( cm_s_1);
  envio.write( cm_s_2);
  envio.write(distancia);
  analogWrite(llanta_1,pwm_1);
  analogWrite(llanta_2,pwm_2);
}

//--------------------CALCULOS DEL PID-------------------
int calculoPID_1(int inpt_1){
  error_1 = cm_s_setpoint - inpt_1;
  integral_1 += error_1*delta_t_1;
  derivativo_1 = (error_1 - error_ant_1)/delta_t_1;
  total_1 = kp*error_1 + kd*derivativo_1 + ki*integral_1;
  error_ant_1 = error_1;
  if(total_1 > 45){
    total_1 = 45;
  }
  if(total_1 < -45){
    total_1 = -45;
  }
  salida_PID_1 = map(total_1,-45,45,0,255);
  return salida_PID_1;
}

int calculoPID_2(int inpt_2){
  error_2 = cm_s_setpoint - inpt_2;
  integral_2 += error_2*delta_t_2;
  derivativo_2 = (error_2 - error_ant_2)/delta_t_2;
  total_2 = kp*error_2 + kd*derivativo_2 + ki*integral_2;
  error_ant_2 = error_2;
  if(total_2 > 45){
    total_2 = 45;
  }
  if(total_2 < -45){
    total_2 = -45;
  }
  salida_PID_2 = map(total_2,-45,45,0,255);
  return salida_PID_2;
}
