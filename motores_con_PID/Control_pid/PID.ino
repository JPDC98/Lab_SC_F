//Asignación de pines modulos
const int llanta_1 = 6;
const int llanta_2 = 7;
const int encoder_1 = 2;
const int encoder_2 = 3;
const int pot = 0;
//Constantes PID a cambiar
double kp = 1.5;
double kd = 1000;
double ki = 0.00001;
double cm_s_setpoint = 40;

//Asignación de variables a utilizar
volatile unsigned delta_1,delta_2,t_actual_1,t_pasado_1,delta_t_1,t_actual_2,t_pasado_2,delta_t_2 = 0;
volatile unsigned espera_pas_1,espera_act_1,espera_1,espera_pas_2,espera_act_2,espera_2 = 0;
double  pwm_1,pwm_2,cm_s_1,cm_s_2,cont_1,cont_2,velocidad = 0; 
double salida_PID_1,error_1,error_ant_1,integral_1,derivativo_1,total_1 = 0;
double salida_PID_2,error_2,error_ant_2,integral_2,derivativo_2,total_2 = 0;


//Inicio del programa
void setup() {
  pinMode(llanta_1,OUTPUT);
  pinMode(llanta_2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_1),t_encoder_1,RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_2),t_encoder_2,RISING);
  Serial.begin(115200);
}

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
  pwm_1 = calculoPID_1(cm_s_1);
  pwm_2 = calculoPID_2(cm_s_2);
  Serial.print(cm_s_setpoint);
  Serial.print("   ");
  Serial.print(cm_s_1);
  Serial.print("   ");
  Serial.println(cm_s_2);
  analogWrite(llanta_1,pwm_1);
  analogWrite(llanta_2,pwm_2);
}

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
