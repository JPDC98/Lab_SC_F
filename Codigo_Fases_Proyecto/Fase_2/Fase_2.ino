//Compontente a utilidar 

//Pines para controlar la velocidad de las llantas
const int llanta_1 = 4;
const int llanta_2 = 5;

//Pines del Ultrasonico
const int TRIG = 22;
const int ECO = 24;
int duracion;
int distancia;

//Variable de muestreo de las llantas
const int divisor = 10;

//
volatile int conteo_0 = 0;
volatile int conteo_1 = 0;

void setup() {
  pinMode(llanta_1, OUTPUT);
  pinMode(llanta_2, OUTPUT);
  pinMode(TRIG,OUTPUT);
  pinMode(ECO,INPUT);
  Serial.begin(9600);
  attachInterrupt(0,interrupcion_1,RISING);
  attachInterrupt(1,interrupcion_2,RISING);
}

void loop() {
  delay(1000/divisor);
  digitalWrite(TRIG,HIGH);
  delay(1);
  digitalWrite(TRIG,LOW);
  duracion = pulseIn(ECO,HIGH);
  distancia = duracion/58.2; 
  Serial.println(distancia);
  Serial.print("RPS_1:");
  Serial.print(conteo_0*divisor);
  Serial.print("  RPS_2:");
  Serial.println(conteo_1*divisor);
  conteo_0=0;
  conteo_1=0;
  if(distancia >= 50 and distancia < 60){
    analogWrite(llanta_1,190);
    analogWrite(llanta_2,190);
  }
  else if(distancia >= 20 and distancia < 50){
    analogWrite(llanta_1,80);
    analogWrite(llanta_2,80);
  }
  else if(distancia >= 0 and distancia < 20){
    analogWrite(llanta_1,0);
    analogWrite(llanta_2,0);
  }
  else{
    analogWrite(llanta_1,255);
    analogWrite(llanta_2,255);
  }
}

void interrupcion_1(){
  conteo_0++;  
}
void interrupcion_2(){
  conteo_1++;  
}
