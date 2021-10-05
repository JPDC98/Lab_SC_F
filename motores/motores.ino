#include <HCSR04.h>

HCSR04 hc(6,7);

//Pines de interrupcion 
int encoder0 = 2;
int encoder1 = 3;

//Pines de las llantas
const int llanta_1=4;
const int llanta_2=5;

//Pines de led y alarma auditiva
const int led=7;
const int audio=8;

//-----------Variables para la medicion de la velocidad lineal y angular------------------
volatile long pulsosEncoder0, pulsosEncoder1=0;//Contador de pulsos
unsigned int rpm0, rpm1=0; //Contador de revoluciones
float velocidad0, velocidad1=0;//velocidad de llantas
unsigned long tiempoPasado,tiempoPresente=0; //variables para la obtencion del tiempo
unsigned int totalRanuras=200;//cantidad de ranuras del encoder----¡¡¡¡¡¡¡¡¡Cambiar con la cantidad real de ranuras del encoder!!!!!!!!!
const int diametroRueda=64;//mm

//----------Variable para la medicion de la distancia----------------------
float distancia=0;
//----------Variables de velocidad de motor--------------------------------
//
unsigned int velocidadMotor1, velocidadMotor2=0;//  0--255;

//--------Declaracion de funciones---------------------------
void velocidadMotor();
void cambioDeVelocidad(float);
//------------------------------------------------------------

void setup() {
 Serial.begin(115200);
 while(!Serial){}
 
 pinMode(llanta_1,OUTPUT);
 pinMode(llanta_2,OUTPUT);
 pinMode(led,OUTPUT);
 pinMode(audio,OUTPUT);
 pinMode(encoder0,INPUT);
 pinMode(encoder1,INPUT);

 attachInterrupt(digitalPinToInterrupt(encoder0),isrEncoder0,RISING);
 attachInterrupt(digitalPinToInterrupt(encoder1),isrEncoder1,RISING);
 ////
 analogWrite(llanta_1,velocidadMotor1);//Prueba para la mitad de la velocidad
 analogWrite(llanta_2,velocidadMotor2);//Prueba para toda la velocidad
 ////
}

void loop() {
  
  if(millis()-tiempoPasado>=1000){//cada segundo se calcula la velocidad y se ajusta la velocidad de las llantas
      noInterrupts();
      /*
      rpm0=(60*1000/totalMuescas)/(millis()-tiempoPasado)*pulsosEncoder0;
      velocidad0=rpm0*3.1416*diametroRueda*60/1000000;
      rpm1=(60*1000/totalMuescas)/(millis()-tiempoPasado)*pulsosEncoder1;
      velocidad1=(rpm1*3.1416*diametroRueda*60/1000000)*(1000/3600);//  m/s
      tiempoPasado=millis(); 
       */     
      velocidadMotor();//Calculo de las revoluciones y la velocidad lineal de las llantas
      distancia=hc.dist();//Obtencion de la distancia
      cambioDeVelocidad(distancia);//Cambia la velocidad de las llantas segun la distancia de algun objeto frente al sensor de distancia
      
      
      //------Impresion de datos por puerto serial para control------------
      Serial.print(millis()/1000);
      Serial.print("     ");
      Serial.print(rpm0,DEC);Serial.print("  ");
      Serial.print(pulsosEncoder0,DEC);Serial.print("  ");
      Serial.println(velocidad0,2);
      //---------------velocidad 1------------------------------------------
      Serial.print(millis()/1000);
      Serial.print("     ");
      Serial.print(rpm1,DEC);Serial.print("  ");
      Serial.print(pulsosEncoder1,DEC);Serial.print("  ");
      Serial.println(velocidad1,2);
      //--------------------------------------------------------------------
      //pulsosEncoder0=pulsosEncoder1=0;
      tiempoPasado=millis();
      interrupts();     
    }
}

void velocidadMotor(){
  rpm0=(60*1000/totalRanuras)/(millis()-tiempoPasado)*pulsosEncoder0;// Calculo de las revolucionespor minuto de la llanta 1
  velocidad0=(rpm0*3.1416*diametroRueda*60/1000000)*(1000/3600);// m/s
  rpm1=(60*1000/totalRanuras)/(millis()-tiempoPasado)*pulsosEncoder1;
  velocidad1=(rpm1*3.1416*diametroRueda*60/1000000)*(1000/3600);//  m/s
  //tiempoPasado=millis();
  pulsosEncoder0=pulsosEncoder1=0;
  //
  }

void cambioDeVelocidad(float distancia){
    if(distancia>100&&distancia<=150){
      velocidadMotor1=velocidadMotor2=int((255*((0.5*distancia)+25)/100));//100%--75%
      analogWrite(llanta_1,velocidadMotor1);
      analogWrite(llanta_2,velocidadMotor2);  
      digitalWrite(led,LOW);
      digitalWrite(audio,LOW);    
      }
    else if(distancia>50 && distancia<=100){
        velocidadMotor1=velocidadMotor2=int((255*((0.5*distancia)+25)/100));//75%--50%
        analogWrite(llanta_1,velocidadMotor1);
        analogWrite(llanta_2,velocidadMotor2);
        digitalWrite(led,HIGH);
        digitalWrite(audio,LOW);  
    }
    else if(distancia>25 && distancia<=50){
        velocidadMotor1=velocidadMotor2=int((255*((2*distancia)-50)/100));//50%--25%
        analogWrite(llanta_1,velocidadMotor1);
        analogWrite(llanta_2,velocidadMotor2);
        digitalWrite(audio,LOW);
      }
    else if(distancia<=25){//Frena los motores
        velocidadMotor1=velocidadMotor2=0;
        analogWrite(llanta_1,velocidadMotor1);
        analogWrite(llanta_2,velocidadMotor2);
        digitalWrite(audio,HIGH);        
        }
    else{
      velocidadMotor1=velocidadMotor2=255;
      analogWrite(llanta_1,velocidadMotor1);
      analogWrite(llanta_2,velocidadMotor2);
      digitalWrite(led,HIGH);
      digitalWrite(audio,LOW);  
      }
  
  }
  
void isrEncoder0(){//contador de pulsos del encoder de la llanta 1
  if(digitalRead(encoder0)){
    pulsosEncoder0++;      
    }
  }

  
void isrEncoder1(){//contador de pulsos del encoder de la llanta 2
  if(digitalRead(encoder1)){
    pulsosEncoder1++;
    }
  }
