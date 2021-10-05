
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
double velocidad0, velocidad1=0;//velocidad de llantas
unsigned long tiempoPasado=0; //variables para la obtencion del tiempo
unsigned int totalRanuras=200;//cantidad de ranuras del encoder----¡¡¡¡¡¡¡¡¡Cambiar con la cantidad real de ranuras del encoder!!!!!!!!!
const int diametroRueda=64;//mm

double velocidadMaxima=15;//m/s ¡¡¡¡¡¡Cambiar por el valor real de velocidad en m/s!!!!!!!!!!!!!!!!!

//----------Variable para la medicion de la distancia--------
float distancia=0;
//----------Variables de velocidad de motor------------------

volatile unsigned int velocidadMotor1, velocidadMotor2=0;//  0--255;

//----------Variables de pid---------------------------------

double kp=2;
double ki=5;
double kd=1;
//pid 0
unsigned long tiempoPresente0,tiempoAnterior0=0;
double deltaTiempo0,error0,errorAnterior0,entrada0,salida0,setPoint0;
double errorAcumulado0, cambioError0;
//pid 1
unsigned long tiempoPresente1,tiempoAnterior1=0;
double deltaTiempo1,error1,errorAnterior1,entrada1,salida1,setPoint1;
double errorAcumulado1, cambioError1;
double salidaPID0,salidaPID1;

//--------Declaracion de funciones---------------------------
void velocidadMotor();
void cambioDeVelocidad(float);
double calculoPID_0(double);
double calculoPID_1(double);
int velocidadAPWM(double);
double pwmAVelocidad(int);

//-----------------------------------------------------------

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
  
  if(millis()-tiempoPasado>=300){//cada 300ms se calcula la velocidad y se ajusta la velocidad de las llantas
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
      
      //------------Aplicación del PID--------------------------------------------------------------------------------------
      if(velocidad0<velocidad1){
         setPoint1=velocidad0;//Cambia el setpoint de la llanta 2 
         salidaPID0=calculoPID_0(velocidad0);//Calcula la velocidad a la cual debe de girar la llanta 1 dependiendo de la velocidad de la llanta actual      
         salidaPID1=calculoPID_1(velocidad1);//Calcula la velocidad a la cual debe de girar la llanta 2 dependiendo de la velocidad de la llanta actual
         // cambio de valor del pwm de las llantas CUANDO LA LLANTA 0 TIENE MENOR VELOCIDAD LA LLANTA 1 TRATA DE IGUALAR DICHA VELOCIDAD PARA EVITAR UN GIRO INESPERADO
         analogWrite(llanta_1,velocidadAPWM(salidaPID0));//Cambia el ciclo de trabajo dependiendo de la salida del pid
         analogWrite(llanta_2,velocidadAPWM(salidaPID1));
        }
      else if(velocidad0>velocidad1){
        setPoint0=velocidad1;//Cambia el setpoint de la llanta 1 
        salidaPID0=calculoPID_0(velocidad0);//Calcula la velocidad a la cual debe de girar la llanta 1 dependiendo de la velocidad de la llanta actual      
        salidaPID1=calculoPID_1(velocidad1);//Calcula la velocidad a la cual debe de girar la llanta 2 dependiendo de la velocidad de la llanta actual
         // cambio de valor del pwm de las llantas CUANDO LA LLANTA 1 TIENE MENOR VELOCIDAD LA LLANTA 0 TRATA DE IGUALAR DICHA VELOCIDAD PARA EVITAR UN GIRO INESPERADO
        analogWrite(llanta_1,velocidadAPWM(salidaPID0));//Cambia el ciclo de trabajo dependiendo de la salida del pid
        analogWrite(llanta_2,velocidadAPWM(salidaPID1));
       }
      else{ 
        salidaPID0=calculoPID_0(velocidad0);//Calcula la velocidad a la cual debe de girar la llanta 1 dependiendo de la velocidad de la llanta actual      
        salidaPID1=calculoPID_1(velocidad1);//Calcula la velocidad a la cual debe de girar la llanta 2 dependiendo de la velocidad de la llanta actual
        analogWrite(llanta_1,velocidadAPWM(salidaPID0));//Cambia el ciclo de trabajo dependiendo de la salida del pid
        analogWrite(llanta_2,velocidadAPWM(salidaPID1));
      }      
      //-------------Termina aplicacion del PID-------------------------------------------------------------------------------------
      
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
      //velocidadMotor1=velocidadMotor2=int((255*((0.5*distancia)+25)/100));//100%--75% Cambio dinamico
      velocidadMotor1=velocidadMotor2=255;//100% de 255 cambio estatico
      
      setPoint0=setPoint1=pwmAVelocidad(velocidadMotor1);
      analogWrite(llanta_1,velocidadMotor1);
      analogWrite(llanta_2,velocidadMotor2);  
      digitalWrite(led,LOW);
      digitalWrite(audio,LOW);    
      }
    else if(distancia>50 && distancia<=100){
        //velocidadMotor1=velocidadMotor2=int((255*((0.5*distancia)+25)/100));//75%--50% cambio dinamico
        velocidadMotor1=velocidadMotor2=191;//75% de 255 cambio estatico
        
        setPoint0=setPoint1=pwmAVelocidad(velocidadMotor1);//coloca el valor del setpoint a utilizar en el control PID para ambos motores
        analogWrite(llanta_1,velocidadMotor1);
        analogWrite(llanta_2,velocidadMotor2);
        digitalWrite(led,HIGH);
        digitalWrite(audio,LOW);  
    }
    else if(distancia>25 && distancia<=50){
        //velocidadMotor1=velocidadMotor2=int((255*((2*distancia)-50)/100));//50%--25% cambio dinamico
        velocidadMotor1=velocidadMotor2=127;//50% de 255 cambio estatico
        
        setPoint0=setPoint1=pwmAVelocidad(velocidadMotor1);//coloca el valor del setpoint a utilizar en el control PID para ambos motores
        analogWrite(llanta_1,velocidadMotor1);
        analogWrite(llanta_2,velocidadMotor2);
        digitalWrite(audio,LOW);
      }
    else if(distancia<=25){//Frena los motores
        velocidadMotor1=velocidadMotor2=0;//0% de 255 cambio estatico directamente
        setPoint0=setPoint1=pwmAVelocidad(velocidadMotor1);//coloca el valor del setpoint a utilizar en el control PID para ambos motores
        analogWrite(llanta_1,velocidadMotor1);
        analogWrite(llanta_2,velocidadMotor2);
        digitalWrite(audio,HIGH);        
        }
    else{
      velocidadMotor1=velocidadMotor2=255;// si no hay algun obstaculo velocidad total
      setPoint0=setPoint1=pwmAVelocidad(velocidadMotor1);//coloca el valor del setpoint a utilizar en el control PID para ambos motores
      analogWrite(llanta_1,velocidadMotor1);
      analogWrite(llanta_2,velocidadMotor2);
      digitalWrite(led,HIGH);
      digitalWrite(audio,LOW);  
      }
  }
  
double calculoPID_0(double in){//ingresa la velocidad de las llantas
    tiempoPresente0=millis();
    deltaTiempo0=(double)(tiempoPresente0 - tiempoAnterior0);
    error0=setPoint0 - in;//calcula el error
    errorAcumulado0+=error0*deltaTiempo0;//Calcula la integral del error;
    cambioError0=(error0-errorAnterior0)/deltaTiempo0;//Calcula la derivada del error

    double out = kp*error0+ki*errorAcumulado0+kd*cambioError0;//Calcula la salida del PID

    errorAnterior0=error0;//para el siguiente calculo
    tiempoAnterior0=tiempoPresente0;//para el siguiente calculo
    return out;
  }

double calculoPID_1(double in){//ingresa la velocidad de las llantas
    tiempoPresente1=millis();
    deltaTiempo1=(double)(tiempoPresente1 - tiempoAnterior1);
    error1=setPoint0 - in;//calcula el error
    errorAcumulado1+=error1*deltaTiempo1;//Calcula la integral del error;
    cambioError1=(error1-errorAnterior1)/deltaTiempo1;//Calcula la derivada del error

    double out = kp*error1+ki*errorAcumulado1+kd*cambioError1;//Calcula la salida del PID

    errorAnterior1=error1;//para el siguiente calculo
    tiempoAnterior1=tiempoPresente1;//para el siguiente calculo
    return out;
  }

int velocidadAPWM(double vel){
    return int((vel*255)/velocidadMaxima);//calcula cuanto del ciclo de trabajo equivale la velocidad actual del la rueda sabiendo la velocidad maxima que puede alcanzar
  }

double pwmAVelocidad(int valpwm){
    return double((valpwm*velocidadMaxima)/255);//calcula la velocidad para el setpoint dependiendo del valor del pwm   
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
