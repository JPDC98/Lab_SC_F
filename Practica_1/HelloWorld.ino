
//----------------------Librerias-----------------------
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//----------------------Variables-----------------------
const int disparo = 51;
const int eco = 50;
long duracion;
int distancia;
//-------------------Configuracines---------------------
LiquidCrystal_I2C lcd(0x27,16,2);

void setup()
{
//------------------Inicializaciones--------------------
  pinMode(disparo,OUTPUT);
  pinMode(eco,INPUT);
  lcd.init();                      
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Distancia Objeto");
}
void loop()
{
  digitalWrite(disparo, LOW);
  delayMicroseconds(2);

  digitalWrite(disparo, HIGH);
  delayMicroseconds(10);
  digitalWrite(disparo, LOW);

  duracion = pulseIn(eco, HIGH);
  distancia = (duracion*0.034)/2;
  lcd.setCursor(0,1);
  lcd.print(distancia);
  lcd.print(" cm");
  delay(10);
}
