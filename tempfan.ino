#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
 
//Definitions
#define FAN 9           // Output Pino do Fan
#define ONE_WIRE_BUS 8  // Input da temperatura 8
#define click 3         //Rotary Encoder Click
#define encoder0PinA  2 //Rotary Encoder Pin A
#define encoder0PinB  4 //Rotary Encoder Pin B
#define CRITICAL 50.0  //Temperatura critaca, se chega a ele ignora o PID e liga o Fan
 
volatile unsigned int encoder0Pos = 0;  //Valor inial do encoder
 
//LiquidCrystal lcd(12, 11, 13, 5,6,7);  //Setando LCD
 
//Setup sensor de temperatura
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 
//Setup PID
double Setpoint, Input, Output;                                             //I/O para o PID
double aggKp=40, aggKi=2, aggKd=10;                                         //original: aggKp=4, aggKi=0.2, aggKd=1, Agressivo em,50,20,20
double consKp=20, consKi=1, consKd=5;                                       //original consKp=1, consKi=0.05, consKd=0.25, Conservativo ,20,10,10
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);  //Inicializando PID
 
//interface
int timeCounter;
void setup()
{  
  // Inicializando porta serial para a leitura da temperatura
  Serial.begin(9600);
  Serial.println("Ligando");
 
  //Setup temp
  sensors.begin();                    //Começando Library
  sensors.requestTemperatures();      // Pegando temperatura
  Input = sensors.getTempCByIndex(0); //Setando Input
  Setpoint = 25;                      //Inicializando temperatura desejada
  encoder0Pos=  25;
 
  //PID Setup
  myPID.SetMode(AUTOMATIC);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;  //adjust the PWM Frequency, note: this changes timing like delay()
 
  //Pinos
  pinMode(FAN, OUTPUT);                   // Output do Fan, 0 ate 255
  pinMode(click, INPUT);                  // Click do encoder
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // ligando o pullup
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // Ligando o pullup
 
  //Set up Interupts
  attachInterrupt(1, clicked, RISING);    // Click button on interrupt 1 - pin 3
  attachInterrupt(0, doEncoder, CHANGE);  // Encoder pin on interrupt 0 - pin 2
  
 
  //interface
  timeCounter=0;
 
  /*Setup LCD 16x2 and display startup message
  lcd.begin(16, 2);
  lcd.print("  Smart   Fan");
  lcd.setCursor(0,1);
  lcd.print("  Ligando ");
  delay(1000);
  lcd.clear();
*/
}
void loop()
{
   timeCounter++;
 
   //Pegando a temperatura para PID
   sensors.requestTemperatures();           //Pedindo a temperatura ao sensor
   Input=sensors.getTempCByIndex(0);        //Pegando a temperatura e atribuindo ao input, em celcius
 
  /*Printando as informaçoes no LCD
  lcd.setCursor(1,0);
  lcd.print("Temp:");
  lcd.print((int)Input);
  lcd.setCursor(9,0);
  lcd.print("RPM:");
  lcd.print((int)Output*4.7059);    //Calculando a regreção, lendo o valor real em RPM
  lcd.setCursor(1,1);
  lcd.print("Set:");
  lcd.print((int)Setpoint);*/
  //Compute PID value
  double gap = abs(Setpoint-Input); //Calculando o Error
  if(gap < 1)
  {  
    //Close to Setpoint, be conservative
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //Far from Setpoint, be aggresive
     myPID.SetTunings(aggKp, aggKi, aggKd);
  } 
  myPID.Compute();
  Serial.print(timeCounter);
  Serial.print(" |TEMP: ");
  Serial.print(Input);
  Serial.print(" |RPM: ");
  Serial.print(Output);
  Serial.print(" |Setpoint:");
  Serial.print(Setpoint);
  Serial.print(" |Gap: ");
  Serial.println(gap);
  //analogWrite(FAN,255);
  //Write PID output to fan if not critical
  if (Input<CRITICAL)
    analogWrite(FAN,Output);
  else
    analogWrite(FAN,255);
}

 
void doEncoder()
{
  //pinA e pinB estao no mesmo estado (alto ou baixo) , rodando para aumentar, se nao esta diminuindo
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
  {
    encoder0Pos++;
  }
  else
  {
    encoder0Pos--;
  }
  Serial.println (encoder0Pos, DEC);  //imprime valor
  Setpoint=encoder0Pos;
}
void clicked()
{
  //lcd.clear();
  //lcd.print("click!");
  delay(1000);
}
