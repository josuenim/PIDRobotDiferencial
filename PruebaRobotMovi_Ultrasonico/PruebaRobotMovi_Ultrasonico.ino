//Link Informacion Encoder : http://bit.ly/2Qm6aKW

#include "PinChangeInterrupt.h"

///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";         // String para contener datos entrantes
bool stringComplete = false;
const char separator = ','; 
const int dataLength = 2;
float data[dataLength];

//////////////////////MOTOR DERECHO///////////////////////////////

const byte    C1_1 = 3;                  // Entrada de la señal A del encoder.  (Ojo invertir canales)
const byte    C2_1 = 2;                  // Entrada de la señal B del encoder.
const byte    in1  = 6;                 
const byte    in2  = 7;         
const byte    enA = 10;                

volatile long contador_1 = 0;
volatile byte ant_1      = 0;
volatile byte act_1      = 0;


//////////////////////MOTOR IZQUIERDO///////////////////////////////

const byte    C1_2 = 5;                  // Entrada de la señal A del encoder.
const byte    C2_2 = 4;                  // Entrada de la señal B del encoder.
const byte    in3 = 8;                  
const byte    in4 = 9;                  
const byte    enB = 11;               

volatile long contador_2 = 0;
volatile byte ant_2      = 0;
volatile byte act_2      = 0;

/////////////////////////// CONTROLADOR PID //////////////////
unsigned long lastTime = 0,   sampleTime = 0;  
 
double        u    = 0.0, w   = 0.0;             
double        lastU   = 0.0, lastW = 0.0; 
             
                                
double        cvR    = 0.0, cvL        = 0.0;   
byte          pwm2  = 0  , pwm1      = 0;
double        uR = 0.0,    uL = 0.0;  //  Velocidad Lineal motores

//////// VARIABLES PARA CALCULAR VELOCIDADES ANGULARES /////////
double w1Ref = 0;//Varible a la que le asignamos desde el monitor serial.
double wR = 0; // Varible que guardara  los calculos velocidad angular motor derecho
double w2Ref = 0; 
double wL = 0;
    
unsigned int ppr = 1890; // Número de muescas que tiene el disco del encoder.
const double alfa = 0.7;
double constValue = 3.3243; // (1000*2*pi)/R ---> R = 1980 Resolucion encoder cuadruple
//const double valueConst1 = 0.10631, wheelDistance = 0.195;


////////////////////// VARIABLEs ULTRASONICO /////////

int TRIG = 10;      // trigger en pin 10
int ECO = 9;      // echo en pin 9
int LED = 3;      // LED en pin 3
int duracion;
int distancia;

void setup()
{
  Serial.begin(9600);
  
  pinMode(C1_1, INPUT);
  pinMode(C2_1, INPUT);
  pinMode(C1_2, INPUT);
  pinMode(C2_2, INPUT);
  
  pinMode(in1, OUTPUT);       
  pinMode(in2, OUTPUT);   
  pinMode(in3, OUTPUT);       
  pinMode(in4, OUTPUT);   

  digitalWrite(in1, false);       
  digitalWrite(in2, false);   
  digitalWrite(in3, false);       
  digitalWrite(in4, false);  
  
  
  attachInterrupt(digitalPinToInterrupt(C1_1), encoder_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2_1), encoder_1, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(C1_2), encoder_2, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(C2_2), encoder_2, CHANGE);
  

  sampleTime = 100;                      // Se le asigna el tiempo de muestreo en milisegundos.       

                          
}

void loop() {
  if (stringComplete) 
   {
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0, index).toFloat();
      inputString = inputString.substring(index + 1);
     }
     
     w1Ref = data[0];
     w2Ref = data[1];
    
    inputString = "";
    stringComplete = false;
  }
  Compute();

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cálculo velocidades.
void Compute(void)
{
  if (millis() - lastTime >= sampleTime){  
    
      noInterrupts(); // Desconectamos la interrupción para que no actué en esta parte del programa.
      wL = constValue*contador_1/(millis()-lastTime);
      wR = constValue*contador_2/(millis()-lastTime); 
      lastTime = millis(); 
      contador_1 = 0;  
      contador_2 = 0;  
      interrupts(); // Reiniciamos la interrupción
    
     // u  = alfa*((uR+uL)/2.0)+(1.0-alfa)*lastU;
     // w  = alfa*((uR-uL)/wheelDistance)+(1.0-alfa)*lastW;

    cvR = w1Ref;
    cvL = w2Ref;

//Movimiento de motores Horaria y antihorario
  if (cvR > 0) mov_horario(in2,in1,enA,cvR); else mov_anti_horario(in2,in1,enA,abs(cvR));     
 if (cvL > 0) mov_anti_horario(in3,in4,enB,cvL); else mov_horario(in3,in4,enB,abs(cvL));
  }
    Serial.print(wL,3); Serial.print("   "); Serial.println(wR,3);
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Encoder x4. Cuando se produzca cualquier cambio en el encoder esta parte hará que incremente o decremente el contador.
void encoder_1(void)
{
    ant_1=act_1;                            // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.
    act_1=PIND & 12;                      // Guardamos en 'act' el valor que hay en ese instante en el encoder y hacemos un
                                        // enmascaramiento para aislar los dos únicos bits que utilizamos para esta finalidad.
    if(ant_1==4  && act_1==12) contador_1++;  // Incrementa el contador si el encoder se mueve hacia delante.
    if(ant_1==0  && act_1==4)  contador_1++;
    if(ant_1==8  && act_1==0)  contador_1++;
    if(ant_1==12 && act_1==8)  contador_1++;

           
    if(ant_1==12 && act_1==4)  contador_1--;  // Decrementa el contador si el encoder se mueve hacia atrás.
    if(ant_1==4  && act_1==0)  contador_1--;
    if(ant_1==0  && act_1==8)  contador_1--;
    if(ant_1==8  && act_1==12) contador_1--;
}


// Encoder x4. Cuando se produzca cualquier cambio en el encoder esta parte hará que incremente o decremente el contador.
void encoder_2(void)
{
    ant_2=act_2;                        // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.
    act_2=PIND & 48;                    // Guardamos en 'act' el valor que hay en ese instante en el encoder y hacemos un
                                        // enmascaramiento para aislar los dos únicos bits que utilizamos para esta finalidad.

    if(ant_2==16 && act_2==48)  contador_2++;  // Decrementa el contador si el encoder se mueve hacia atrás.
    if(ant_2==0  && act_2==16)  contador_2++;
    if(ant_2==32 && act_2==0)   contador_2++;
    if(ant_2==48 && act_2==32)  contador_2++;
    
    if(ant_2==48 && act_2==16)  contador_2--;  // Incrementa el contador si el encoder se mueve hacia delante.
    if(ant_2==16 && act_2==0)   contador_2--;
    if(ant_2==0  && act_2==32)  contador_2--;
    if(ant_2==32 && act_2==48)  contador_2--;
 }

 void mov_horario(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin,pwm);
}

void mov_anti_horario(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, HIGH);  
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin,pwm);
}
