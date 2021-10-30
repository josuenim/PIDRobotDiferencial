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

double        outMin   = 0.0, outMax     = 0.0; 
double        Input1    = 0.0, Setpoint1   = 0.0;  
double        ITerm1    = 0.0, lastInput1 = 0.0;
double        error1    = 0.0; 
double        lastError = 0.0;
double        rateError = 0.0;
                                
double        cvR    = 0.0, cvL        = 0.0;   
byte          pwm2  = 0  , pwm1      = 0;
double        uR = 0.0,    uL = 0.0;  //  Velocidad Lineal motores

//////// VARIABLES PARA CALCULAR VELOCIDADES ANGULARES /////////
double w1Ref = 0;//Varible a la que le asignamos desde el monitor serial.
double wR = 0; // Varible que guardara  los calculos velocidad angular motor derecho
double w2Ref = 0; 
double wL = 0;
double vW = 0.0;
double vR    = 0.0;
double        kpW       = 0.0, kiW         = 0.0,   kdW = 0.0; 
    
unsigned int ppr = 1890; // Número de muescas que tiene el disco del encoder.
const double alfa = 0.95;
double constValue = 3.1733  ; // (1000*2*pi)/R ---> R = 1980 Resolucion encoder cuadruple
//const double valueConst1 = 0.10631, wheelDistance = 0.195;
const double ConstValue2 = 3.15;

////////////////////// VARIABLE ULTRASONICO ////////////////////

int TRIG = A1;      // 
int ECO = A2;      // 
int LED = A4;      // 
int BUZ = A3;
int duracion;
int distancia;

void setup()
{
  Serial.begin(9600);
  ///////////Ultrasonico ////////////////////
  pinMode(TRIG, OUTPUT);   // trigger como salida
  pinMode(ECO, INPUT);    // echo como entrada
  pinMode(LED, OUTPUT);   // LED como salida
  pinMode(BUZ, OUTPUT);   // LED como salida
  //////////Control De Motores////////////////
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


  /////////////////// valores para el control del PID //////////////////////////////////
  outMax =  115.0;                      // Límite máximo del controlador PID %75.
  outMin = -outMax;                     // Límite mínimo del controlador PID.
  sampleTime = 100;                      // Se le asigna el tiempo de muestreo en milisegundos.
  kpW = 16; //15; 16;
  kiW = 0.00015; //Oscila si > 0.5 | 0.00012; 0.00008;
  kdW = 2;//2; Oscila si > 2.3
  Setpoint1 = 0.0;                          
}



void loop() {
  
 
  // ULTRASONICO
  digitalWrite(TRIG, HIGH);       // generacion del pulso a enviar
  delay(1);                       // al pin conectado al trigger
  digitalWrite(TRIG, LOW);        // del sensor
  duracion = pulseIn(ECO, HIGH);  // con funcion pulseIn se espera un pulso  alto en Echo
  distancia = duracion / 58.2;    // distancia medida en centimetros
    if (distancia <= 100 && distancia >= 51){
          w1Ref = 65;//75%
          //w2Ref = 143;
          digitalWrite(LED, HIGH);      // enciende LED
    } 
    else if(distancia <= 50 && distancia >=26)
    {      w1Ref = 45;//50%
           //w2Ref = 96;
           digitalWrite(BUZ,HIGH);//
     }
     else if (distancia <= 25){
              w1Ref = 0;
            digitalWrite(BUZ,LOW);
            digitalWrite(LED, LOW);
              }
    else { 
           w1Ref = 85;//100%
           digitalWrite(LED, LOW);
           digitalWrite(BUZ,LOW);// apagar
           }

  
  ComputePID(); //CALCULOS 

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cálculo velocidades.
void ComputePID(void)
{
  if (millis() - lastTime >= sampleTime){  
    
      noInterrupts(); // Desconectamos la interrupción para que no actué en esta parte del programa.
      wL = constValue*contador_1/(millis()-lastTime);
      wR = ConstValue2*contador_2/(millis()-lastTime);
      lastTime = millis(); 
      Setpoint1 = wL; 
      contador_1 = 0;  
      contador_2 = 0;  
      interrupts(); // Reiniciamos la interrupción
    
     Input1  = alfa*((wR)/2.0)+(1.0-alfa)*lastInput1;
     lastInput1 = Input1;      // Se guarda la posición para convertirla en pasado.  
     //Control Proporcional
     error1  = (Setpoint1 - Input1)  * kpW; 
     //Control Integral
     ITerm1 += (error1 * kiW);
     //Control Derivativo
     rateError = (error1 - lastError)*kdW;
     lastError= error1;

    // Acota el error integral para eliminar el "efecto windup".
    if (ITerm1 > outMax) ITerm1 = outMax; else if (ITerm1 < outMin) ITerm1 = outMin;
    
    vW = error1 + ITerm1 + rateError;      // Suma todos los errores, es la salida del control PID.

    vR = vW;
    if (vR > outMax) vR = outMax; else if (vR < outMin) vR = outMin; // Acota la salida para que el PWM pueda estar entre outMin y outMax.
     
// Guardar Contador de 0 a 255 en variables CvR cvL
    cvR = w1Ref;
    cvL = vR;

//Movimiento de motores Horaria y antihorario
 if (cvR > 0) mov_horario(in2,in1,enA,cvR); else mov_anti_horario(in2,in1,enA,abs(cvR));     
 if (cvL > 0) mov_anti_horario(in3,in4,enB,cvL); else mov_horario(in3,in4,enB,abs(cvL));
  }
  

   Serial.println(wL,3);

   Serial.println(wR,3);   
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
