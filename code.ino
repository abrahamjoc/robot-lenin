 /*
 *  [ Código Fuente: Robot Velocista Lenin ]
 *  [ Fecha: 23 Julio 2013 ]
 *  
 *  [ Equipo: MLGIPAR ]
 *    - Abraham José
 *    - Lihon Fredy
 *    - Oliveros Hector
 */

#include <QTRSensors.h>

#define STANDBY          9           // pin STANDBY del Motor Driver
#define MOTORLEFT_DIR_A  7           // pin 1 de direccion del Motor Izquierdo
#define MOTORLEFT_DIR_B  8           // pin 2 de direccion del Motor Izquierdo
#define MOTORLEFT_PWM    6           // pin PWM del Motor Izquierdo
#define MOTORRIGH_DIR_A  4           // pin 1 de direccion del Motor Derecho
#define MOTORRIGH_DIR_B  3           // pin 2 de direccion del Motor Derecho
#define MOTORRIGH_PWM    5           // pin PWM del Motor Derecho

#define NUM_SENSORS             8    // número de sensores usados
#define NUM_SAMPLES_PER_SENSOR  5    // cantidad de lecturas analogicas que serán leidas por sensor
#define EMITTER_PIN             11   // pin emisor del QTR

#define ENCODERPIN 10                // pin del encoder
#define LEDPIN     13                // número del pin de test
#define BUTTONPIN  2                 // boton externo

// funcion para pulsar el boton, posee un ciclo para esperar que lo pulse, y otro para cuando deje de pulsarlo
#define esperarBoton() while(!digitalRead(BUTTONPIN)); while(digitalRead(BUTTONPIN))

// estructura para los sensores
QTRSensorsAnalog qtra((unsigned char[]) 
  {A0, A1, A2, A3, A4, A5, A6, A7}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

// arreglo para almacenamiento de valores por sensor
unsigned int sensorValues[NUM_SENSORS];

// variables para tiempo, millis()
unsigned long begintime;
unsigned long actualtime;

void setMotorLeft(int value)
{
  if ( value >= 0 )
  {
    // si valor positivo vamos hacia adelante
    digitalWrite(MOTORRIGH_DIR_A,HIGH);
    digitalWrite(MOTORRIGH_DIR_B,LOW);
  }
  else
  {
    // si valor negativo vamos hacia atras
    digitalWrite(MOTORRIGH_DIR_A,LOW);
    digitalWrite(MOTORRIGH_DIR_B,HIGH);
    value *= -1;
  }

  // Setea Velocidad
  analogWrite(MOTORRIGH_PWM,value);
}

void setMotorRigh(int value)
{  
  if ( value >= 0 )
  {
    // si valor positivo vamos hacia adelante
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,LOW);
  }
  else
  {
    // si valor negativo vamos hacia atras
    digitalWrite(MOTORLEFT_DIR_A,LOW);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    value *= -1;
  }    

  // Setea Velocidad
  analogWrite(MOTORLEFT_PWM,value);
}

void setMotors(int left, int righ)
{
  digitalWrite(STANDBY,HIGH);
  setMotorLeft(left);
  setMotorRigh(righ);
}

void setBrake(boolean left, boolean righ, int value)
{
  // pin STAND BY
  digitalWrite(STANDBY,HIGH);

  if ( left )
  {
    // pines LEFT motor
    digitalWrite(MOTORRIGH_DIR_A,HIGH);
    digitalWrite(MOTORRIGH_DIR_B,HIGH);
    analogWrite (MOTORRIGH_PWM, value);
  }

  if ( righ )
  {
    // pines RIGHT motor
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    analogWrite (MOTORLEFT_PWM, value);
  }
}

void imprimirsensores()
{
    Serial.print(" | ");
    Serial.print(sensorValues[0]); Serial.print(" | ");
    Serial.print(sensorValues[1]); Serial.print(" | ");
    Serial.print(sensorValues[2]); Serial.print(" | ");
    Serial.print(sensorValues[3]); Serial.print(" | ");
    Serial.print(sensorValues[4]); Serial.print(" | ");
    Serial.print(sensorValues[5]); Serial.print(" | ");
    Serial.print(sensorValues[6]); Serial.print(" | ");
    Serial.print(sensorValues[7]); Serial.println(" |");
}

void debugMotorStop()
{
    setMotors(0,0);
    while ( !digitalRead(BUTTONPIN) );
}


///////////////////////  SECCION PATRONES  ////////////////////////
#define CANT_P 1                // Cantidad de patrones
#define U_N    500              // Umbral Negro
boolean (*patrones[CANT_P])();  // Arreglo de patrones
int i = 0;                      // Indice patron actual

void avanzarSiguientePatron()
{
  ( i > CANT_P-1 ) ?  i = 0 : i++;
}

boolean detectarPatronActual()
{
  return patrones[i]();
}

void respuestaPatronActual()
{
  switch ( i )
  {
      case 0:
          debugMotorStop();
          break;
  }
  avanzarSiguientePatron();
}

boolean patron_todonegro()
{
  return 
    ( sensorValues[0] < U_N ) && 
    ( sensorValues[1] < U_N ) && 
    ( sensorValues[2] < U_N ) && 
    ( sensorValues[3] < U_N ) && 
    ( sensorValues[4] < U_N ) && 
    ( sensorValues[5] < U_N ) && 
    ( sensorValues[6] < U_N ) && 
    ( sensorValues[7] < U_N );
}

boolean patron_todoblanco()
{
  return 
    sensorValues[0] > U_N && 
    sensorValues[1] > U_N && 
    sensorValues[2] > U_N && 
    sensorValues[3] > U_N && 
    sensorValues[4] > U_N && 
    sensorValues[5] > U_N && 
    sensorValues[6] > U_N && 
    sensorValues[7] > U_N;
}
///////////////////////  FIN SECCION PATRONES  /////////////////////

void setup()
{
  // Señalar patrones en buffer circular
  //patrones[0] = patron_todonegro;

  // Comunicaciones para debug
  //Serial.begin(9600);

  // inicializar pines de salida
  pinMode(LEDPIN          ,OUTPUT);
  pinMode(STANDBY         ,OUTPUT);
  pinMode(MOTORRIGH_DIR_A ,OUTPUT);
  pinMode(MOTORRIGH_DIR_B ,OUTPUT);
  pinMode(MOTORRIGH_PWM   ,OUTPUT);
  pinMode(MOTORLEFT_DIR_A ,OUTPUT);
  pinMode(MOTORLEFT_DIR_B ,OUTPUT);
  pinMode(MOTORLEFT_PWM   ,OUTPUT);
  pinMode(BUTTONPIN       ,INPUT);

  // presiona boton para activar calibracion
  while ( !digitalRead(BUTTONPIN) );

  for ( int i=0; i<50; i++)      // make the calibration take about seconds
  {
    digitalWrite(LEDPIN, HIGH);  // turn on LED (flicker LED during calibration)
    delay(20);
    qtra.calibrate();            // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    digitalWrite(LEDPIN, LOW);   // turn off LED
    delay(20);
  }

  digitalWrite(LEDPIN, LOW);     // turn off LED to indicate we are through with calibration

  // presionar boton para correr el robot
  while ( !digitalRead(BUTTONPIN) );

  // esperar 5 segundos 
  delay(5000);
  
  // mover el robot suave para ganar inercia
  setMotors(100, 100);
  
  // durante 100 centesimas de segundo
  delay(100);  
}

unsigned int position = 0;
int derivative = 0;
int proportional = 0;
int power_difference = 0;

// Ultima Proporcional
int last_proportional;

// Constantes Proporcional y Derivativa
float KP = 0.18;
float KD = 1.56;

// Establecer máxima velocidad en el poder diferencial
int max = 162;


// Constante para Rango de Freno (Range Brake)
int RANGEBRAKE = 3500;

void loop()
{
  // Obtiene la posicion de la linea
  // Aquí no estamos interesados ​​en los valores individuales de cada sensor
  position = qtra.readLine(sensorValues);

  // El término proporcional debe ser 0 cuando estamos en línea
  proportional = ((int)position) - 3500;

/*
  // Si entra en el rango de freno aplicarlo en la direccion de la curva
  if ( proportional <= -RANGEBRAKE )
  {
    setMotorRigh(0);
    setBrake(true,false,255);
    delay(1);
  }
  else if ( proportional >= RANGEBRAKE )
  {
    setMotorLeft(0);
    setBrake(false,true,255);
    delay(1);
  }
*/

  /* DEGBUG */
  //Serial.print(proportional);

  // Calcula el término derivativo (cambio) y el término integral (suma) de la posición
  derivative = proportional - last_proportional;

  // Recordando la última posición
  last_proportional = proportional;

  // Calcula la diferencia entre la potencia de los dos motores [ m1 - m2 ]. 
  // Si es un número positivo, el robot gira a la [ derecha ] 
  // Si es un número negativo, el robot gira a la [ izquierda ]
  //  y la magnitud del número determina el ángulo de giro.
  int power_difference = ( proportional * KP ) + ( derivative * KD );

  // Si la velocidad a ejercer es mayor que la posible tanto positiva como negativa, asignar la maxima permitida
  if ( power_difference > max ) power_difference = max; 
  else if ( power_difference < -max ) power_difference = -max;
  
  // Asignar velocidad calculada en el poder diferencial de los motores
  ( power_difference < 0 ) ? setMotors(max+power_difference, max) : setMotors(max, max-power_difference);

  /* DEGBUG */
  //Serial.print(","); Serial.println(power_difference);
  //imprimirsensores();
  
  /* DETECCION PATRONES */
  //if ( detectarPatronActual() )
      //respuestaPatronActual();
}
