<h1>Selbstbalancierender Roboter - Projektseite</h1>
code zum testen:

```c
#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=10, consKi=40, consKd=0.4;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  mpu.calcOffsets();

  
  Input = mpu.getAngleX();
  Serial.println("funktioniert noch");
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);  
}

void loop() {
  //MPU6050 Auslesen
  mpu.update();
  Input = mpu.getAngleX();

  myPID.Compute();

  Serial.println(Output);

  delay(10);
}
```

```c








int fgh = 50;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=10, consKi=40, consKd=0.4;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


void setup() {
  Serial.begin(115200);
  
  Input = fgh;
  Serial.println("funktioniert noch");
  Setpoint = 100;
  Serial.println("funktioniert noch");
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);  
}

void loop() {
  //MPU6050 Auslesen
  Serial.println(fgh);


  Input = fgh;

  myPID.Compute();

  Serial.println(Output);

  delay(100);
  fgh = fgh + 1;
}
```

```c










#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=10, consKi=40, consKd=0.4;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Serial.println("funktioniert noch");
  Wire.begin();
  Serial.println("funktioniert noch");
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("funktioniert noch");
  
  Input = mpu.getAngleX();
  Serial.println("funktioniert noch");
  Setpoint = 175;
  Serial.println("funktioniert noch");
  myPID.SetMode(AUTOMATIC);
  Serial.println("funktioniert noch");
}

void loop() {
  //MPU6050 Auslesen
  Serial.println("funktioniert noch");
  mpu.update();
  Serial.println("funktioniert noch");
  Input = mpu.getAngleX();
  Serial.println("funktioniert noch");
  myPID.Compute();
  Serial.println("funktioniert noch");
  Serial.println(Output);
  Serial.println("funktioniert noch");
  delay(10);
}
```
<h3>Von Ben und Martin, 12bc</h3>

<h2>Inhaltsverzeichnis</h2>

<ul style="list-stlye-type:none">
    <li><a href="#einleitung">1. Einleitung</a></li>
    <li><a href="#endergebnis">2. Das Endergebnis</a></li>
    <li><a href="#hardware">3. Hardware</a></li>
    <ul>
        <li><a href="#aufbau">3.1 Aufbau</a></li>
        <li><a href="#bauteile">3.2 Bauteile</a></li>
        <li><a href="#schaltplan">3.3 Schaltplan</a></li>
    </ul>
    <li><a href="#software">2. Software</a></li>
</ul>

<h2 id="einleitung">Einleitung</h2>

Die meisten fehrngesteuerten Fahrzeuge haben drei oder mehr Räder. Daher dachten wir uns, dass es doch interresant wäre ein fahrzeug zu bauen, das mit nur zwei Rädern auskommt. Und ganau das haben wir dann auch gebaut. Im folgenden können sie nachlesen, wie unser Selbstbalcierender Roboter aufgebaut ist und funktioniert. Der lange weg, der zu diesem Fertigen Roboter geführt hat wurde in unseren [Stundenprotokoll](https://github.com/Bnlng/Stundenprotokoll-2.-Halbjahr) dokumentiert.

<h2 id="endergebnis">Das Endergebnis</h2>



<h2 id="hardware">Hardware</h2>

<h3 id="aufbau">Aufbau</h3>


<h3 id="bauteile">Bauteile</h3>

<h4>Arduino</h4>

Arduino ist eine Open Source Physical-Computing-Plattform. Es handelt sich um einen Microcontroller mit mehreren analogen und digitalen Ein und Ausgängen. Dieser ist das Herzstück unseres Projektes und für die Verarbeitung der Steuersignale und später für das Steuern des Gegnerflugzeuges und das Punktezählen zuständig.

<h4>MPU 6050</h4>


<h4>DC-Motor-Treiber</h4>

Hierbei handelt es sich um den BTS7960B. Dieser ermöglicht die Steuerung von zwei Gleichstrommotoren. Also die Änderung von Drehrichtung und Geschwindigkeit.

<h4>DC-Motoren</h4>


<h4>Akkus</h4>


<h4>3D-Gedruckte Komponenten</h4>


<h3 id="schaltplan">Schaltplan</h3>



<h2 id="software">Software</h2>
Sketch überarbeitet:

```c
#include <Wire.h>
#include <MPU6050_light.h>

//Pin Belegung
const int linksVorwaertsPin = 3;
const int linksRueckwaertsPin = 5;
const int rechtsVorwaertsPin = 6;
const int rechtsRueckwaertsPin = 9;

//Kalibrierung
const int schwelle = 0;
const int maxWinkel = 90;

//Kalibrierung der Motoren (nur Werte zwischen 0 und 1)
const float linksVorwaertsKali = 1;
const float linksRueckwaertsKali = 1;
const float rechtsVorwaertsKali = 1;
const float rechtsRueckwaertsKali = 1;

//Zwischenspeicher
int winkel = 0;
int outputWert = 0;

//MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Wire.begin();
  mpu.calcOffsets(); // gyro and accelero
}

void loop() {
  //MPU6050 Auslesen
  mpu.update();
  winkel = mpu.getAngleX();
  
  //Balancieren
  if (abs(winkel) < schwelle || winkel < -1 * maxWinkel || winkel > maxWinkel){
    analogWrite(linksRueckwaertsPin, 0);
    analogWrite(rechtsRueckwaertsPin, 0);
    analogWrite(linksVorwaertsPin, 0);
    analogWrite(rechtsVorwaertsPin, 0);
  }
  else if (winkel < -1 * schwelle){
    outputWert = map(abs(winkel), 0, maxWinkel, 0, 155);
    analogWrite(linksRueckwaertsPin, outputWert * linksRueckwaertsKali + 100);
    analogWrite(rechtsRueckwaertsPin, outputWert * rechtsRueckwaertsKali +100);
  }
  else if (winkel > schwelle){
    outputWert = map(winkel, 0, maxWinkel, 0, 155);
    analogWrite(linksVorwaertsPin, outputWert * linksVorwaertsKali + 100);
    analogWrite(rechtsVorwaertsPin, outputWert * rechtsVorwaertsKali + 100);
  }
  delay(10);
}
```
<details>
    <summary>Gesamter Sketch</summary>
    
```c
#include <Wire.h>
#include <MPU6050_light.h>

//Pin Belegung
const int linksVorwärtsPin = 3;
const int linksRückwärtsPin = 5;
const int rechtsVorwärtsPin = 6;
const int rechtsRückwärtsPin = 9;

//Kalibrierung
const int schwelle = 2;
const int maxWinkel = 90;

//Kalibrierung der Motoren (nur Werte zwischen 0 und 1)
const float linksVorwärtsKali = 1;
const float linksRückwärtsKali = 1;
const float rechtsVorwärtsKali = 1;
const float rechtsRückwärtsKali = 1;

//Zwischenspeicher
int winkel = 0;
int outputWert = 0;

//MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Wire.begin();
  mpu.calcOffsets(); // gyro and accelero
}

void loop() {
  //MPU6050 Auslesen
  mpu.update();
  winkel = mpu.getAngleX();
  
  //Balancieren
  if (abs(winkel) < schwelle || winkel < -1 * maxWinkel || winkel > maxWinkel){
    analogWrite(linksRückwärtsPin, 0);
    analogWrite(rechtsRückwärtsPin, 0);
    analogWrite(linksVorwärtsPin, 0);
    analogWrite(rechtsVorwärtsPin, 0);
  }
  else if (winkel < -1 * schwelle){
    outputWert = map(abs(winkel), 0, maxWinkel, 0, 255);
    analogWrite(linksRückwärtsPin, outputWert * linksRückwärtsKali);
    analogWrite(rechtsRückwärtsPin, outputWert * rechtsRückwärtsKali);
  }
  else if (winkel > schwelle){
    outputWert = map(winkel, 0, maxWinkel, 0, 255);
    analogWrite(linksVorwärtsPin, outputWert * linksVorwärtsKali);
    analogWrite(rechtsVorwärtsPin, outputWert * rechtsVorwärtsKali);
  }
  delay(10);
} 
```
    
```c
    

#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*********Tune these 4 values for your BOT*********/
double setpoint= 177; //set the value when the bot is perpendicular to ground using serial monitor. 
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 10; //Set this first
double Kd = 0.4; //Set this secound
double Ki = 40; //Finally set this 
/******End of values setting*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);

  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

     // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

//Initialise the Motor outpu pins
    pinMode (6, OUTPUT);
    pinMode (9, OUTPUT);
    pinMode (10, OUTPUT);
    pinMode (11, OUTPUT);

//By default turn off both the motors
    analogWrite(6,LOW);
    analogWrite(9,LOW);
    analogWrite(10,LOW);
    analogWrite(11,LOW);
}

void loop() {
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();   
        
        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
               
        if (input>150 && input<200){//If the Bot is falling 
          
        if (output>0) //Falling towards front 
        Forward(); //Rotate the wheels forward 
        else if (output<0) //Falling towards back
        Reverse(); //Rotate the wheels backward 
        }
        else //If Bot not falling
        Stop(); //Hold the wheels still
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

        input = ypr[1] * 180/M_PI + 180;

   }
}

void Forward() //Code to rotate the wheel forward 
{
    analogWrite(6,output);
    analogWrite(9,0);
    analogWrite(10,output);
    analogWrite(11,0);
    Serial.print("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
    analogWrite(6,0);
    analogWrite(9,output*-1);
    analogWrite(10,0);
    analogWrite(11,output*-1); 
    Serial.print("R");
}

void Stop() //Code to stop both the wheels
{
    analogWrite(6,0);
    analogWrite(9,0);
    analogWrite(10,0);
    analogWrite(11,0); 
    Serial.print("S");
}
    
```
    
</details>

<h3>Schritt für Schritt Erklärung</h3>

<h4>1. Programmbibliotheken einbinden</h4>

Zuerst müssen die verwendeten Programmbibliotheken (Libaries) eingebunden werden. In Programmbibliotheken befinden sich kleine Unterprogramme, die aufgerufen werden können. Die <code>Wire.h</code> Biblieothek ermöglicht die Kommunikation mit Geräten, wie dem MP6050. Die <code>MPU6050_light</code> Bibliotek dient dem Auslesen des Gyroskop, durch sie kann der relativ einfach mit einem Befehl der Neigunswinkel ermittelt werden. Außerdem ist Sofware zur Kallibrierung des Gyroskops enthalten.

```c
#include <Wire.h>
#include <MPU6050_light>
```

<h4>2. Pins definieren</h4>

Der nächste Schritt ist die Definition der Pins am Arduino. Bei den Pins für die Motorsteuerung werden dabei Variablen für jede Motorbewegung erstellt, in denen die jeweilige Nummer des Pins gespeichert wird. Auf diese Art kann man auch im Nachhinein Schnell die Pinbelegung ändern ohne im Code herumsuchen zu müssen.

```c
//Pin Belegung
const int linksVorwärtsPin = 3;
const int linksRückwärtsPin = 5;
const int rechtsVorwärtsPin = 6;
const int rechtsRückwärtsPin = 9;
```

<h4>3. Kalibrieungskonstanten</h4>

Als nächstes müssen die Kalibrierungskonstanten festgelegt werden. <code>schwelle</code> legt die neigung in Grad fest, die der Roboter überschreiten muss, ehe der Roboter eine ausgleichsbewegung ausführt. Wenn der Winkel <code>maxWinkel</code> überschritten wird versucht der Roboter nicht mehr die neigung Auszugleichen, da er es sowieso nicht mehr schaffen würde, wodurch sich die Räder nicht permanent drehen wenn er umfällt. <code>const</code> bedeuted, dass diese Variable im weiteren Code nicht verändert werden kann. <code>int</code> ist eine abkürzung für integer und legt den Datentyp fest, bei <code>int</code> bedeutet es, dass nur ganze Zahlen in dieser Variable gespeichert werden können. 

Da die beiden Motoren bei gleicher Spannung nicht gleich schnell drehen und vorwärts sowie rückwarts zusätzlich noch einmal unterschiedlich schnell drehen müssen die veschiedenen Motoren und Bewegungsrichtungen individuel gedrosselt werden. Dazu dienen die Konstanden <code>linksVorwärtsKali</code>, <code>linksRückwärtsKali</code>, <code>rechtsVorwärtsKali</code>, <code>rechtsRückwärtsKali</code>

```c
//Kalibrierung
const int schwelle = 2;
const int maxWinkel = 90;

//Kalibrierung der Motoren (nur Werte zwischen 0 und 1)
const float linksVorwärtsKali = 1;
const float linksRückwärtsKali = 1;
const float rechtsVorwärtsKali = 1;
const float rechtsRückwärtsKali = 1;
```

<h4>4. Zwischenspeicher</h4>

Hier werden die Variablen erstellt, in denen nachher im loop Werte zwischengespeichert werden. 

```c
//Zwischenspeicher
int winkel = 0;
int outputWert = 0;
```
<h4>5. Setup</h4>

Dieser Teil des Sketches wird nur ein einziges Mal beim Starten des Arduinos ausgeführt.

```c    
```
