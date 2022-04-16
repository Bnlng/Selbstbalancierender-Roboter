<h1>Selbstbalancierender Roboter - Projektseite</h1>

<h3>Von Ben und Martin, 12bc</h3>

<h2>Inhaltsverzeichnis</h2>

<ul style="list-stlye-type:none">
    <li><a href="#1">1. Die Idee</a></li>
    <li><a href="#proj">2. Das Projekt</a></li>
    <li><a href="#hard">3. Hardware</a></li>
    <ul>
        <li><a href="#aufb">3.1 Aufbau</a></li>
        <li><a href="#teil">3.2 Verwendete Bauteile</a></li>
        <li><a href="#schalt">3.3 Schaltplan</a></li>
    </ul>
    <li><a href="#soft">2. Software</a></li>
</ul>

<h2 id="1">Die Idee</h2>


<h2 id="soft">Software</h2>

```c
#include "Wire.h"
#include <MPU6050_light.h>

//Pin Belegung
const int linksVorwärtsPin = 3;
const int linksRückwärtsPin = 5;
const int rechtsVorwärtsPin = 6;
const int rechtsRückwärtsPin = 9;

//Kalibrierung
const int schwelle = 2;
const int maxWinkel = 90;

//Kalibrierung der Motore (nur Werte zwischen 0 und 1)
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
