<h1>Selbstbalancierender Roboter - Projektseite</h1>

<h3>Von Ben und Martin, 12bc</h3>

<h2>Inhaltsverzeichnis</h2>

<ul style="list-stlye-type:none">
    <li><a href="#einleitung">1. Einleitung</a></li>
    <li><a href="#Das Projekt">2. Das Projekt</a></li>
    <li><a href="#hardware">3. Hardware</a></li>
    <ul>
        <li><a href="#aufbau">3.1 Aufbau</a></li>
        <li><a href="#bauteile">3.2 Bauteile</a></li>
        <li><a href="#schaltplan">3.3 Schaltplan</a></li>
    </ul>
    <li><a href="#software">2. Software</a></li>
</ul>

<h2 id="einleitung">Einleitung</h2>

Die meisten ferngesteuerten Fahrzeuge haben drei oder mehr Räder. Daher dachten wir uns, dass es doch interresant wäre ein fahrzeug zu bauen, welches mit nur zwei Rädern auskommt. Und ganau das haben wir dann auf der Basis eines Arduino Nanos gebaut. Im folgenden können sie nachlesen, wie unser Selbstbalcierender Roboter aufgebaut ist und funktioniert. Der lange Weg, der zu diesem fertigen Roboter geführt hat wurde in unserem [Stundenprotokoll](https://github.com/Bnlng/Stundenprotokoll-2.-Halbjahr) dokumentiert.

<h2 id="Das Projekt">Das Projekt</h2>
Bei unserem diesjährigen Informatikprojekt handelt es sich abermals um ein Physical-Computing Projekt. Das Ergebnis ist ein 44cm Breiter und 9Kg schwerer Roboter, welcher auf seinen zwei 16,5 cm großen Räder graziel aufrecht balanciert. Dank den zwei leistungsstarken Lithium-Polymer-Akkus und zwei drehmomentstarken Bürstenmotoren verfügt unser Roboter über eine hohe Reichweite und üppige Leistungsreserven, welche auch den Transport von Zusätzlicher Last denkbar machen. Außerdem kann die Riemenspannung und der parallellauf der der Zahnriemenräder genau eingestellt wereden, um ein geräuch- und verschleißarmes Arbeiten zu garantieren. Die 2,4 GHz Pistolengriff-Fernsteuerung ermöglicht eine leichte aber dennoch präzise Steuerung auch in schwierigem Gelände. 

![Informatik 2  Halbjahr v28](https://user-images.githubusercontent.com/88385986/163799663-b83478d5-fcf9-4b11-82d3-8f03084c468b.png)







<h2 id="hardware">Hardware</h2>

<h3 id="bauteile">Bauteile</h3>
<h4>MPU6050</h4>
Das MPU6050 Modul kombiniert einen Beschleunigungs- und Lagersensor in nur einem Chip. Durch das Nutzen des 3-Achsen Gyroskops verbunden mit dem 3-Achsen Beschleunigungssensor sind 6 Freiheitsgrade ermittelbar.

<h4>BTS7960</h4>
Bei diesem Bauteil handelt es sich um einen Leistungsstarken H-Brücken Motortreiber für Gleichstrommotoren. Dieses ist in der Lage den angeschlossenen Motor in beide Richtungen rotieren zu lassen, ihn also umzupolen. Der Controller kann Spitzenströmen von bis zu 43A stand halten und kommt so auf eine kurzzeitige Leistungfähigkeit von bis zu 1.150 W .

<h4> Arduino Nano </h4>
Arduino ist eine Open Source Physical-Computing-Plattform. Es handelt sich um einen Microcontroller mit mehreren analogen und digitalen Ein und Ausgängen. Dieser ist das Herzstück unseres Projektes und für die Verarbeitung der Steuersignale und später für das Steuern des Gegnerflugzeuges und das Punktezählen zuständig.

<h4>Fernsteuerung + Receiver</h4>
Bei der Fernsteuerung handelt es sich um eine Pistolengriff-Fernsteuerung für den Modellbau der Marke Reely. Das Modell zeichnet sich vor allem durch seinen Geringen Preis aus. Die Genaue funktion des Ausgangssignals und das Zusammenarbeiten mit dem Arduino wird später noch genauer erläutert.

<h4> LiPo Akkus</h4>

<h4> Motoren</h4>
 
<h4> Riemen + Riemenscheiben</h4>

<h4> Räder</h4>

<h4> Baustahl</h4>

<h3 id="aufbau">Aufbau</h3>




<h4>Arduino</h4>

Arduino ist eine Open Source Physical-Computing-Plattform. Es handelt sich um einen Microcontroller mit mehreren analogen und digitalen Ein und Ausgängen. Dieser ist das Herzstück unseres Projektes und für die Verarbeitung der Steuersignale und später für das Steuern des Gegnerflugzeuges und das Punktezählen zuständig.

<h4>MPU 6050</h4>


<h4>DC-Motor-Treiber</h4>

Hierbei handelt es sich um den BTS7960B. Dieser ermöglicht die Steuerung von zwei Gleichstrommotoren. Also die Änderung von Drehrichtung und Geschwindigkeit.

<h4>DC-Motoren</h4>


<h4>Akkus</h4>


<h4>3D-Gedruckte Komponenten</h4>


<h3 id="schaltplan">Schaltplan</h3>

![Schaltplan](https://user-images.githubusercontent.com/88385986/163812863-497f506b-6a97-4490-a7d8-ff0faf9adc85.png)




<h2 id="software">Software</h2>

<details>
    <summary>Gesamter Sketch</summary>
    
```c
#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

//Variablen um den PID Algorythmus zu Steuern
double Setpoint, Input, Output;

//Parameter zur Feinjustierung des PID Algorythmus
double Kp=10, Ki=40, Kd=0.4;

//Verlinkt die Variablen mit dem PID Algorythmus und gibt die Parameter weiter
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Pin Belegung
const int linksVorwaertsPin = 3;
const int linksRueckwaertsPin = 5;
const int rechtsVorwaertsPin = 6;
const int rechtsRueckwaertsPin = 9;

//MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  //MPU6050 starten
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets(); // gyro and accelero
  
  //PID algorythmus starten
  Input = mpu.getAngleX();
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); 
}

void loop() {
  //MPU6050 Auslesen
  mpu.update();
  
  //Winkel mit dem PID algorythmus zu einem Output wert für die Motoren umrechnen
  Input = mpu.getAngleX();
  myPID.Compute();
  
  //Balancieren
  if (Output > 0){
    analogWrite(linksVorwaertsPin, 0);
    analogWrite(rechtsVorwaertsPin, 0);
    
    analogWrite(linksRueckwaertsPin, Output);
    analogWrite(rechtsRueckwaertsPin, Output);
  }
  else if (Output < 0){
    analogWrite(linksRueckwaertsPin, 0);
    analogWrite(rechtsRueckwaertsPin, 0);
    
    analogWrite(linksVorwaertsPin, -1 * Output);
    analogWrite(rechtsVorwaertsPin, -1 * Output);
  }
  else{
    analogWrite(linksRueckwaertsPin, 0);
    analogWrite(rechtsRueckwaertsPin, 0);
    analogWrite(linksVorwaertsPin, 0);
    analogWrite(rechtsVorwaertsPin, 0);
  }
}
```

</details>

<h3>Schritt für Schritt Erklärung</h3>

<h4>1. Programmbibliotheken einbinden</h4>

```c
#include <Wire.h>
#include <MPU6050_light>
#include <PID_v1.h>
```

Zuerst müssen die verwendeten Programmbibliotheken (Libaries) eingebunden werden. In Programmbibliotheken befinden sich kleine Unterprogramme, die aufgerufen werden können. Die <code>Wire.h</code> Biblieothek ermöglicht die Kommunikation mit Geräten, wie dem MP6050. Die <code>MPU6050_light</code> Bibliotek dient dem Auslesen des Gyroskop, durch sie kann der relativ einfach mit einem Befehl der Neigunswinkel ermittelt werden. Außerdem ist Sofware zur Kallibrierung des Gyroskops enthalten. Die <code>PID_v1.h></code> Bibliothek dient der verwendung eines PID Algorythmusses ([Wikipedia](https://de.wikipedia.org/wiki/Regler#PID-Regler)). Mit der PID Regelung lässt sich ermitteln, was die Motoren tun sollen, wenn der Roboter kippt. Dahinter stehen ausgeklügelte Formeln die eine Überkonpensation verhindern und eine schnelle Rückkehr z

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
