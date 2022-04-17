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
Bei unserem diesjährigen Informatikprojekt handelt es sich abermals um ein Physical-Computing Projekt. Das Ergebnis ist ein 44cm Breiter und 9Kg schwerer Roboter, welcher auf seinen zwei 61,5 cm großen Räder graziel aufrecht balanciert. Dank den zwei leistungsstarken Lithium-Polymer-Akkus und zwei drehmomentstarken Bürstenmotoren verfügt unser Roboter über eine hohe Reichweite und üppige Leistungsreserven, welche auch den Transport von Zusätzlicher Last denkbar machen. Außerdem kann die Riemenspannung und der parallellauf der der Zahnriemenräder genau eingestellt wereden, um ein geräuch- und verschleißarmes Arbeiten zu garantieren. Die 2,4 GHz Pistolengriff-Fernsteuerung ermöglicht eine leichte aber dennoch präzise Steuerung auch in schwierigem Gelände. 

![Informatik 2  Halbjahr v24](https://user-images.githubusercontent.com/88385986/163733852-6afc4107-2454-4833-b59b-043851cddc3e.png)






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



<h2 id="software">Software</h2>

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
