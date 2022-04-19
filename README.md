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
<br>
<br>
3D Modell des Roboters:
<br>
<br>

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


<h3 id="schaltplan">Schaltplan</h3>

![Schaltplan](https://user-images.githubusercontent.com/88385986/163812863-497f506b-6a97-4490-a7d8-ff0faf9adc85.png)




<h2 id="software">Software</h2>

Die Programmiersprache von Arduino basiert auf c++, verfügt aber über zusätzliche befehle, genaueres über die Sprache kann [hier](https://www.arduino.cc/reference/de/) nachgelesen werden. Der gesamte Sketch wird im Folgenden Schritt für Schritt erklärt.

<details>
    <summary>Gesamter Sketch</summary>
    
```c
#include <Wire.h>
#include <MPU6050_light.h>
#include <ServoInput.h>
#include <PID_v1.h>

//Variablen für die PID Regelung
double Setpoint, Input, Output;

//Parameter zur Feinjustierung der PID Regelung
double Kp=4, Ki=4.5, Kd=0.01;

//Verlinkt die Variablen mit dem PID Regelung und gibt die Parameter weiter
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Pin Belegung
const int linksVorwaertsPin = 3;
const int linksRueckwaertsPin = 5;
const int rechtsVorwaertsPin = 6;
const int rechtsRueckwaertsPin = 9;

//MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;

//fehrnsteuerung
const int ThrottleSignalPin = 3;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

void setup() {
  //MPU6050 starten
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets();
  
  //PID Regelung starten
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255); 
}

void loop() {
  
  //Fehrnsteuerung
  int throttlePercent = throttle.map(100, -100);

  if (throttlePercent > 10) {
    setpoint = map(throttlePercent, 10, 100, 0, 4);
  }

  if (throttlePercent < -10) {
    setpoint = map(throttlePercent, -10, -100, 0, -4);
  }
  
  //MPU6050 Auslesen
  mpu.update();
  
  //Winkel mit der PID Regelung zu einem Output wert für die Motoren umrechnen
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

<h4>1. Bibliotheken einbinden und Variablen Deklarieren</h4>

Am Anfang des Sketches werden die Bibliotheken eingebunden und die Variablen definiert damit später im Gesamten Code darauf zugegriffen werden kann.

<h4>1.1 Bibliotheken einbinden</h4>

```c
#include <Wire.h>
#include <MPU6050_light.h>
#include <ServoInput.h>
#include <PID_v1.h>
```

Zuerst müssen die verwendeten Programmbibliotheken (Libaries) eingebunden werden. In Programmbibliotheken befinden sich kleine Unterprogramme, die aufgerufen werden können. 
<br>
<br>
`Wire.h` ermöglicht die Kommunikation des Arduinos über I²C ([Wikipedia](https://de.wikipedia.org/wiki/I%C2%B2C)) mit Geräten, wie dem MP6050. Diese Bibliothek ist in der Arduino IDE standartmäßig installiert.
<br>
<br>
Die `MPU6050_light` Bibliotek dient dem Auslesen des Gyroskop, durch sie kann der relativ einfach mit einem Befehl der Neigunswinkel ermittelt werden. Außerdem ist Sofware zur Kallibrierung des Gyroskops enthalten. Sie kann unter https://github.com/rfetick/MPU6050_light runtergeladen werden.
<br>
<br>
Mit `ServoInput.h` lassen sich die Eingabewerte der Fehrnbedienung auslesen. Die Bibliothek kann unter https://github.com/dmadison/ServoInput heruntergeladen werden.
<br>
<br>
Die `PID_v1.h` Bibliothek dient der verwendung einer PID Regelung ([Wikipedia](https://de.wikipedia.org/wiki/Regler#PID-Regler)). PID steht für Proportional, Integral und Derivative (deutsch: Derivative = Differential/Ableitung). Mit dieser Regelung lässt sich ermitteln, was die Motoren tun sollen, wenn der Roboter kippt. Sie funktioniert über folgender Formel: 

![Unbenannt](https://user-images.githubusercontent.com/88386307/163874219-80f2882a-4b0c-49ac-bdf5-f394ae132c90.JPG)

**Proportional:** e(t) ist der Momentane abstand zum Optimum (aufrechter Roboter). Kp, Ki und Kd sind faktoren, die manuell eingestellt werden müssen, mit ihnen lässt sich das ganze feinjustieren.

**Integral:** Kp * e(t) verursacht eine Proportionale reaktion, d.h. dass die doppelte entfernung vom optimalwinkel zu einer doppelt so großen reaktion führt. Ki * Integral von e(t) addiert alle vorherigen Werte von e(t), was dazu führt, dass wenn der Roboter lange in einer Neigungsrichtung bleibt die Ausgleichsreaktion mit der Zeit immer stärker wird.

**Differential:** Der letzte Teil dient als eine art Vorhersage darüber was als nächstes Passiert. Hiermit wird verhindert, dass der Roboter überkompensiert, wass eine Schnelle Oszilliation zur Folge hätte.

Die `PID_v1.h` Bibliothek kann unter https://github.com/br3ttb/Arduino-PID-Library heruntergeladen werden.


<h4>1.2 Variablen für die PID Regelung</h4>

```c
//Variablen für die PID Regelung
double Setpoint, Input, Output;

//Parameter zur Feinjustierung der PID Regelung
double Kp=4, Ki=4.5, Kd=0.01;

//Verlinkt die Variablen mit der PID Regelung und gibt die Parameter weiter
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
```
Hier werden die Variablen und für die PID Regelung erstellt und mit der PID Libary verknüpft.

`double Setpoint, Input, Output;` erstellt Variablen, die später im Code noch gebraucht werden, um die PID Regelung zu verwenden.

`double Kp=10, Ki=40, Kd=0.4;` erstellt die Variablen zur Feinjustierung der PID Steurung und weist ihnen Werte zu. Die Werte, Die für Kp, Ki und Kd eingetragen werdenn mussten experimentell ermittelt werden. Das Experimentelle ermitteln läuft so ab, dass zuerst alle Werte auf Null gesetzt werden und dann Kp in kleinen schritten erhöht wird, biss der Roboter fast von alleine stehen bleibt, aber noch stark hin und her schwankt. Dann erhöht man Ki solange, bis der Roboter von alleine Stehenbleibt, aber noch stark hin und her wakelt. Dann erhöht man Kd langsam, um das starke hin und her wakeln zu unterbinden. Falls es dann noch noch immer nicht ruhig steht, dann dreht man noch etwas weiter an den Werten herum, bis es funktioniert.

`PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);` verlinkt die Variablen mit dem PID Regelung, sodass wenn sie Später geändert werden diese änderung auch in der PID Bibliothek vorgenommen wird. Außerdem werden die eben genannten Parameter an die PID Bibliothek weitergegeben.


<h4>1.3 Pins definieren</h4>

```c
//Pin Belegung
const int linksVorwärtsPin = 3;
const int linksRückwärtsPin = 5;
const int rechtsVorwärtsPin = 6;
const int rechtsRückwärtsPin = 9;
```

Der nächste Schritt ist die Definition der Pins am Arduino. Bei den Pins für die Motorsteuerung werden dabei Variablen für jede Motorbewegung erstellt, in denen die jeweilige Nummer des Pins gespeichert wird. Auf diese Art kann man auch im Nachhinein Schnell die Pinbelegung ändern ohne im Code herumsuchen zu müssen.


<h4>1.4 MPU6050</h4>

```c
//MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;
```

Hier werden die Startvorrausetzungen für die MPU6050 Libary getroffen. `MPU6050 mpu(Wire);` wird benötigt, damit die MPU6050 Libary über die Wire Libary informationen aus dem MPU6050 auslesen kann. Durch `unsigned long timer = 0;` kann mit der Varible timer jederzeit ausgelesen werden wie lange (in Millisekunden) der Arduino schon läuft, das ist für die MPU6050 Libary wichtig

<h4>2. setup()</h4>

```c
void setup() {
  //MPU6050 starten
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets();
  
  //PID Regelung starten
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255); 
}
```

Dieser Teil des Sketches wird nur ein einziges Mal beim Starten des Arduinos ausgeführt. `Wire.begin();` Startet die Wire Libary, die die die datenübertragung durch I²C ermöglicht. `byte status = mpu.begin();` Startet die Libary, mit der der Winkel des Roboters ausgelesen wird und `mpu.calcOffsets();` sorgt dann dafür, dass die Libary das Gyroskop Nullen, sodass der ausgegebene Winkel während der Roboter aufrecht steht Null ist. Unter //PID Regelung starten weitere einstellungen für die PID Regelung festgelegt. `Setpoint = 0;` sorgt dafür, dass der durch die PID Regelung angestrebte Winkel Null grad ist (Roboter Aufrecht). `myPID.SetMode(AUTOMATIC);` stellt den Regler auf Automatisch statt Manuell und `myPID.SetSampleTime(10);` stellt die Zeitintervalle (in ms) ein, in denen die PID Libary Outputwerte berechnet. `myPID.SetOutputLimits(-255, 255);` stellt den Bereich in dem die Outputwerte sind ein. Wir haben -255 bis 255 gewählt, da wir so die Richtung in die sich die Räder drehen sollen mit dem Vorzeichen mitgeteilt bekommen und dann mit dem Betrag des Outputs den Wert für den Motore haben, den wir brauchen

<h4>3. loop()</h4>

```c
void loop() {
  //MPU6050 Auslesen
  mpu.update();
  
  //Winkel mit der PID Regelung zu einem Output Wert für die Motoren umrechnen
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
