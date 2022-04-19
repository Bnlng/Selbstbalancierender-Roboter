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
Bei unserem diesjährigen Informatikprojekt handelt es sich abermals um ein Physical-Computing Projekt. Das Ergebnis ist ein 44cm Breiter und 9Kg schwerer Roboter, welcher auf seinen zwei 16,5 cm großen Räder graziel aufrecht balanciert. Dank den zwei leistungsstarken Lithium-Polymer-Akkus und zwei drehmomentstarken Bürstenmotoren verfügt unser Roboter über eine hohe Reichweite und üppige Leistungsreserven, welche auch den Transport von Zusätzlicher Last denkbar machen. Außerdem kann die Riemenspannung und der parallellauf der der Zahnriemenräder genau eingestellt wereden, um ein geräuch- und verschleißarmes Arbeiten zu garantieren. Die 2,4 GHz Pistolengriff-Fernsteuerung ermöglicht die Steuerung.
<br>
<br>
Video des Roboters in Action:
<br>
<br>

[![IMAGE ALT TEXT HERE](https://media.discordapp.net/attachments/836887245268844594/966081099140853790/Untitled.png?width=1202&height=676)](https://www.youtube.com/watch?v=AILGkgTYkng)




Video des CAD-Modelles:

[![IMAGE ALT TEXT HERE](https://cdn.discordapp.com/attachments/836887245268844594/966054547468464199/Untitled.jpg)](https://youtu.be/wmyFSYbn0NQ)






<h2 id="hardware">Hardware</h2>

<h3 id="bauteile">Bauteile</h3>
<h4>MPU6050</h4>
Das MPU6050 Modul kombiniert einen Beschleunigungs- und Lagersensor in nur einem Chip. Durch das Nutzen des 3-Achsen Gyroskops verbunden mit dem 3-Achsen Beschleunigungssensor sind 6 Freiheitsgrade ermittelbar. Aus diesen drei Freiheitsgeraden kann in Kombination eine sehr genau Lagebestimmung des Chips statt finden. Der Beschleunigungssensor kann hier vergleichbar mit einer Wasserwage benutzt werden. Je neher der Beschleunigungswert der Senkrechten Achse an g (9,81 m/s^2) ist, je näher ist der Chip an der Waagerechten. Mit dem Gyroskop kann die Winkelbeschleunigung gemessen werden, also wie stark sich der Chip um eine der drei Achsen dreht. Aus diesen Daten kann dann die aktuelle Neigung relativ zur Lage die der Chip beim Start des Programms hatte, bestimmt werden.

<h4>BTS7960</h4>
Bei diesem Bauteil handelt es sich um einen Leistungsstarken H-Brücken Motortreiber für Gleichstrommotoren. Dieses ist in der Lage den angeschlossenen Motor in beide Richtungen rotieren zu lassen, ihn also umzupolen. Der Controller kann Spitzenströmen von bis zu 43A stand halten und kommt so auf eine kurzzeitige Leistungfähigkeit von bis zu 1.150 W . Für eine solche H-Brücken schaltung werden vier MOSFETS wie im unteren Bild vereinfacht zu sehen zusammen geschaltet. Zwei der Transistoren werden durchgeschaltet (S1 und S4) um so einen Stromfluss durch den Motor zu ermöglichen. Wenn die anderen zwei Transistoren (S2 und S3) durchgeschaltet werden, fließt der Strom in die andere Richtung, ergo der Motor dreht sich in die andere Richtung. 

![310px-H_bridge svg](https://user-images.githubusercontent.com/88385986/144763248-f9dd9373-5276-48ae-a490-cbb7d415482a.png)

<h4> Arduino Nano </h4>
Arduino ist eine Open Source Physical-Computing-Plattform. Es handelt sich um einen Microcontroller mit mehreren analogen und digitalen Ein und Ausgängen. Dieser ist das Herzstück unseres Projektes und für die Verarbeitung der Signale des Gyruskops und des Receivers und die Ausgabe von Steuersignalen an die Motortreiber zuständig.

<h4>Fernsteuerung + Receiver</h4>
Bei der Fernsteuerung handelt es sich um eine Pistolengriff-Fernsteuerung für den Modellbau der Marke Reely. Das Modell zeichnet sich vor allem durch seinen Geringen Preis aus. Die Genaue funktion des Ausgangssignals und das Zusammenarbeiten mit dem Arduino wird später noch genauer erläutert.

<h4> LiPo Akkus</h4>
Für die Stromversorgung unseres Roboters sind zwei 3-Zellen Lithium Polymer Akkus mit einer Kapazität von jeweils 5000mAh zuständig. In reihe geschaltet würde man so auf eine Spannung von 25V kommen. Allerdings ist die Motorleistung mit nur 12,5V bereits völlig ausreichend, weshalb die Akkus in Pallelschaltung verwendet werden. LiPo-Akkus zeichnen sich durch ihr sehr gutes Gewichts-Kapazitätsverhätnis aus, sowie durch äußerst hohe Entladeströme.


<h4> Motoren</h4>
Die Motoren für unserern Roboter stammen aus der Altmetallkiste einer Gabelstablerwerkstatt. Ursprünglich waren sie als Lüftermotoren gedacht.
Es handelt sich um einfache Bürsten-Gleichstrommotoren.
 
<h4> Riemen + Riemenscheiben</h4>
Die Riemen bzw. Zahnscheiben folgen der Normung HTD15. Sie sind 15mm Breit und speziell für hohe Drehmomente gemacht. Die Riemenscheiben sind 3D-Gedruckt. Die Riemenscheibe am Rad hat 72 Zähne und das am Motor 24. Daraus ergibt sich ein Übersetzungsverhältnis von 1:3. Das Drehmoment am Rad verdreifacht sich, während die Drehzahl um den Faktor 3 kleiner wird.

<h4> Räder</h4>
 Die Felgen der Räder wurden aus PVC selbst an der Drehbank angefertigt. Hierbei wurden an beiden Seiten Lagersitze für die Kugellager eingearbeitet. An der Außenseite ist jeweils eine Aluminiumscheibe fest geschraubt, welche die Vollgummireifen am Runterrutschen hindert. 

<h4> Baustahl</h4>
Der Rahmen wurde Hauptsächlich aus Flachstählen gefertigt. Der einfache Baustahl ist preisgünstig und leicht zu bearbeiten und zu verschweißen. Die Schweißarbeiten wurden mit einem Fülldrahtschweißgerät ausgeführt. Die Räder sind an einer Ø15mm Stahlachse befestigt, an dessen Ende M6 Innengewinde eingearbeitet wurden. Die Schrauben in diesen verhindern ein Abrutschen der Räder. Zwischen Rädausßenseite und SChraubenkopf wurden Axiallager angebracht, um die Reibung zu verringern.

<h3 id="aufbau">Aufbau</h3>


<h3 id="schaltplan">Schaltplan</h3>

![Circuit design Ingenious Blorr _ Tinkercad - Mozilla Firefox 19 04 2022 21_43_18](https://user-images.githubusercontent.com/88385986/164083552-35b890f5-facc-49da-8c51-a5c79b7247e4.png)

<h3> Erklärung der Elektrik: </h3>

Die zwei Motorcontroller werden parallel an den Akku geschaltet. Ein Controller hat jeweils zwei Input Pins. Wenn an dem einen eine Spannung anliegt dreht sich der Motor in die eine Richtung, wenn an dem anderen Input eine Spannung anliegt in die andere Richtung. Die Leistung der Motoren hängt von der höhe der Inputspannung ab (0V-5V).
Wichtig ist, dass der Arduino und die Motorcontroller am gleichen Ground angeschlossen sind, also das gleich 0-Potenzial haben.
<br>
<br>
Das MPU6050 Modul übermittelt die Daten über einen I2C Datenbus. Die Leitungen hierfür sind an den Pins A4 und A5 angeschlossen. Das Modul wird mit 5V Spannung versorgt.
<br> 
<br>
Die Spannungsversorgung für den Arduino wird durch eine 9V Blockbatterie gewährleistet. 
<br> 
<br>
Der 2. Arduino lößt auf recht unübliche Weise das Problem, dass der Arduino Nano nur 2 Pins hat, welche als Interrupt Pins genutzt werden können. Der eine dieser Pins, Pin 2, wird für den MPU6050 genutzt. Sobald dieser eine Änderung wahrnimmt, wird ein Interrupt an den Arduino gesendet. Der zweite Intterrupt Pin, Pin 3, ist für den Throttle Input der Fernbedienung belegt. Wie die Aufnahme des Signals des Receivers funktioniert, wird später noch erklärt. Nun wird noch ein 3. Interrupt-Pin benötigt, um den Receiver Output für die Lenkung aufzunehmen. Diese Arbeit übernimmt der zweit Arduino. Je nach dem ob nach links oder rechts gelenkt wird, wird einer der Outputpins 5 und & auf HIGH gezogen. Diese Outputpins des Arduino Nr.2 sind an den Pins 4 und 5 des main-Arduinos angeschlossen. 



<h2 id="software">Software</h2>

Die Programmiersprache von Arduino basiert auf c++, verfügt aber über zusätzliche befehle, genaueres über die Sprache kann [hier](https://www.arduino.cc/reference/de/) nachgelesen werden. Der gesamte Sketch wird im Folgenden Schritt für Schritt erklärt.

<details>
    <summary>Gesamter Sketch</summary>
    
```c
#include <Wire.h>
#include <MPU6050_light.h>
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

Der Folgende code befindet sich in einer Schleife und wird daher immer wieder ausgeführt, bis der Arduino Ausgeschaltet wird.

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

`mpu.update();` Gibt der MPU6050 Libary den Befehl  zu aktualisieren. `Input = mpu.getAngleX();` liest den aktuellen Winkel des Roboters aus und speichert diesen in der Variable Input, sodass die PID Regelung damit arbeiten kann. `myPID.Compute();` sorgt dann dafür, dass die PID Bibliothek mit dem Eben neu ermittelten Wert einen neuen Outputwert für die Motoren Brechnet.

Dieser Teil gibt die Outputwerte, die eben berechnet worden sind an die Motoren weiter. Die erste `if` Anweisung überprüft, ob Output größer als null ist. Wenn das der Fall ist, dann schaltetet der Arduino die beiden Pins die die Vorwärtsbewegung auf Null und die Pins, die die Motoren rückwärts drehen lassen werden werden mit Output = Intensität eingeschaltet.

Die `else if` Anweisung überprüft, ob Output kleiner als 0 ist. Wenn das der Fall ist, dann werden diesmal die Pins zu Rückwärtsbewegung ausgeschaltet und die für die Vorwärtsbewegung eingeschaltet. Die Intensität ergibt sich dabei aus `-1 * Output`, da Output negativ ist, wir aber einen Positiven Wert brauchen.

<dr>

<dr>

<dr>

Dieser Code funktioniert zwar, allerdings ist der Roboter dabei nicht ganz so stabil wie wir uns es erhofft hatten. Daher haben wir auch mal einen Fertigen Code aus dem Internet getestet, welcher wie sich herausstellte etwas besser funktionierte. Daher haben wir Vor diesen Code für die Vorstellung im Unterricht zu verwenden. Da wir den Roboter auch noch fernsteuern wollten haben wir in den Code aus dem Internet zusätzlich eine Funktion zur Fernsteuerung eingebaut. Hierbei mussten wir feststellen, dass das implementieren von eigenem Code in fremder Software recht kompliziert ist. Trotzdem haben wir es am Ende geschafft.

    <h3> Funktion der Fernsteuerung </h3>
    Die Output Signale der Fernsteuerung sind leider keine einfachen Spannungswerte, sondern sogenannte Interrupt-Signale. Diese Funtionieiren über einen periodischen Spannungsanstieg. Dieser Spannungsanstieg ist, wenn an der Fernbedinung kein Regler betätigt wird, 1,5 ms lang. Für einen Vollen Ausschlag eines Regler steigt diese Zeit auf 2ms und für einen Ausschlag in die andere Rcihtung, sinkt sie auf 1ms. Für die Umrechnung dieser Signale in einfache Zahlenwerte haben wir eine Libary genutzt. Im folgend ist der Code dieser Libary zu sehen.
    ```c
    #include <ServoInput.h>


// Steering Setup
const int SteeringSignalPin = 2;  // MUST be interrupt-capable!
const int SteeringPulseMin = 1000;  // microseconds (us)
const int SteeringPulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

// Throttle Setup
const int ThrottleSignalPin = 3;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

void setup() {
	Serial.begin(115200);

	
	}
}

void loop() {
	Serial.print("RC - ");

	float steeringAngle = 90.0 - steering.getAngle();  // returns 0 - 180, subtracting from 90 to center at "0" and invert for "normal" steering
	Serial.print("Steering: ");
	Serial.print(steeringAngle);
	Serial.print("deg");

	Serial.print(" | ");  // separator

	int throttlePercent = throttle.map(-100, 100);  // remap to a percentage both forward and reverse
	Serial.print("Throttle: ");
	Serial.print(throttlePercent);
	Serial.print("% ");

	if (throttlePercent >= 0) {
		Serial.print("(Forward)");
	}
	else {
		Serial.print("(Reverse)");
	}

	Serial.println();
}
    ```
<details>
    <summary>Code</summary>

```c

#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <ServoInput.h>

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
double setpoint= 174; //set the value when the bot is perpendicular to ground using serial monitor. 
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
const int ThrottleSignalPin = 3;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

float left = 1;
float right = 1;
float right1 = 1;
float left1 = 1;

const int leftIn = 4;
const int rightIn = 5;

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


    pinMode (leftIn, INPUT);
    pinMode (rightIn, INPUT);

//By default turn off both the motors
    analogWrite(6,LOW);
    analogWrite(9,LOW);
    analogWrite(10,LOW);
    analogWrite(11,LOW);

    
}

void loop() {
  
 int throttlePercent = throttle.map(100, -100);

if (throttlePercent > 1) {
  setpoint = map(throttlePercent, 1, 100, 175, 170);
}

if (throttlePercent < -1) {
  setpoint = map(throttlePercent, -1, -100, 175, 185);
}
if (throttlePercent < 1 && throttlePercent > -1) {
  setpoint = 175;
}

   
   if (digitalRead(leftIn) == HIGH) {
     right = 1;
    left = 2;
    right1 = 1;
    left1 = 1;
  }
   
  if (digitalRead(rightIn) == HIGH) {
   left = 1;
     right = 2;
       right1 = 1;
    left1 = 1;
   }


 if (digitalRead(leftIn) == LOW && digitalRead(rightIn) == LOW) {
   left = 1;
     right = 1;
     left1 = 1;
     right1 = 1;
   }
   
   
   
   


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
    analogWrite(6,output*left);
    analogWrite(9,0);
    analogWrite(10,output*right);
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
