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
//Pin Belegung
const int linksVorwärtsPin = 3;
const int linksRückwärtsPin = 5;
const int rechtsVorwärtsPin = 6;
const int rechtsRückwärtsPin = 9;

const int Multiplikator = 5;
const int Schwelle = 1;

int Winkel = 0;

void setup() {
  
}

void loop() {
  if (Winkel < 90 || Winkel > 90){
  }
  else if (Winkel < 0 - Schwelle){
    analogWrite(linksRückwärtsPin, -1 * Winkel * Multiplikator);
    analogWrite(rechtsRückwärtsPin, -1 * Winkel * Multiplikator);
  }
  else if (Winkel > Schwelle){
    analogWrite(linksRückwärtsPin, Winkel * Multiplikator);
    analogWrite(rechtsRückwärtsPin, Winkel * Multiplikator);
  }

  delay(10);
}
```
```c
//Pin Belegung
const int linksVorwärtsPin = 3;
const int rechtsVorwärtsPin = 6;
const int linksRückwärtsPin = 5;
const int rechtsRückwärtsPin = 9;

//Kalibrierung
const int schwelle = 2;
const int maxWinkel = 90;

//Zwischenspeicher
int winkel = 0;
int outputWert = 0;

void setup() {
  
}

void loop() {

  //Balancieren
  if (abs(winkel) < schwelle || winkel < -1 * maxWinkel || winkel > maxWinkel){
    analogWrite(linksRückwärtsPin, 0);
    analogWrite(rechtsRückwärtsPin, 0);
    analogWrite(linksVorwärtsPin, 0);
    analogWrite(rechtsVorwärtsPin, 0);
  }
  else if (winkel < -1 * schwelle){
    outputWert = map(abs(winkel), 0, maxWinkel, 0, 255);
    analogWrite(linksRückwärtsPin, outputWert);
    analogWrite(rechtsRückwärtsPin, outputWert);
  }
  else if (winkel > schwelle){
    outputWert = map(winkel, 0, maxWinkel, 0, 255);
    analogWrite(linksVorwärtsPin, outputWert);
    analogWrite(rechtsVorwärtsPin, outputWert);
  }
  delay(10);
}
```
