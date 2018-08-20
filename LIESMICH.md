# gnFellerSwitch
Arduino Bibliothek (Library) für Feller EDIZIOdue Elektroniktaster.

For English Documentation see [README.md](./README.md).

Der Feller Elektroniktaster ist ein äusserst interessantes Produkt um in eine bestehende EDIZIOdue Installation - wie sie in der Schweiz sehr verbreitet sind - einen programmierbaren/intelligenten/hackbaren Taster zu integrieren.

# Informations zum Feller EDIZIOdue Elektroniktaster
Dieses Projekt bezieht sich auf den Elektroniktaster mit der Feller Artikelnummer: 900-3924.FMI oder 900-3928.FMI, https://online-katalog.feller.ch/kat_details.php?fnr=3928-1.F.L.P.61
Bild:![Picture from Feller Homepage](https://online-katalog.feller.ch/pict/FEG/FEG_3928-1.F.L.P.61.PNG "Bild von 900-3928.FMI von Feller")

Um das Produkt zu verstehen, empfiehlt sich die Lektüre der [Applikationsbeschreibung](https://mam.schneider-electric.com/public/SUM_B-AB_Elektroniktaster_94-3924_0702-D.pdf) und des [Datenblatt's](https://online-katalog.feller.ch/kat_produkt_datenblatt.php?fnr=3928-1.F.L.P.61).
## Bezugsquellen
Feller verkauft nicht direkt an Privatpersonen. Ich empfehle den Kontakt zum örtlichen Elektroinstallateur zu suchen. Diese sind meist offen für "interessante" Anforderungen oder neue Produkte und können die Elektroniktaster bestellen.
Ich würde ausschliesslich zur "grossen" Variante 3923-1.F.L.P.61 raten. Dieser bietet 8 Taster und RGB LED's. Die Ersparniss bei Verwendung einer kleineren Version ist marginal. Werden die Tasten nicht benötigt, lassen sie sich auch mit kompatiblen Blenden der zeptrion Schalter abdecken.
# Was ist der Zweck dieser Bibliothek - Wofür brauche ich den Elektroniktaster
Meine Anforderung war es in eine bestehende Umgebung mit Feller EDIZIOdue Schaltern eine intelligente Heimautomatisierung einzubauen. Ich will Lampen steuern (Relais, ZigBee, ...) aber auch Rolladen und Szenen sollen möglich sein. Mir war es wichtig ein flexibles und erweiterbares System zu bauen. Eines das über ein modernes, Netzwerkbasiertes und offenes Protokoll läuft. Bevorzugt MQTT.
Wer lieber ein geschlossenes, einfach zu installierendes System mit Support vom Hersteller wünscht, dem sei z.B. Zeptrion oder [Zeptrion Air](https://www.feller.ch/de/Produktangebot/Licht-und-Storensteuerungssystem/Licht-Storensteuerung-zeptrionAIR) von Feller empfohlen.
Beim Elektroniktaster handelt es sich lediglich um ein _Frontend_. Das heisst der Taster bietet 8 beleuchtete Tasten (sie können in Rot, Grün _oder_ Blau leuchten/blinken). Das _Backend_ gilt es selbst zu bauen. Üblicherweise setzt ein Systemintegrator hier sein Backend ein und verkauft das Ganze als eigene Lösung. Aber das können wir auch selbst!
Daher habe ich ein eigenes Board (PCB) entwickelt und produziert welches als _Backend_ für den Elektroniktaster dient. Es ist Arduino-Kompatibel und verwendet denselben Chip wie ein Arduino Uno. Auf der einen Seite kommuniziert es mit dem Feller Elektroniktaster. Auf der anderen Seite hängt es sich in einen RS485 BUS ein, wo es mittels [gnMSUP1](../../../gnMSUP1/) mit einem Master kommunizieren kann. Bei diesem handelt es sich um einen Arduino Mega welcher mehrere RS485 Busse ansteuert und über ein Ethernet-Shield die Daten an einen MQTT Broker weiterreicht. Dahinter steckt dann z.B. ein Raspery Pi mit NodeRed oder OpenHAB. Hier sind keine Grenzen gesetzt...
Diese Bibliothek hier dient nun aber "nur" dazu den Feller Elektroniktaster anzusteuern.
# Ich interessiere mich dafür - wie soll ich vorgehen?
Als erstes empfehle ich folgende Fragen zu beantworten:
* Will ich auf ein System setzen um das ich mich selbst kümmern muss und für das wohl kein Elektroinstallateur unterstützung bieten kann?
* Habe ich bereits ein EDIZIOdue System, oder will ich ein solches verwenden?
* Kann ich in meiner Installation ein Kommunikationskabel verlegen? Siehe auch Installation, Risiken im Disclaymer.
* Habe ich genügend Erfahrung mit Arduino und Elektronik?
Wenn ja, dann könnte dieses Projekt etwas für Dich sein.
Besorge Dir als erstes einen Test-Schalter (siehe Bezugsquellen). Diesen kannst Du bereits mit einem Arduino Uno ansteuern und die Bibliothek testen.
Wenn Du Dir sicher bist, dass die Lösung für Dich passt, kannst Du mich auch kontaktieren. Das PCB kann ich zur Verfügung stellen. Es steht auch unter einer offenen Lizenz.
# Disclaymer
## Feller
Feller, EDIZIOdue und zeptrion sind Markenzeichen der Feller AG, http://www.feller.ch.
Ich stehe in keiner direkten Verbindung mit Feller AG. Über meinen Elektroinstallateur konnte ich einen Kontakt herstellen um an einige hilfreiche Informationen zu gelangen. Diese sind vertraulich und daher nicht hier zu finden. Sie sind jedoch nicht für das Projekt relevant.
## Installation, Risiken
Die Installation des Elektroniktasters erfolgt üblicherweise in denselben Schaltstellen (_Wandsteckdosen_) wie auch die 230V Netzspannung. Diese Installation sollte nur durch einen versierten Fachmann erfolgen. Ideal sind natürlich dedizierte Leerrohre für die Kommunikation. Alternativ verwende ich _KNX-Kabel_ - ein Kabel das explizit für die parallele Installation mit 230V vorgesehen ist.
230V können lebensgefährlich sein. Das Risiko trägt jeder selbst.
## Support
Bitte nimm zur Kenntniss, dass es von mir keinerlei Support oder Garantie geben kann. Wenn Du Dich für eine Lösung mit dieser Bibliothek entscheidest, dann machst Du dies auf eigene Gefahr.
## Lizenz
GnFellerSwitch stands under the MIT License.

Copyright (c) 2018 Andreas Gloor

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
