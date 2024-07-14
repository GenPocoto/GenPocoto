#include <SoftwareSerial.h>
#include <DHT11.h>
#include <HCSR04.h>
#include <Wire.h>
#include <RTClib.h>

// Definizione dei pin
#define NEXTION_RX 28
#define NEXTION_TX 29
#define TRIG_PIN 30
#define ECHO_PIN 31

#define ResSx 3
#define ResDx 4
#define Fan 5
#define Pomp 6
#define FLUSSO_PIN 32

#define MAX_VASI 10

const int pinSensori[MAX_VASI] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9};
const int pinValvole[MAX_VASI] = {1, 2, 7, 8, 22, 23, 24, 25, 26, 27};

// Pin RTC DS1302
#define RTC_CLK 10
#define RTC_DAT 11
#define RTC_RST 12

#define DHTTYPE DHT11
DHT11 TempSx(A4);
DHT11 TempDx(A5);
DHT11 TempC(A6);
SoftwareSerial nextionSerial(NEXTION_RX, NEXTION_TX);
RTC_DS1307 rtc;

String messageQueue[10];
int queueHead = 0;
int queueTail = 0;

int tempObiettivo;
int umidita[MAX_VASI];
int umiditaSetpoint[MAX_VASI];
bool valvoleHigh[MAX_VASI];
int numVasi = 4;

bool controlloFlusso = true;

int umiditaMin[MAX_VASI];
int umiditaMax[MAX_VASI];
int livelloMin;
int livelloMax;

const byte ID_UMIDITA_VASO = 0x01;
const byte ID_TEMPERATURA_TARGET = 0x05;
const byte ID_PULSANTE_POMPA = 0x06;
const byte ID_PULSANTE_IRRIGAZIONE_MANUALE = 0x07;
const byte ID_PULSANTE_MANUTENZIONE = 0x08;
const byte ID_NUMERO_VASI = 0x09;
const byte ID_PULSANTE_DISABILITA_FLUSSO = 0x0A;
const byte ID_PULSANTE_TEST_RES_SX = 0x0B;
const byte ID_PULSANTE_TEST_RES_DX = 0x0C;
const byte ID_PULSANTE_TEST_FAN = 0x0D;
const byte ID_PULSANTE_TEST_POMP = 0x0E;
const byte ID_PULSANTE_TEST_VALVOLA = 0x0F;

static boolean headerFound = false;
static int headerIndex = 0;
const byte header[3] = {0x23, 0x02, 0x54};

unsigned long previousMillis = 0;
const long interval = 2000;
unsigned long previousSerialMillis = 0;
const long serialInterval = 2000;
unsigned long lastUpdateTime = 0;
const long updateInterval = 2000;

const int inizioFascia1 = 4;
const int fineFascia1 = 6;
const int inizioFascia2 = 20;
const int fineFascia2 = 22;

bool irrigazioneManuale = false;
bool manutenzione = false;

struct PulsanteTest {
  bool attivo;
  unsigned long startMillis;
};

PulsanteTest testResSx = {false, 0};
PulsanteTest testResDx = {false, 0};
PulsanteTest testFan = {false, 0};
PulsanteTest testPomp = {false, 0};
PulsanteTest testValvola[MAX_VASI] = {{false, 0}};

void setup() {
  Serial.begin(9600);
  nextionSerial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  for (int i = 0; i < MAX_VASI; i++) {
    pinMode(pinValvole[i], OUTPUT);
  }
  pinMode(ResSx, OUTPUT);
  pinMode(ResDx, OUTPUT);
  pinMode(Fan, OUTPUT);
  pinMode(Pomp, OUTPUT);
  pinMode(FLUSSO_PIN, INPUT);


  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // Inizializza l'RTC con la data e ora attuali
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  irrigazioneManuale = false;
  manutenzione = false;

  leggiSetpointDaNextion();
  Serial.println("Setup completato");
}

void loop() {
  Serial.println("Inizio ciclo loop");

  controlloIrrigazione();
  inviaDatiUmidita();

  float livelloAcquaPercentuale = leggiLivelloAcqua();
  inviaLivelloAcquaANextion(livelloAcquaPercentuale);

  leggiDatiDHT11();
  controlloRiscaldamentoRaffreddamento();

  gestisciComunicazioneNextion();
  controlloPompa();
  inviaMessaggiDalBuffer();
  aggiornaNextionSequenzialmente();
  controlloPulsantiTest();

  Serial.println("Fine ciclo loop");
  delay(1000); // Aggiungi un piccolo ritardo per evitare un ciclo eccessivamente rapido
}

int leggiValoreIntDaSerial() {
  int valore = 0;
  for (int i = 0; i < 4; i++) {
    valore += nextionSerial.read() << (i * 8);
  }
  return valore;
}

void leggiSetpointDaNextion() {
  leggiNumeroVasiDaNextion();
  leggiSetpointUmiditaDaNextion();
  leggiTemperaturaObiettivo();
}

void leggiSetpointUmiditaDaNextion() {
  for (int i = 0; i < numVasi; i++) {
    String comando = "get n" + String(i + 1) + ".val";
    nextionSerial.print(comando);
    nextionSerial.write(0xFF);
    nextionSerial.write(0xFF);
    nextionSerial.write(0xFF);
    delay(50);
    if (nextionSerial.available() >= 4) {
      umiditaSetpoint[i] = leggiValoreIntDaSerial();
    }
  }
}

void leggiTemperaturaObiettivo() {
  nextionSerial.print("get n0.val");
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  delay(50);
  if (nextionSerial.available() >= 4) {
    tempObiettivo = leggiValoreIntDaSerial();
  }
  Serial.print("Temperatura obiettivo letta da Nextion: ");
  Serial.println(tempObiettivo);
  aggiungiMessaggioAlBuffer("g0.txt=\"Temperatura obiettivo letta da Nextion: " + String(tempObiettivo) + "\"");
}

void leggiNumeroVasiDaNextion() {
  nextionSerial.print("get n11.val");
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  delay(50);
  if (nextionSerial.available() >= 4) {
    numVasi = leggiValoreIntDaSerial();
    numVasi = constrain(numVasi, 4, MAX_VASI);
  }
  Serial.print("Numero di vasi letto da Nextion: ");
  Serial.println(numVasi);
  aggiungiMessaggioAlBuffer("g0.txt=\"Numero di vasi letto da Nextion: " + String(numVasi) + "\"");
}

void inviaTemperaturaMediaANextion(float tempMedia) {
  String comando = "x8.val=" + String((int)tempMedia);
  aggiungiMessaggioAlBuffer(comando);
}

void inviaUmiditaSetpointANextion() {
  for (int i = 0; i < numVasi; i++) {
    String comando = "n" + String(i + 1) + ".val=" + String(umiditaSetpoint[i]);
    aggiungiMessaggioAlBuffer(comando);
  }
}

void inviaAUmiditaANextion(int sensore, int umidita) {
  String comando = "x" + String(sensore * 2) + ".val=" + String(umidita);
  aggiungiMessaggioAlBuffer(comando);
}

void inviaLivelloAcquaANextion(float livelloAcqua) {
  String comando = "x9.val=" + String((int)livelloAcqua);
  aggiungiMessaggioAlBuffer(comando);
  aggiungiMessaggioAlBuffer("g0.txt=\"Livello acqua inviato a Nextion: " + String(livelloAcqua) + "\"");
}

void inviaSerialToNextion(String message) {
  String comando = "g0.txt=\"" + message + "\"";
  aggiungiMessaggioAlBuffer(comando);
}

void inviaStatoPompaANextion(bool stato) {
  String comando = "bt0.val=" + String(stato ? 1 : 0);
  aggiungiMessaggioAlBuffer(comando);
}

void aggiornaSetpointUmiditaSuNextion() {
  inviaUmiditaSetpointANextion();
  inviaTemperaturaMediaANextion(tempObiettivo);
}

int leggiTemperaturaMedia() {
  int temperaturaSx = TempSx.readTemperature();
  int temperaturaDx = TempDx.readTemperature();
  int temperaturaC = TempC.readTemperature();
  int temperaturaMedia = (temperaturaSx + temperaturaDx + temperaturaC) / 3;
  return temperaturaMedia;
}

void controlloRiscaldamentoRaffreddamento() {
  int temperaturaMedia = leggiTemperaturaMedia();
  int tolleranza = 1;

  if (temperaturaMedia < (tempObiettivo - tolleranza)) {
    digitalWrite(ResSx, HIGH);
    digitalWrite(ResDx, HIGH);
    digitalWrite(Fan, LOW);
    Serial.println("Riscaldamento attivato");
  } else if (temperaturaMedia > (tempObiettivo + tolleranza)) {
    digitalWrite(Fan, HIGH);
    digitalWrite(ResSx, LOW);
    digitalWrite(ResDx, LOW);
    Serial.println("Raffreddamento attivato");
  } else {
    digitalWrite(ResSx, LOW);
    digitalWrite(ResDx, LOW);
    digitalWrite(Fan, LOW);
    Serial.println("Temperatura stabile");
  }
  aggiungiMessaggioAlBuffer("g0.txt=\"Temperatura media: " + String(temperaturaMedia) + "\"");
  inviaTemperaturaMediaANextion(temperaturaMedia);
}

void controlloIrrigazione() {
  for (int i = 0; i < numVasi; i++) {
    umidita[i] = leggiUmidita(pinSensori[i], umiditaMin[i], umiditaMax[i]);
    Serial.print("Umidità vaso ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(umidita[i]);

    if (umidita[i] < umiditaSetpoint[i]) {
      digitalWrite(pinValvole[i], HIGH);
      valvoleHigh[i] = true;
      Serial.println("Mando Acqua in Vaso " + String(i + 1));
    } else {
      digitalWrite(pinValvole[i], LOW);
      valvoleHigh[i] = false;
      Serial.println("Fermo Acqua in Vaso " + String(i + 1));
    }
    inviaAUmiditaANextion(i, umidita[i]);
  }
}

void controlloPompa() {
  bool statoPompa = false;
  for (int i = 0; i < numVasi; i++) {
    if (valvoleHigh[i]) {
      statoPompa = true;
      break;
    }
  }

  bool inFasciaOraria = (oraCorrente() >= inizioFascia1 && oraCorrente() < fineFascia1) ||
                        (oraCorrente() >= inizioFascia2 && oraCorrente() < fineFascia2);

  if ((inFasciaOraria || irrigazioneManuale) && !manutenzione) {
    digitalWrite(Pomp, statoPompa ? HIGH : LOW);
    if (statoPompa && controlloFlusso) {
      delay(10000);
      if (digitalRead(FLUSSO_PIN) == LOW) {
        digitalWrite(Pomp, LOW);
        inviaSerialToNextion("Errore: Nessun flusso d'acqua rilevato!");
        inviaStatoPompaANextion(false);
        Serial.println("Errore: Nessun flusso d'acqua rilevato!");
      }
    }
  } else {
    digitalWrite(Pomp, LOW);
  }

  inviaStatoPompaANextion(digitalRead(Pomp));
  Serial.println(digitalRead(Pomp) ? "Pompa ACCESA" : "Pompa SPENTA");
  aggiungiMessaggioAlBuffer(digitalRead(Pomp) ? "g0.txt=\"Pompa ACCESA\"" : "g0.txt=\"Pompa SPENTA\"");
}

int leggiUmidita(int pin, int minVal, int maxVal) {
  int valore = analogRead(pin);
  int umidita = map(valore, minVal, maxVal, 100, 0);
  return umidita;
}

float leggiLivelloAcqua() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long durata = pulseIn(ECHO_PIN, HIGH);
  float distanza = durata * 0.034 / 2;
  float percentuale = map(distanza, livelloMin, livelloMax, 100, 0);
  percentuale = constrain(percentuale, 0, 100);
  Serial.print("Distanza: ");
  Serial.println(distanza);
  aggiungiMessaggioAlBuffer("g0.txt=\"Distanza: " + String(distanza) + "\"");

  return percentuale;
}

void leggiDatiDHT11() {
  float temperatureSx = TempSx.readTemperature();
  float humiditySx = TempSx.readHumidity();
  float temperatureDx = TempDx.readTemperature();
  float humidityDx = TempDx.readHumidity();
  float temperatureC = TempC.readTemperature();
  float humidityC = TempC.readHumidity();

  Serial.print("Temperatura SX: ");
  Serial.print(temperatureSx);
  Serial.print(" °C\tUmidità SX: ");
  Serial.print(humiditySx);
  Serial.println(" %");
  aggiungiMessaggioAlBuffer("g0.txt=\"Temperatura SX: " + String(temperatureSx) + " °C\tUmidità SX: " + String(humiditySx) + " %\"");

  Serial.print("Temperatura DX: ");
  Serial.print(temperatureDx);
  Serial.print(" °C\tUmidità DX: ");
  Serial.print(humidityDx);
  Serial.println(" %");
  aggiungiMessaggioAlBuffer("g0.txt=\"Temperatura DX: " + String(temperatureDx) + " °C\tUmidità DX: " + String(humidityDx) + " %\"");

  Serial.print("Temperatura C: ");
  Serial.print(temperatureC);
  Serial.print(" °C\tUmidità C: ");
  Serial.print(humidityC);
  Serial.println(" %");
  aggiungiMessaggioAlBuffer("g0.txt=\"Temperatura C: " + String(temperatureC) + " °C\tUmidità C: " + String(humidityC) + " %\"");

  int Temperature = (temperatureSx + temperatureDx + temperatureC) / 3;
  inviaTemperaturaMediaANextion(Temperature);
}

void inviaDatiUmidita() {
  for (int i = 0; i < numVasi; i++) {
    inviaAUmiditaANextion(i, umidita[i]);
  }
  inviaUmiditaSetpointANextion();
}

int oraCorrente() {
  DateTime now = rtc.now();
  return now.hour();
}

void gestisciComunicazioneNextion() {
  while (nextionSerial.available()) {
    byte inByte = nextionSerial.read();
    if (inByte == header[headerIndex]) {
      headerIndex++;
      if (headerIndex >= sizeof(header)) {
        headerFound = true;
        headerIndex = 0;
      }
    } else {
      headerIndex = 0;
    }
  }

  if (headerFound) {
    if (nextionSerial.available() >= 5) {
      byte idComponente = nextionSerial.read();
      int valoreSetpoint = leggiValoreIntDaSerial();
      switch (idComponente) {
        case ID_UMIDITA_VASO:
          aggiornaSetpointUmidita(idComponente - ID_UMIDITA_VASO, valoreSetpoint);
          break;
        case ID_TEMPERATURA_TARGET:
          aggiornaTempTarget(valoreSetpoint);
          break;
        case ID_PULSANTE_POMPA:
          digitalWrite(Pomp, valoreSetpoint);
          break;
        case ID_PULSANTE_IRRIGAZIONE_MANUALE:
          irrigazioneManuale = valoreSetpoint;
          break;
        case ID_PULSANTE_MANUTENZIONE:
          manutenzione = valoreSetpoint;
          break;
        case ID_NUMERO_VASI:
          aggiornaNumeroVasi(valoreSetpoint);
          break;
        case ID_PULSANTE_DISABILITA_FLUSSO:
          controlloFlusso = valoreSetpoint;
          break;
        case ID_PULSANTE_TEST_RES_SX:
          avviaPulsanteTest(&testResSx);
          break;
        case ID_PULSANTE_TEST_RES_DX:
          avviaPulsanteTest(&testResDx);
          break;
        case ID_PULSANTE_TEST_FAN:
          avviaPulsanteTest(&testFan);
          break;
        case ID_PULSANTE_TEST_POMP:
          avviaPulsanteTest(&testPomp);
          break;
        case ID_PULSANTE_TEST_VALVOLA:
          avviaPulsanteTest(&testValvola[idComponente - ID_PULSANTE_TEST_VALVOLA]);
          break;
      }
      headerFound = false;
    }
  }
}

void aggiornaSetpointUmidita(int sensore, int valoreSetpoint) {
  umiditaSetpoint[sensore] = valoreSetpoint;
  inviaUmiditaSetpointANextion();
}

void aggiornaTempTarget(int valoreSetpoint) {
  tempObiettivo = valoreSetpoint;
  inviaTemperaturaMediaANextion(tempObiettivo);
}

void aggiornaNumeroVasi(int valoreSetpoint) {
  numVasi = valoreSetpoint;
  inviaAUmiditaANextion(numVasi, valoreSetpoint);
}

void controlloPulsantiTest() {
  unsigned long currentMillis = millis();
  if (testResSx.attivo && currentMillis - testResSx.startMillis >= 2000) {
    testResSx.attivo = false;
    digitalWrite(ResSx, LOW);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Resistenza Sinistra completato\"");
  }
  if (testResDx.attivo && currentMillis - testResDx.startMillis >= 2000) {
    testResDx.attivo = false;
    digitalWrite(ResDx, LOW);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Resistenza Destra completato\"");
  }
  if (testFan.attivo && currentMillis - testFan.startMillis >= 2000) {
    testFan.attivo = false;
    digitalWrite(Fan, LOW);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Ventilatore completato\"");
  }
  if (testPomp.attivo && currentMillis - testPomp.startMillis >= 2000) {
    testPomp.attivo = false;
    digitalWrite(Pomp, LOW);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Pompa completato\"");
  }
  for (int i = 0; i < MAX_VASI; i++) {
    if (testValvola[i].attivo && currentMillis - testValvola[i].startMillis >= 2000) {
      testValvola[i].attivo = false;
      digitalWrite(pinValvole[i], LOW);
      aggiungiMessaggioAlBuffer("g0.txt=\"Test Valvola " + String(i + 1) + " completato\"");
    }
  }
}

void avviaPulsanteTest(PulsanteTest* test) {
  test->attivo = true;
  test->startMillis = millis();
  if (test == &testResSx) {
    digitalWrite(ResSx, HIGH);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Resistenza Sinistra avviato\"");
  } else if (test == &testResDx) {
    digitalWrite(ResDx, HIGH);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Resistenza Destra avviato\"");
  } else if (test == &testFan) {
    digitalWrite(Fan, HIGH);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Ventilatore avviato\"");
  } else if (test == &testPomp) {
    digitalWrite(Pomp, HIGH);
    aggiungiMessaggioAlBuffer("g0.txt=\"Test Pompa avviato\"");
  } else {
    for (int i = 0; i < MAX_VASI; i++) {
      if (test == &testValvola[i]) {
        digitalWrite(pinValvole[i], HIGH);
        aggiungiMessaggioAlBuffer("g0.txt=\"Test Valvola " + String(i + 1) + " avviato\"");
        break;
      }
    }
  }
}

void aggiungiMessaggioAlBuffer(String messaggio) {
  messageQueue[queueTail] = messaggio;
  queueTail = (queueTail + 1) % 10;
  if (queueTail == queueHead) {
    queueHead = (queueHead + 1) % 10;
  }
}

void inviaMessaggiDalBuffer() {
  if (queueHead != queueTail && millis() - previousSerialMillis >= serialInterval) {
    String messaggio = messageQueue[queueHead];
    for (int i = 0; i < messaggio.length(); i++) {
      nextionSerial.write(messaggio[i]);
    }
    nextionSerial.write(0xFF);
    nextionSerial.write(0xFF);
    nextionSerial.write(0xFF);
    queueHead = (queueHead + 1) % 10;
    previousSerialMillis = millis();
  }
}

void aggiornaNextionSequenzialmente() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateTime >= updateInterval) {
    inviaDatiUmidita();
    lastUpdateTime = currentMillis;
  }
}
