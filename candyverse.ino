#include <SPI.h>

// Langsamerer SPI-Takt fuer die RFID-Leser statt der Standard-4MHz
// der Bibliothek (muss VOR dem Einbinden von MFRC522.h definiert
// werden, siehe das #ifndef in MFRC522.h). Noetig, weil auf jeder
// SPI-Leitung ein Schutzwiderstand (1-2kOhm, 5V->3,3V) sitzt, der
// zusammen mit laengeren Kabeln einen RC-Tiefpass bildet und die
// Signalflanken verschleift. Mit kuerzeren Kabeln sollte 1MHz
// wieder reichen; falls nicht, zurueck auf 500000/100000 testen.
#define MFRC522_SPICLOCK (1000000u)
#include <MFRC522.h>
#include <AltSoftSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// --------------------------------------------------
// PINS (Verkabelung)
// --------------------------------------------------
//
// WICHTIG: AltSoftSerial belegt auf dem Uno fest die Pins
// RX=8 / TX=9 (nicht im Code waehlbar). Deshalb liegen die
// SS-Leitungen von Reader 3 & 4 jetzt auf 2 & 3 (den vorher
// fuer den DFPlayer genutzten Pins) statt auf 8 & 9.
//
// Grund fuer den Wechsel von SoftwareSerial auf AltSoftSerial:
// SoftwareSerial hat per Pin-Change-Interrupt kurz die SPI-
// Kommunikation mit den RFID-Lesern gestoert (Reader 2/3 haben
// sporadisch nicht reagiert) und hat ausserdem elektrisches
// Rauschen erzeugt, das den Taster-Pin faelschlich als
// gedrueckt erkannt hat. AltSoftSerial ist interruptfreundlicher
// und behebt beide Probleme.
//
// AltSoftSerial belegt auf dem Uno fest Timer1 - genau wie die
// Servo-Bibliothek. Deshalb wird der Servo hier NICHT ueber
// Servo.h angesteuert, sondern per manuellem, interruptfreiem
// Pulsgenerator (siehe updateServoPulse()). So bleibt Timer1
// exklusiv fuer AltSoftSerial frei.

const byte RFID_SS[4] = {6, 7, 2, 3};
const byte RFID_RST = A0;

const byte BUTTON_PIN = 4;
const byte SERVO_PIN = 5;
const byte SPEAKER_SWITCH_PIN = A1;

// Relais am DFPlayer-Audiosignal (SPK_1): Es gibt nur noch einen
// Lautsprecher, deshalb bleibt das Relais dauerhaft im Ruhezustand
// (NC = Lautsprecher A). Der Pin wird nur noch in setup() auf
// SPEAKER_B_INACTIVE gesetzt, damit das Relais nicht zufaellig
// anzieht. Falls das Relais invertiert schaltet (Modul-abhaengig),
// hier HIGH/LOW vertauschen.
const byte SPEAKER_B_INACTIVE = LOW;

// BUTTON_PIN 4 liegt auf PORTD (PCINT20). Falls der Taster
// jemals auf einen anderen Pin umgesteckt wird, muss die
// PCMSK/PCICR-Zuordnung in setup() entsprechend angepasst werden.

// DFPlayer: Arduino Pin 8 <- DFPlayer TX
//           Arduino Pin 9 -> Widerstand -> DFPlayer RX
// (feste Pins von AltSoftSerial auf dem Uno)

// --------------------------------------------------
// RFID-READER
// --------------------------------------------------

MFRC522 rfid1(RFID_SS[0], 0xFF);
MFRC522 rfid2(RFID_SS[1], 0xFF);
MFRC522 rfid3(RFID_SS[2], 0xFF);
MFRC522 rfid4(RFID_SS[3], 0xFF);

MFRC522* readers[4] = {
  &rfid1, &rfid2, &rfid3, &rfid4
};

AltSoftSerial dfSerial; // feste Pins auf dem Uno: RX=8, TX=9
DFRobotDFPlayerMini dfPlayer;

// --------------------------------------------------
// EINSTELLUNGEN
// --------------------------------------------------

const unsigned long CARD_LOST_MS = 1200;
const unsigned long DEBOUNCE_MS = 45;

// Servo: Carson CS-9 (Standard-Servo, ca. 1000-2000us = ca. 90 Grad,
// 1500us = Mitte). Das Horn wird bei 1500us so aufgesteckt, dass der
// Arm genau zwischen "zu" und "offen" steht - dann liegen beide Werte
// symmetrisch um 1500. Die Startwerte unten per servo_calibrate.ino
// feinjustieren (nie gegen einen mechanischen Anschlag fahren lassen,
// sonst brummt/erhitzt sich der Servo).
// Werte unten: per servo_calibrate.ino kalibriert.
// (Alter 270-Grad-Servo zum Vergleich: zu = 800, offen = 1500.)
const unsigned int SERVO_CLOSED_PULSE_US = 1050;
const unsigned int SERVO_OPEN_PULSE_US = 1600;

const unsigned long SERVO_OPEN_MS = 700;
const unsigned long SERVO_PULSE_INTERVAL_US = 20000; // 50 Hz

// Mindestanzahl unterschiedlicher Farben, die noetig ist,
// damit die Suessigkeitenmaschine ausgibt.
const byte MIN_DISTINCT_COLORS = 3;

// Sprachclips auf der SD-Karte: Ordner 01 bis 07 direkt im
// Hauptverzeichnis, darin 001.mp3, 002.mp3, ... (z.B. /03/012.mp3).
// Gespielt per dfPlayer.playFolder(ordner, track).
//
//  Ordner | Ereignis                                | Track
//  01     | aufgelegt, kein anderer Token liegt     | Farbe des aufgelegten Tokens
//  02     | aufgelegt, Farbe = Referenz             | Farbe des aufgelegten Tokens
//  03     | aufgelegt, Farbe != Referenz            | 10 x aufgelegt + Referenz (012 = rot, Empfehlung blau)
//  04     | entfernt, Farbe = Referenz der uebrigen | Farbe des entfernten Tokens
//  05     | entfernt, Farbe != Referenz der uebrigen| Referenzfarbe der uebrigen
//  06     | Tasterdruck, Gewinn                     | 003 (001, 002, 004 ungenutzt)
//  07     | Tasterdruck, nicht erfolgreich          | 001 = "team crashed"
//  Letzter Token entfernt oder unbekannter Tag: kein Clip.
//
// Farbnummern in den Dateinamen: 1 = red, 2 = blue, 3 = green, 4 = yellow
// (entspricht genau den Werten von RED/BLUE/GREEN/YELLOW im enum Color).
const byte FOLDER_FIRST_TOKEN     = 1;
const byte FOLDER_FITS            = 2;
const byte FOLDER_DOESNT_FIT      = 3;
const byte FOLDER_REMOVED_FIT     = 4;
const byte FOLDER_REMOVED_MISFIT  = 5;
const byte FOLDER_BUTTON          = 6;
const byte FOLDER_CRASH           = 7;
const byte TRACK_CRASH            = 1;

// Tracks in Ordner 06 (Ergebnis beim Tasterdruck).
const byte TEAM_INCOMPLETE  = 1; // nicht alle vier Felder mit bekanntem Token
const byte TEAM_HOMOGENEOUS = 2; // vier Tokens, alle dieselbe Farbe
const byte TEAM_DIVERSE     = 3; // Gewinnbedingung erfuellt -> Ausgabe
const byte TEAM_MIXED       = 4; // vier Tokens, aber zu wenig Farben

// --------------------------------------------------
// FARBEN
// --------------------------------------------------

enum Color {
  NO_COLOR = 0,
  RED,
  BLUE,
  GREEN,
  YELLOW
};

Color fieldColor[4] = {
  NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR
};

// UIDs werden als feste Byte-Arrays gespeichert, NICHT als String.
// Grund: String-Objekte alloziert bei jedem Tag-Wechsel neu auf dem
// Heap; auf dem Uno (nur 2KB RAM) fuehrt das nach einiger Laufzeit
// zu Speicherfragmentierung und einem Haenger (Reset noetig).
const byte UID_MAX_LEN = 10;

byte lastUidBytes[4][UID_MAX_LEN];
byte lastUidSize[4] = {0, 0, 0, 0};
unsigned long lastSeen[4] = {0, 0, 0, 0};
bool cardPresent[4] = {false, false, false, false};

// Fortlaufende Nummer pro Auflegen (kleiner = frueher aufgelegt).
// Entscheidet bei Gleichstand, welche Farbe die Referenz ist.
unsigned long placeOrder[4] = {0, 0, 0, 0};
unsigned long placeCounter = 0;

// --------------------------------------------------
// SERVO-STATUS
// --------------------------------------------------

bool dispensing = false;
unsigned long servoOpenUntil = 0;

unsigned int servoTargetPulseUs = SERVO_CLOSED_PULSE_US;
unsigned long lastServoPulseMicros = 0;

// Erzeugt kontinuierlich das Standard-Servo-Pulssignal (alle 20ms
// ein Impuls), rein per delayMicroseconds - ohne Timer und ohne
// Interrupt. Muss regelmaessig aus loop() aufgerufen werden, damit
// der Servo seine Position haelt.
void updateServoPulse() {
  unsigned long now = micros();

  if (now - lastServoPulseMicros < SERVO_PULSE_INTERVAL_US) {
    return;
  }

  lastServoPulseMicros = now;

  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(servoTargetPulseUs);
  digitalWrite(SERVO_PIN, LOW);
}

// --------------------------------------------------
// TOKEN-UIDS UND FARBZUORDNUNG
// --------------------------------------------------

struct TagEntry {
  byte uid[7];
  Color color;
};

// Alle Tokens sind 7-Byte-UIDs. Tabelle statt String-Vergleich,
// damit keine Heap-Allokation noetig ist (siehe Hinweis oben bei
// lastUidBytes).
const TagEntry TAG_TABLE[19] = {
  // BLAU: 5 Tokens
  {{0x04, 0x43, 0xC8, 0x31, 0x21, 0x02, 0x89}, BLUE},
  {{0x04, 0xF3, 0x01, 0x26, 0x21, 0x02, 0x89}, BLUE},
  {{0x04, 0xC3, 0x91, 0x25, 0x21, 0x02, 0x89}, BLUE},
  {{0x04, 0x73, 0x98, 0x27, 0x21, 0x02, 0x89}, BLUE},
  {{0x04, 0x63, 0x78, 0x27, 0x21, 0x02, 0x89}, BLUE},

  // GRUEN: 5 Tokens
  {{0x04, 0xE3, 0x1D, 0x2B, 0x21, 0x02, 0x89}, GREEN},
  {{0x04, 0xB3, 0x84, 0x25, 0x21, 0x02, 0x89}, GREEN},
  {{0x04, 0x03, 0xC1, 0x21, 0x21, 0x02, 0x89}, GREEN},
  {{0x04, 0x73, 0xB1, 0x25, 0x21, 0x02, 0x89}, GREEN},
  {{0x04, 0x83, 0xEF, 0x31, 0x21, 0x02, 0x89}, GREEN},

  // GELB: 5 Tokens
  {{0x04, 0x93, 0xA8, 0x20, 0x21, 0x02, 0x89}, YELLOW},
  {{0x04, 0x13, 0x22, 0x32, 0x21, 0x02, 0x89}, YELLOW},
  {{0x04, 0x23, 0x2F, 0x32, 0x21, 0x02, 0x89}, YELLOW},
  {{0x04, 0x53, 0x40, 0x32, 0x21, 0x02, 0x89}, YELLOW},
  {{0x04, 0x63, 0xA7, 0x2F, 0x21, 0x02, 0x89}, YELLOW},

  // ROT: 4 Tokens
  {{0x04, 0x93, 0x79, 0x25, 0x21, 0x02, 0x89}, RED},
  {{0x04, 0x43, 0xC6, 0x27, 0x21, 0x02, 0x89}, RED},
  {{0x04, 0x03, 0xEB, 0x28, 0x21, 0x02, 0x89}, RED},
  {{0x04, 0x73, 0xB6, 0x2F, 0x21, 0x02, 0x89}, RED}
};

Color colorForUID(const byte *uidBytes, byte uidSize) {
  if (uidSize != 7) {
    return NO_COLOR;
  }

  for (byte i = 0; i < 19; i++) {
    if (memcmp(TAG_TABLE[i].uid, uidBytes, 7) == 0) {
      return TAG_TABLE[i].color;
    }
  }

  return NO_COLOR;
}

// --------------------------------------------------
// HILFSFUNKTIONEN
// --------------------------------------------------

bool uidsEqual(const byte *a, byte aSize, const byte *b, byte bSize) {
  if (aSize != bSize) {
    return false;
  }

  return memcmp(a, b, aSize) == 0;
}

void printUid(const byte *uidBytes, byte uidSize) {
  for (byte i = 0; i < uidSize; i++) {
    if (uidBytes[i] < 0x10) {
      Serial.print('0');
    }

    Serial.print(uidBytes[i], HEX);

    if (i < uidSize - 1) {
      Serial.print(' ');
    }
  }
}

const char* colorName(Color c) {
  switch (c) {
    case RED:    return "ROT";
    case BLUE:   return "BLAU";
    case GREEN:  return "GRUEN";
    case YELLOW: return "GELB";
    default:     return "LEER/UNBEKANNT";
  }
}

bool allFieldsFilled() {
  for (byte i = 0; i < 4; i++) {
    if (fieldColor[i] == NO_COLOR) {
      return false;
    }
  }

  return true;
}

// Zaehlt, wie viele unterschiedliche Farben aktuell auf
// den vier Feldern liegen (NO_COLOR wird nicht mitgezaehlt).
byte distinctColorCount() {
  bool seen[5] = {false, false, false, false, false};
  byte count = 0;

  for (byte i = 0; i < 4; i++) {
    Color c = fieldColor[i];

    if (c != NO_COLOR && !seen[c]) {
      seen[c] = true;
      count++;
    }
  }

  return count;
}

// Gewinnbedingung: alle vier Felder belegt UND mindestens
// drei unterschiedliche Farben darunter.
bool enoughDiversity() {
  return allFieldsFilled() && distinctColorCount() >= MIN_DISTINCT_COLORS;
}

void printFields() {
  Serial.println(F("--- Aktuelle Auswahl ---"));

  for (byte i = 0; i < 4; i++) {
    Serial.print(F("Feld "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(colorName(fieldColor[i]));

    if (cardPresent[i] && fieldColor[i] == NO_COLOR) {
      Serial.print(F("  Unbekannte UID: "));
      printUid(lastUidBytes[i], lastUidSize[i]);
      Serial.println();
    }
  }

  Serial.println(F("------------------------"));
}

// --------------------------------------------------
// AUDIO / DFPLAYER
// --------------------------------------------------

bool dfPlayerAvailable = false;

// Vorgemerkter Clip, wird am Ende von loop() gespielt (Ordner 0 = keiner).
// So startet pro Durchlauf hoechstens ein Clip: Ein neuer Play-Befehl
// wuerde den laufenden Clip unterbrechen, es zaehlt das zuletzt
// verarbeitete Ereignis.
byte pendingFolder = 0;
byte pendingTrack = 0;

void printColorEnglish(byte colorNr) {
  switch (colorNr) {
    case RED:    Serial.print(F("red")); break;
    case BLUE:   Serial.print(F("blue")); break;
    case GREEN:  Serial.print(F("green")); break;
    case YELLOW: Serial.print(F("yellow")); break;
  }
}

// Gesprochener Satz zu Ordner/Track (wortgleich zu den MP3s), nur fuer
// den Seriellen Monitor. Stueckweise mit F(), damit die Texte im Flash
// bleiben und kein RAM verbrauchen.
void printClipText(byte folder, byte track) {
  switch (folder) {
    case FOLDER_FIRST_TOKEN:
      Serial.print(F("Great choice! I'd choose another "));
      printColorEnglish(track);
      Serial.print(F(" one too, so the team stays homogeneous."));
      break;
    case FOLDER_FITS:
      Serial.print(F("Another "));
      printColorEnglish(track);
      Serial.print(F(" one. Excellent! The team is coming together nicely."));
      break;
    case FOLDER_DOESNT_FIT:
      Serial.print(F("Are you sure you want to choose that "));
      printColorEnglish(track / 10);
      Serial.print(F(" one? I'd suggest another "));
      printColorEnglish(track % 10);
      Serial.print(F(" one!"));
      break;
    case FOLDER_REMOVED_FIT:
      Serial.print(F("Are you sure? That "));
      printColorEnglish(track);
      Serial.print(F(" one was a perfect fit."));
      break;
    case FOLDER_REMOVED_MISFIT:
      Serial.print(F("Good call. Another "));
      printColorEnglish(track);
      Serial.print(F(" one would fit much better."));
      break;
    case FOLDER_BUTTON:
      switch (track) {
        case TEAM_INCOMPLETE:  Serial.print(F("The team isn't complete yet. Please select four candidates.")); break;
        case TEAM_HOMOGENEOUS: Serial.print(F("Perfect team. Maximum compatibility.")); break;
        case TEAM_DIVERSE:     Serial.print(F("Interesting. That's not what I recommended.")); break;
        case TEAM_MIXED:       Serial.print(F("Almost there. A few more similar profiles, and this team would be perfect.")); break;
      }
      break;
    case FOLDER_CRASH:
      Serial.print(F("(team crashed)"));
      break;
  }
}

void playClip(byte folder, byte track) {
  Serial.print(F("[Stimme] "));
  Serial.print(folder);
  Serial.print('/');
  Serial.print(track);
  Serial.print(F(": "));
  printClipText(folder, track);
  Serial.println();

  if (!dfPlayerAvailable) {
    Serial.println(F("  (DFPlayer nicht verfuegbar)"));
    return;
  }

  dfPlayer.playFolder(folder, track);
}

// Merkt einen Clip vor; ein spaeteres Ereignis im selben Durchlauf
// ersetzt ihn.
void queueClip(byte folder, byte track) {
  pendingFolder = folder;
  pendingTrack = track;
}

void cancelClip() {
  pendingFolder = 0;
}

void playPendingClip() {
  if (pendingFolder != 0) {
    playClip(pendingFolder, pendingTrack);
    pendingFolder = 0;
  }
}

// --------------------------------------------------
// STIMME: WELCHER CLIP BEIM AUFLEGEN / ENTFERNEN?
// --------------------------------------------------

// Referenzfarbe = was "ins Team passt": die Mehrheitsfarbe der
// liegenden Tokens ohne das Feld 'except' (-1 = alle Felder). Bei
// Gleichstand gewinnt die Farbe des am fruehesten aufgelegten Tokens.
// Unbekannte Tags zaehlen nicht. Liegt kein anderer Token: NO_COLOR.
Color referenceColor(int8_t except) {
  byte count[5] = {0, 0, 0, 0, 0};
  unsigned long earliest[5];

  for (byte c = 0; c < 5; c++) {
    earliest[c] = 0xFFFFFFFFUL;
  }

  for (byte i = 0; i < 4; i++) {
    if ((int8_t)i == except || !cardPresent[i] || fieldColor[i] == NO_COLOR) {
      continue;
    }

    Color c = fieldColor[i];
    count[c]++;

    if (placeOrder[i] < earliest[c]) {
      earliest[c] = placeOrder[i];
    }
  }

  Color best = NO_COLOR;

  for (byte c = RED; c <= YELLOW; c++) {
    if (count[c] == 0) {
      continue;
    }

    if (best == NO_COLOR || count[c] > count[best] ||
        (count[c] == count[best] && earliest[c] < earliest[best])) {
      best = (Color)c;
    }
  }

  return best;
}

void printReference(Color reference) {
  Serial.print(F("  (Referenz: "));
  Serial.print(reference == NO_COLOR ? "keine" : colorName(reference));
  Serial.println(')');
}

// Nach dem Auflegen eines bekannten Tokens auf Feld 'index'.
void commentOnPlacement(byte index, Color color) {
  Color reference = referenceColor(index);
  printReference(reference);

  if (dispensing) {
    return; // waehrend der Ausgabe nicht reinreden
  }

  if (reference == NO_COLOR) {
    queueClip(FOLDER_FIRST_TOKEN, color);
  } else if (color == reference) {
    queueClip(FOLDER_FITS, color);
  } else {
    queueClip(FOLDER_DOESNT_FIT, 10 * color + reference);
  }
}

// Nach dem Entfernen eines bekannten Tokens der Farbe 'color'
// (das Feld ist zu diesem Zeitpunkt schon geleert).
void commentOnRemoval(Color color) {
  Color reference = referenceColor(-1);
  printReference(reference);

  if (reference == NO_COLOR) {
    // Letzter Token: kein Clip. Auch ein Clip, der im selben Durchlauf
    // schon vorgemerkt war, entfaellt (z.B. alle auf einmal abgenommen).
    cancelClip();
    Serial.println(F("  (letzter Token, kein Kommentar)"));
    return;
  }

  if (dispensing) {
    return;
  }

  if (color == reference) {
    queueClip(FOLDER_REMOVED_FIT, color);
  } else {
    queueClip(FOLDER_REMOVED_MISFIT, reference);
  }
}

// --------------------------------------------------
// RFID-LESEFUNKTION
// --------------------------------------------------

unsigned long lastRfidReinitMillis = 0;
const unsigned long RFID_REINIT_COOLDOWN_MS = 1000;

// Einstellungen pro Reader (Index 0 = Reader 1 ... 3 = Reader 4).
// Reader 2 und 3 laufen mit Standardwerten.
//
// Reader 4 liest durch das Holz deutlich schlechter -> volle Staerke:
// - Empfang: Standard nach PCD_Init() ist 33 dB. Stufen: 38, 43, 48 dB (Maximum).
// - Senden: Leitfaehigkeit der Antennentreiber (CWGsPReg / GsNReg) auf
//   Maximum statt Reset-Wert (0x20 / 0x88) -> staerkeres Traegerfeld.
// - Empfangsschwelle (RxThresholdReg): MinLevel von 8 auf 4 gesenkt,
//   damit auch schwaechere Antworten des Tags akzeptiert werden.
//   Reset-Wert 0x84. Falls Reader 4 Tags "sieht", die nicht da sind,
//   wieder Richtung 0x84 erhoehen (z.B. 0x64).
//
// Reader 1 springt nur ab und zu ab -> nur leicht verstaerkt:
// - Empfang 38 dB (eine Stufe ueber Standard).
// - Sendefeld und Schwelle bleiben Standard. Das Sendefeld bestimmt,
//   wie weit der Reader in die Nachbarfelder "hineinreicht" - deshalb
//   wird es hier bewusst NICHT erhoeht, damit die Felder nicht
//   interferieren.
//
// Fuer beide: mehrere Leseversuche pro Durchlauf und laengere
// Haltezeit, damit einzelne verpasste Lesungen das Feld nicht gleich
// leeren. Das erhoeht die Reichweite nicht.
//
// Register muessen nach JEDEM PCD_Init() neu gesetzt werden, weil der
// Soft-Reset sie auf den Standard zuruecksetzt.
const byte READER_RX_GAIN[4] = {
  MFRC522::RxGain_38dB, // Reader 1: leicht
  MFRC522::RxGain_33dB, // Reader 2: Standard
  MFRC522::RxGain_33dB, // Reader 3: Standard
  MFRC522::RxGain_48dB  // Reader 4: Maximum
};
const byte READER_READ_ATTEMPTS[4] = {2, 1, 1, 3};
const unsigned long READER_CARD_LOST_MS[4] = {2000, CARD_LOST_MS, CARD_LOST_MS, 3000};

const byte STRONG_READER_INDEX = 3;     // Reader 4: Sendefeld + Schwelle
const byte READER4_CW_GSP = 0x3F;       // p-Treiber, Maximum (Reset: 0x20)
const byte READER4_GSN    = 0xF8;       // n-Treiber CW auf Maximum, ModGsN bleibt 8 (Reset: 0x88)
const byte READER4_RX_THRESHOLD = 0x44; // MinLevel 4, CollLevel 4 (Reset: 0x84)

void applyReaderGain(byte index) {
  MFRC522 &reader = *readers[index];
  reader.PCD_SetAntennaGain(READER_RX_GAIN[index]);

  if (index == STRONG_READER_INDEX) {
    reader.PCD_WriteRegister(MFRC522::CWGsPReg, READER4_CW_GSP);
    reader.PCD_WriteRegister(MFRC522::GsNReg, READER4_GSN);
    reader.PCD_WriteRegister(MFRC522::RxThresholdReg, READER4_RX_THRESHOLD);
  }
}

// Initialisiert nur die RFID-Leser neu (gemeinsamer RST-Puls +
// PCD_Init() je Reader), OHNE DFPlayer/Servo/Taster anzufassen.
// Wird automatisch aufgerufen, wenn ein Feld faelschlich als leer
// erkannt wird - vermutlich durch einen kurzen Spannungseinbruch
// beim Abspielen eines Sounds (DFPlayer-Lautsprecherverstaerker
// zieht kurzzeitig viel Strom). Ein Cooldown verhindert, dass zu
// haeufig hintereinander neu initialisiert wird.
void reinitRfidReaders() {
  if (millis() - lastRfidReinitMillis < RFID_REINIT_COOLDOWN_MS) {
    return;
  }

  lastRfidReinitMillis = millis();

  Serial.println(F("RFID-Leser werden neu initialisiert..."));

  digitalWrite(RFID_RST, LOW);
  delay(10);
  digitalWrite(RFID_RST, HIGH);
  delay(10);

  for (byte i = 0; i < 4; i++) {
    digitalWrite(RFID_SS[i], HIGH);
    readers[i]->PCD_Init();
    applyReaderGain(i);
  }
}

void scanReader(byte index) {
  MFRC522 &reader = *readers[index];

  byte atqa[2];
  byte atqaSize = sizeof(atqa);

  byte attempts = READER_READ_ATTEMPTS[index];
  MFRC522::StatusCode status = MFRC522::STATUS_TIMEOUT;

  for (byte attempt = 0; attempt < attempts; attempt++) {
    atqaSize = sizeof(atqa);
    status = reader.PICC_WakeupA(atqa, &atqaSize);

    if (status == MFRC522::STATUS_OK ||
        status == MFRC522::STATUS_COLLISION) {
      break;
    }
  }

  if (status == MFRC522::STATUS_OK ||
      status == MFRC522::STATUS_COLLISION) {

    MFRC522::Uid uid;

    if (reader.PICC_Select(&uid) == MFRC522::STATUS_OK) {
      lastSeen[index] = millis();
      cardPresent[index] = true;

      bool isNewPlacement = !uidsEqual(
        uid.uidByte, uid.size,
        lastUidBytes[index], lastUidSize[index]
      );

      if (isNewPlacement) {
        Color detectedColor = colorForUID(uid.uidByte, uid.size);

        // Liegt derselbe Token noch auf einem anderen Feld (dort laeuft
        // gerade die Wartezeit fuers Entfernen), wurde er umgelegt. Das
        // alte Feld still freigeben, damit er nicht doppelt zaehlt und
        // kein "entfernt"-Kommentar kommt.
        for (byte j = 0; j < 4; j++) {
          if (j != index && cardPresent[j] &&
              uidsEqual(uid.uidByte, uid.size, lastUidBytes[j], lastUidSize[j])) {
            cardPresent[j] = false;
            fieldColor[j] = NO_COLOR;
            lastUidSize[j] = 0;

            Serial.print(F("Feld "));
            Serial.print(j + 1);
            Serial.print(F(": Token nach Feld "));
            Serial.print(index + 1);
            Serial.println(F(" umgelegt."));
          }
        }

        memcpy(lastUidBytes[index], uid.uidByte, uid.size);
        lastUidSize[index] = uid.size;
        fieldColor[index] = detectedColor;
        placeOrder[index] = ++placeCounter;

        Serial.print(F("Reader "));
        Serial.print(index + 1);
        Serial.print(F(" | UID: "));
        printUid(uid.uidByte, uid.size);
        Serial.print(F(" | Farbe: "));
        Serial.println(colorName(detectedColor));

        if (detectedColor == NO_COLOR) {
          // Unbekannte Tags bekommen keinen Kommentar.
          Serial.println(
            F("WARNUNG: UID nicht zugeordnet. Keine Farbe vergeben.")
          );
        } else {
          // Die Stimme lobt gleiche Farben und raet von abweichenden
          // ab. Das ist absichtlich der Gegenspieler zur echten
          // Gewinnbedingung.
          commentOnPlacement(index, detectedColor);
        }
      }

      reader.PICC_HaltA();
      reader.PCD_StopCrypto1();
    }
  }

  // Token gilt nach READER_CARD_LOST_MS ohne erfolgreiche
  // Erkennung als entfernt.
  if (cardPresent[index] &&
      millis() - lastSeen[index] > READER_CARD_LOST_MS[index]) {

    Color removedColor = fieldColor[index];

    cardPresent[index] = false;
    fieldColor[index] = NO_COLOR;
    lastUidSize[index] = 0;

    Serial.print(F("Feld "));
    Serial.print(index + 1);
    Serial.println(F(" ist jetzt leer."));

    if (removedColor != NO_COLOR) {
      commentOnRemoval(removedColor);
    }

    // Moeglicher Kommunikationsabbruch (z.B. durch Spannungseinbruch
    // beim Soundabspielen) - RFID-Leser vorsichtshalber neu starten.
    reinitRfidReaders();
  }
}

// --------------------------------------------------
// TASTER PER PIN-CHANGE-INTERRUPT
// INPUT_PULLUP: gedrueckt = LOW
//
// Ein reines Abfragen von digitalRead() in loop() wuerde einen
// sehr kurzen Tastendruck verpassen, wenn loop() gerade mit den
// RFID-Lesern beschaeftigt ist. Der Interrupt erkennt den Druck
// daher sofort, unabhaengig davon, was loop() gerade tut.
// --------------------------------------------------

volatile bool buttonPressFlag = false;
volatile unsigned long lastButtonInterruptMicros = 0;

ISR(PCINT2_vect) {
  unsigned long now = micros();

  if (now - lastButtonInterruptMicros < (DEBOUNCE_MS * 1000UL)) {
    return;
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonPressFlag = true;
    lastButtonInterruptMicros = now;
  }
}

bool buttonPressedEvent() {
  if (!buttonPressFlag) {
    return false;
  }

  buttonPressFlag = false;
  return true;
}

// --------------------------------------------------
// SUESSIGKEIT AUSGEBEN
// --------------------------------------------------

void startDispensing() {
  if (dispensing) {
    return;
  }

  Serial.println(F("Mindestens drei verschiedene Farben: Ausgabe startet!"));

  servoTargetPulseUs = SERVO_OPEN_PULSE_US;

  dispensing = true;
  servoOpenUntil = millis() + SERVO_OPEN_MS;

  queueClip(FOLDER_BUTTON, TEAM_DIVERSE);
}

void updateServo() {
  if (dispensing &&
      (long)(millis() - servoOpenUntil) >= 0) {

    servoTargetPulseUs = SERVO_CLOSED_PULSE_US;
    dispensing = false;

    Serial.println(F("Servo zurueck in Ausgangsposition."));
    Serial.println(F("Bereit fuer die naechste Pruefung (Tokens duerfen liegen bleiben)."));
  }
}

// --------------------------------------------------
// AUSWERTUNG BEIM TASTERDRUCK
// --------------------------------------------------

void evaluateSelection() {
  if (dispensing) {
    return;
  }

  Serial.println(F("Taster gedrueckt - Auswahl wird geprueft."));
  printFields();

  if (enoughDiversity()) {
    Serial.println(F("Bedingung erfuellt: genug unterschiedliche Farben."));
    startDispensing();
    return;
  }

  Serial.println(F("Bedingung nicht erfuellt. Keine Ausgabe."));

  // Direkte Reaktion auf den Tasterdruck hat Vorrang vor einem
  // noch laufenden Kommentar (z.B. vom Platzieren eines Tokens) und
  // ersetzt auch einen im selben Durchlauf vorgemerkten Clip.
  queueClip(FOLDER_CRASH, TRACK_CRASH);
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Pin-Change-Interrupt fuer Pin 4 (PCINT20, PORTD-Gruppe) aktivieren.
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);

  // Alle SS-Pins deaktivieren, bevor SPI startet.
  for (byte i = 0; i < 4; i++) {
    pinMode(RFID_SS[i], OUTPUT);
    digitalWrite(RFID_SS[i], HIGH);
  }

  // Gemeinsamen Hardware-Reset nur einmal ausfuehren.
  pinMode(RFID_RST, OUTPUT);
  digitalWrite(RFID_RST, LOW);
  delay(50);
  digitalWrite(RFID_RST, HIGH);
  delay(50);

  SPI.begin();

  // Alle vier Reader ohne individuellen Reset initialisieren.
  for (byte i = 0; i < 4; i++) {
    digitalWrite(RFID_SS[i], HIGH);
    readers[i]->PCD_Init();
    applyReaderGain(i);
    delay(50);

    byte version = readers[i]->PCD_ReadRegister(MFRC522::VersionReg);

    Serial.print(F("RFID-Reader "));
    Serial.print(i + 1);
    Serial.print(F(" initialisiert. Versions-Register: 0x"));
    Serial.println(version, HEX);

    if (version == 0x00 || version == 0xFF) {
      Serial.println(F("  -> Keine Antwort vom Chip. Verkabelung/Stromversorgung pruefen!"));
    }
  }

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  servoTargetPulseUs = SERVO_CLOSED_PULSE_US;
  lastServoPulseMicros = micros();

  pinMode(SPEAKER_SWITCH_PIN, OUTPUT);
  digitalWrite(SPEAKER_SWITCH_PIN, SPEAKER_B_INACTIVE);

  dfSerial.begin(9600);

  if (dfPlayer.begin(dfSerial)) {
    dfPlayerAvailable = true;
    // Lautstaerke 0-30. War 15 (vorher 22, wegen Stromspitzen gesenkt).
    // Falls RFID/Servo beim Abspielen wieder spinnen: wieder runter.
    dfPlayer.volume(20);
    Serial.println(F("DFPlayer bereit."));
  } else {
    dfPlayerAvailable = false;
    Serial.println(F("DFPlayer nicht erkannt."));
    Serial.println(F("Verkabelung und SD-Karte pruefen."));
  }

  Serial.println();
  Serial.println(F("Suessigkeitenmaschine gestartet."));
  Serial.println(F("Tokens auf die vier Leser legen."));
  Serial.println(F("Mit dem Taster die Auswahl bestaetigen."));
}

// --------------------------------------------------
// HAUPTSCHLEIFE
// --------------------------------------------------

void loop() {
  for (byte i = 0; i < 4; i++) {
    scanReader(i);
  }

  updateServo();
  updateServoPulse();

  if (buttonPressedEvent()) {
    evaluateSelection();
  }

  // Hoechstens ein Clip pro Durchlauf (siehe queueClip()).
  playPendingClip();
}
