/*
 * E-Auto-Wallbox für Typ2-Ladung
 * Basierend auf "Open EVSE Firmware"
 * 
 * Features:
 *  - Steuerung Pilot-Signal mit +12V/-12V oder PWM
 *  - Rückmessung Pilotspannung min/max
 *  - Zustandsmaschine inklusive Fehlerzustand
 *  - Fehler wird nach 10s wieder verlassen in Richtung "Standby"
 * 
 * Änderungshistorie:
 *  * 2020-03-31 Uwe:
 *       - Zustandsübergänge funktionieren, Trockentest an Schalterbox-EV-Simulator bestanden
 *  * 2020-04-01 Uwe:
 *       - LED-Streifen mit WS2812 angebunden, dafür grüne LED an D13 entfernt, weil D13 auch die rote On-Board-LED ist.
 *       - Strom von 12 auf 14 A erhoht, weil mit 12A PWM nur 11A (laut Sonoff) fließen. Ggf. ist die Kabelcodierung (1k5 am PP-Kontakt)
 *         jedoch das limitierende Element.
 *  * 2021-12-21 Uwe:
 *       - 5%-PWM for triggering Powerline-Communication
 *  * 2022-03-07 Uwe:
 *       - increased time for starting PWM from 200ms to 6s (just for testing)
 *  * 2022-03-10 Uwe:
 *       - time for starting PWM from 6s to 1s (just for testing)
 *       - bugfix: initialize the timer for PWM generation even on static pin output, and start it from 0 if PWM is started.
 *                 This avoids starting the PWM with an unintended looong -12V phase.
 *       - increased time for starting PWM from 200ms to 6s (just for testing)
 *  * 2022-08-10 Uwe:
 *       - PWM duty cycle adjustable by Poti on A2 (just for testing)
 *       - corrected the CP voltage scaling (was showing 12V even if electrically only 11V were present)
 *       - decreased time for starting the 5% PWM to 1 (this means "immediately", around 100ms)
 *       - added debug pin for timing measurement. Result:
 *           100 reads take ~12ms. But in between the reads, we need ~100ms for the delay(100) in the main loop.
 *       - to improve the latency, changed the main loop delay from 100ms to (20-12)ms. This leads to cycle time of 20ms.
 *  * 2022-08-12 Uwe:
 *       - Poti selects between 5% (left side) and adjustable (right side), and PWM is updated live
 *  * 2024-03-20 lewurm:
 *       - Start porting it to Arduino Uno R4 WiFi.
 *       - Use LED Matrix instead of dedicated LED strip.
 *       - Hardcode PWM to be set to 5% instead of using a Poti to configure it.  It will only be used to initiate DC operation.
 *       - Use PWM lib to setup instead of chip specific registers.
 *       
 * Todos:
 *    - Temperaturmessung, Abschaltung oder Ladestromreduzierung bei hoher Temperatur
 *    - Telemetriedaten in JSON-Format ausgeben, so dass z.B ein ESP8266 sie auf MQTT weiterleiten kann
 *    - Error-Codes mit den roten LEDs anzeigen (z.B. 1 LED = Diodencheck, 2 LEDs = Kurzschluss, 3 LEDs = Übertemperatur, ...)
 * 
 */

/*******************************************************************************/
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;


byte frame[8][12] = {
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

void setLedStrip(uint32_t x) {
  int i = 0;
  for (; i < x && i < 8; i++) {
    for (int y = 0; y < 12; y++) {
      frame[i][y] = 1;
    }
  }
  for (int j = i; j < 8; j++) {
    for (int y = 0; y < 12; y++) {
      frame[j][y] = 0;
    }
  }
  matrix.renderBitmap(frame, 8, 12);
}

/***********************************************************************************/
/* global variables and definitions */
 
#define VOLT_PIN 1 // ControlPilot analog voltage reading pin A1
#define CHARGING_PIN 8 // digital Charging LED and Relay Trigger pin
#define PILOT_PIN D10 // n.b. PILOT_PIN *MUST* be digital 10 because SetPWMForDigitalComm() assumes it
#include "pwm.h"
PwmOut pwm(D10);
#define DEBUG_PIN 6 /* for debugging */


#define MAIN_LOOP_CYCLE_TIME_MS 20 /* 20 milliseconds main loop cycle time */

/**********************************************************************************/
/* Pilot-Signal-Erzeugung und -Messung */

typedef enum {
  PILOT_STATE_P12,PILOT_STATE_PWM,PILOT_STATE_N12} 
PILOT_STATE;
class J1772Pilot {
  PILOT_STATE m_State;
public:
  J1772Pilot() {
  }
  void Init();
  void SetState(PILOT_STATE pstate); // P12/N12
  PILOT_STATE GetState() { 
    return m_State; 
  }
  int SetPWMForDigitalComm(); // 12V 1KHz PWM
};

J1772Pilot m_Pilot;
uint8_t pilotVoltageRange;


int16_t uPilotHigh_mV;
int16_t uPilotLow_mV;
uint8_t isPwmOn=0;

void readPilotVoltages(bool printThisRound) {
 int16_t reading;
 uPilotLow_mV = 32000;
 uPilotHigh_mV = -32000;
 digitalWrite(DEBUG_PIN,HIGH);

 // TODO: tweak measurement loops?

 for (int i=0;i < 5*100;i++) {
    reading = analogRead(VOLT_PIN);  // measures pilot voltage
    if (i == 10 && printThisRound) {
      Serial.print("  reading(");
      Serial.print(i);
      Serial.print(")=");
      Serial.println(  reading);

      // TODO: measure -12V (!), 0V, 3V, 6V, 9V and +12V

      // 472 -> 0V
      // 632 -> 5.43V
      // 691 -> 7.23V
      // 743 -> 9V
      // 790 -> nix (11.08V?)
      // 853 -> 13.38V
    }
    /* tuned for:
        CP       v
                 |
       200kΩ     |
                 |
     VOLT_PIN    +--- 56kΩ >---> 5V
                 |
       100kΩ     |
                 |
       GND       v
    */
    reading -= 472; /* entspricht 0V am ControlPilot */
    reading *= 33; /* auf Millivolt skalieren */
    if (reading > uPilotHigh_mV) {
        uPilotHigh_mV = reading;
      }
      else if (reading < uPilotLow_mV) {
        uPilotLow_mV = reading;
      }
    }
 digitalWrite(DEBUG_PIN,LOW);   
}
#define PILOT_RANGE_A 1
#define PILOT_RANGE_B 2
#define PILOT_RANGE_C 3
#define PILOT_RANGE_ERROR 4
#define PILOT_RANGE_ERROR_DIODE_CHECK 5

uint8_t convertPilotVoltageToRange(void) {
  uint8_t rc;
  if (isPwmOn && ((uPilotLow_mV>-10000) || (uPilotLow_mV<-13500))) {
    /* Bei PWM sollten wir -12V sehen, sonst ist etwas faul */
    rc = PILOT_RANGE_ERROR_DIODE_CHECK;
  } else if ((uPilotHigh_mV>=10000) and (uPilotHigh_mV<=13500)) {
    rc= PILOT_RANGE_A; /* 12V, not connected */
  } else if ((uPilotHigh_mV>=7000) and (uPilotHigh_mV<=10500)) {
    rc= PILOT_RANGE_B; /* 9V, vehicle detected */

    // TODO: verify, uPilotLow_mV should be -12V
  } else if ((uPilotHigh_mV>=4000) and (uPilotHigh_mV<=7500)) {
    rc= PILOT_RANGE_C; /* 6V, ready, charging */

    // TODO: verify, uPilotLow_mV should be -12V
  } else {
    rc= PILOT_RANGE_ERROR; /* Defekt */
  }
  return rc;
}

void printPilotRange(uint8_t r) {
  switch (r) {
    case PILOT_RANGE_A: Serial.print("Range A: not connected"); break;
    case PILOT_RANGE_B: Serial.print("Range B: vehicle detected"); break;
    case PILOT_RANGE_C: Serial.print("Range C: ready/charging"); break;
    case PILOT_RANGE_ERROR: Serial.print("Range error"); break;
    case PILOT_RANGE_ERROR_DIODE_CHECK: Serial.print("Range diode check failed"); break;
    default: Serial.print("Range undefined");
  }
}


void J1772Pilot::Init()
{
  /* TODO: is this needed or taken care of by the PWM lib? */
  pinMode(PILOT_PIN, OUTPUT);

  pwm.begin(1000.0f /*Hz*/, 100.0f /*%*/);
 
  SetState(PILOT_STATE_P12); // turns the pilot on 12V steady state
}


// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_PWM = 5% duty cycle
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state) {
  /* uno r4 wifi */
  if (state == PILOT_STATE_P12) {
    pwm.pulse_perc(100.0);
  } else if (state == PILOT_STATE_PWM) {
    pwm.pulse_perc(5.0);
  } else {
    pwm.pulse_perc(0.0);
  }
  isPwmOn = 0;
  m_State = state;
}

void printPilotVoltages(void) {
  Serial.print(F("Pilot Voltages high="));
  Serial.print(  uPilotHigh_mV);
  Serial.print("  low=");
  Serial.println(  uPilotLow_mV);
}

/*********************************************************************************************************/
/* Wallbox State Machine */

#define T_TRANSITION_DEBOUNCE (250 / MAIN_LOOP_CYCLE_TIME_MS) /* ca. 250 ms für normale Zustandsübergänge */
#define T_TRANSITION_DEBOUNCE_ERR_to_A (10000/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 10 Sekunden zum Wechsel ERROR zu Standby */
//#define T_TRANSITION_DEBOUNCE_A_B (6000/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 6 Sekunden vom Einstecken bis zum Aktivieren der PWM */
//#define T_TRANSITION_DEBOUNCE_A_B (1000/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 1 Sekunde vom Einstecken bis zum Aktivieren der PWM */
#define T_TRANSITION_DEBOUNCE_A_B (200/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 200 ms vom Einstecken bis zum Aktivieren der PWM */

#define WB_STATE_UNDEFINED 0 /* nicht initialisiert */
#define WB_STATE_A 1 /* Standby */
#define WB_STATE_B 2 /* vehicle detected */
#define WB_STATE_C 3 /* ready/charging */
#define WB_STATE_ERR 4 /* error */

uint8_t wallbox_state = WB_STATE_UNDEFINED;
uint16_t tTransitionDebounce_A_B;
uint16_t tTransitionDebounce_B_C;
uint16_t tTransitionDebounce_C_B;
uint16_t tTransitionDebounce_BC_A;
uint16_t tTransitionDebounce_A_ERR;
uint16_t tTransitionDebounce_BC_ERR;
uint16_t tTransitionDebounce_ERR_A;
uint8_t printModulo;

uint8_t checkTransition_A_B(void) {
  if (pilotVoltageRange==PILOT_RANGE_B) tTransitionDebounce_A_B++; else tTransitionDebounce_A_B=0;
  return (tTransitionDebounce_A_B >= T_TRANSITION_DEBOUNCE_A_B);
}

uint8_t checkTransition_B_C(void) {
  if (pilotVoltageRange==PILOT_RANGE_C) tTransitionDebounce_B_C++; else tTransitionDebounce_B_C=0;
  return (tTransitionDebounce_B_C >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_BC_ERR(void) {
  if ((pilotVoltageRange==PILOT_RANGE_ERROR) || (pilotVoltageRange==PILOT_RANGE_ERROR_DIODE_CHECK)) tTransitionDebounce_BC_ERR++; else tTransitionDebounce_BC_ERR=0;
  return (tTransitionDebounce_BC_ERR >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_C_B(void) {
  if (pilotVoltageRange==PILOT_RANGE_B) tTransitionDebounce_C_B++; else tTransitionDebounce_C_B=0;
  return (tTransitionDebounce_C_B >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_BC_A(void) {
  if (pilotVoltageRange==PILOT_RANGE_A) tTransitionDebounce_BC_A++; else tTransitionDebounce_BC_A=0;
  return (tTransitionDebounce_BC_A >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_A_ERR(void) {
  if  ((pilotVoltageRange==PILOT_RANGE_ERROR) 
    || (pilotVoltageRange==PILOT_RANGE_ERROR_DIODE_CHECK)
    || (pilotVoltageRange==PILOT_RANGE_C)) tTransitionDebounce_A_ERR++; else tTransitionDebounce_A_ERR=0;
  return (tTransitionDebounce_A_ERR >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_ERR_A(void) {
  tTransitionDebounce_ERR_A++; /* Feste Zeit ohne Bedingung, dann einfach wieder in Normalzustand wechseln */
  return (tTransitionDebounce_ERR_A >= T_TRANSITION_DEBOUNCE_ERR_to_A);
}

void resetAllTimers(void) {
  tTransitionDebounce_A_B=0;
  tTransitionDebounce_B_C=0;
  tTransitionDebounce_C_B=0;
  tTransitionDebounce_BC_A=0;
  tTransitionDebounce_A_ERR=0;
  tTransitionDebounce_BC_ERR=0;
  tTransitionDebounce_ERR_A=0;
}

void enterState_A(void) {
  Serial.println("Entering State STANDBY, State A");
  digitalWrite(CHARGING_PIN,LOW);  
  setLedStrip(1); /* GRÜN, aber nicht zu hell */
  m_Pilot.SetState(PILOT_STATE_P12);  /* +12V */
  resetAllTimers();
  wallbox_state = WB_STATE_A;
}

void enterState_B(void) {
  Serial.println("Entering State VEHICLE DETECTED, State B");
  digitalWrite(CHARGING_PIN,LOW);  
  setLedStrip(2); /* GELB */
  m_Pilot.SetState(PILOT_STATE_PWM);  
  resetAllTimers();
  wallbox_state = WB_STATE_B;
}

void enterState_C(void) {
  Serial.println("Entering State READY/CHARGING, State C");
  digitalWrite(CHARGING_PIN,HIGH);   /* Relais-EIN und BLAU */ 
  setLedStrip(3); /* BLAU */
  m_Pilot.SetState(PILOT_STATE_PWM);  
  resetAllTimers();
  wallbox_state = WB_STATE_C;
}

void enterState_ERR(void) {
  Serial.println("Entering State ERROR");
  digitalWrite(CHARGING_PIN,LOW);
  setLedStrip(4); /* ROT */
  /* Wir könnten hier mit -12V dem Fahrzeug einen Fehler signalisieren. Aber um es nicht
   *  zu sehr zu verwirren, schalten wir mit konstant +12V die Ladung ab, damit sollte ein stabiler und sicherer
   *  Zustand erreicht sein.
   */

#if 1
  m_Pilot.SetState(PILOT_STATE_P12);  /* +12V */
#else
  /* HACK because voltage drops significantly :-/ */
  m_Pilot.SetState(PILOT_STATE_PWM);  
#endif
  resetAllTimers();
  wallbox_state = WB_STATE_ERR;
}


void runWbStateMachine(void) {
  printModulo++;
  bool printThisRound = (printModulo % (32*32))==0;
  long int t1 = millis();
  readPilotVoltages(printThisRound);
  long int t2 = millis();
  pilotVoltageRange = convertPilotVoltageToRange();
  if (printThisRound) {
     Serial.print("readPilotVoltages took "); Serial.print(t2 - t1); Serial.println(" ms (it is assumed to be 12ms)");
     printPilotVoltages();
     printPilotRange(pilotVoltageRange);
     Serial.println("");
  }
  switch (wallbox_state) {
   case WB_STATE_A: /* standby */
     if (checkTransition_A_B()) {
         Serial.println("Transition A->B");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_B();
     }
     if (checkTransition_A_ERR()) {
         Serial.println("Transition A->ERR");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_ERR();
     }
     /* Wir erlauben hier explizit NICHT den direkten Übergang zu "C ready/charging", sonst könnte man mit bloßem Anlegen eines
      *  Widerstands die Spannung einschalten.
      */
     break;
   case WB_STATE_B: /* vehicle detected */
     if (checkTransition_B_C()) {
         Serial.println("Transition B->C");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_C();
     }
     if (checkTransition_BC_ERR()) {
         Serial.println("Transition B->ERR");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_ERR();
     }
     if (checkTransition_BC_A()) {
         Serial.println("Transition B->A");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_A();
     }
     break;
   case WB_STATE_C: /* ready / charging */
     if (checkTransition_C_B()) {
         Serial.println("Transition C->B");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_B();
     }
     if (checkTransition_BC_A()) {
         Serial.println("Transition C->A");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_A();
     }
     if (checkTransition_BC_ERR()) {
         Serial.println("Transition C->ERR");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_ERR();
     }
     break;
   case WB_STATE_ERR: /* Fehler */
     if (checkTransition_ERR_A()) {
         Serial.println("Transition ERR->A");
         printPilotVoltages();
         printPilotRange(pilotVoltageRange);

         enterState_A();   
     }
     break;
   default:
     enterState_A(); /* Beim Init und falls der Zustand seltsame Werte hat */
  }
  delay(MAIN_LOOP_CYCLE_TIME_MS-12); /* 12 ms are needed for the ADC-multi-read-loop */
}

/*********************************************************************************************************/

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starte..."));
  matrix.begin();

  pinMode(CHARGING_PIN, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(CHARGING_PIN, LOW);
  m_Pilot.Init(); // init the pilot

  /* bootup animation */
  for (int i = 0; i < 10; i++) {
    setLedStrip(8);
    delay(60);
    setLedStrip(7);
    delay(60);
    setLedStrip(6);
    delay(60);
  }
}

void updatePWM(void) {
  /* hardcode to 5% PWM signal */
  if ((wallbox_state == WB_STATE_B) || (wallbox_state == WB_STATE_C)) {
    m_Pilot.SetState(PILOT_STATE_PWM);  /* live update of PWM */
  }
}

void loop() {
  runWbStateMachine();
  updatePWM();
}
