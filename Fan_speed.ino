// Preconfigured board options:
//   Digispark Pro    (One fan)
//   Arduino Uno/Nano (One fan)
//   Teensy 2.0       (Two fans)
//   Arduino Mega     (Four fans)
// And a custom config to allow any other board with one 16-bit timer and free hardware interrupt pin per fan configured

// Uses Timer1/3/4/5 for each fan respectively as these are usually the 16-bit timers if the board has them

// Example fans: 
// In (Noctua NF S12A):
//   Fan speed 300-1200 RPM
//   600-2400 half_revolutions/min
//   10-40 Hz
//   **100-25 ms**
// Out (Emulating: Nidec V12E12BS1B570):
//   Fan speed 1600?-6400 RPM
//   3200-12800 half_revolutions/min
//   53.33-213.33 Hz
//   18.75-4.69 ms
//   **4687-1172 counts**
// Factor 5.333

// For each fan x (0-3), the following pins need to be configured:
  //  Connected to the fan to read its actual RPM
  // #define IN_PIN_x  3  // INT0/1/2/...

  //  Connected to the end device to output the modified signal to
  // #define OUT_PIN_x 5  // Any other non-restricted pin

  //  And optionally, to use a potentiometer to set the modification factor between fan speed and output speed
  // #define POT_PIN_x A0 // An analog input pin
  //  Can also define the same pin to control multiple fans from the same potentiometer, e.g.:
  // #define POT_PIN_3 POT_PIN_1
// There are defaults for a few boards configured below.
// Ideally comment out fan configs that aren't being used on your board if any.

#if defined(PINMAPPING_DIGI)
  // Could use defined(__AVR_ATtinyX7__) instead for more general boards, but not sure on pin mapping
  // Use the ATTinyCore for the Digispark Pro boards
  // Possibly need to make sure "Digispark" is selected from the Tools -> Pin Mapping submenu
  // Extra info:
  //  https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/ATtiny_x7.md
  //  https://github.com/SpenceKonde/ATTinyCore/blob/c0b7e78f750111bbfeeb36088ef9be3d5728a927/avr/variants/tinyx7_digi/pins_arduino.h
  #define IN_PIN_0  PA3  // INT1/Digital pin 9 on the Digispark Pro
  #define OUT_PIN_0 5    // Any other non-restricted pin
  // #define POT_PIN_0 A0 // Uncomment to enable potentiometer (An analog input pin)

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #define IN_PIN_0  3  // INT1
  #define OUT_PIN_0 5  // Any other non-restricted pin
  // #define POT_PIN_0 A0 // Uncomment to enable potentiometer (An analog input pin)

#elif defined(__AVR_ATmega32U4__)  // Teensy 2.0 etc.
  #define IN_PIN_0  5  // INT0
  #define OUT_PIN_0 4  // Any other non-restricted pin
  // #define POT_PIN_0 A0 // Uncomment to enable potentiometer (An analog input pin)

  #define IN_PIN_1  6  // INT1
  #define OUT_PIN_1 7  // Any other non-restricted pin
  // #define POT_PIN_1 A1 // Uncomment to enable potentiometer (An analog input pin)

#elif defined(ARDUINO_AVR_MEGA2560)
  // INTx as defined by the Arduino library rather than the chip:
  //  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/#_interrupt_numbers
  #define IN_PIN_0  3  // INT1
  #define OUT_PIN_0 5  // Any other non-restricted pin
  // #define POT_PIN_0 A0 // Uncomment to enable potentiometer (An analog input pin)

  #define IN_PIN_1 21  // INT2
  #define OUT_PIN_1 7  // Any other non-restricted pin
  // #define POT_PIN_1 A1 // Uncomment to enable potentiometer (An analog input pin)

  #define IN_PIN_2 20  // INT3
  #define OUT_PIN_2 9  // Any other non-restricted pin
  // #define POT_PIN_2 A2 // Uncomment to enable potentiometer (An analog input pin)

  #define IN_PIN_3 19  // INT4
  #define OUT_PIN_3 11 // Any other non-restricted pin
  // #define POT_PIN_3 A3 // Uncomment to enable potentiometer (An analog input pin)

#else
  // Define custom boards here
  //  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  // #define IN_PIN_0  ?  // INT0/1/2/3/... (A free hardware interrupt pin)
  // #define OUT_PIN_0 ?  // Any other non-restricted pin
  // #define POT_PIN_0 A0 // Uncomment to enable potentiometer (An analog input pin)

#endif

// Conversion factor between fan speed and output speed
// Set by a potentiometer if respective pin is defined above
#define FACTOR_POT_MIN 0.1f
#define FACTOR_POT_MAX 10.0f
// Otherwise set to a fixed value here:
#define FACTOR 2.0f

// Time in milliseconds between updates to the RPM (longer than 6 sec can cause overflow)
// Can be helpful to make this shorter if the fan gets tested for it's speed range at startup
#define UPDATE_INTERVAL_MS 500

// Rate at which average updates to new data
// Usually want lower alpha/rate if the update interval is bigger if output smoothing is desired
// Set to 1 and 0 respectively to disable averaging and use instantaneous timing instead
#define ALPHA 0.5f
#define ONE_MINUS_ALPHA 0.5f

// TODO: CS1x is only for Timer1, unsure if it really matters that using it for other timers?
// Uncomment desired resolution setting of output RPM:
  // clk/64 (011):
  //   0.000,004 sec resolution & Max interval: 0.262 sec
  //   Min pulse frequency: 3.8 Hz    & Max: 250,000 Hz
  //   Min(2 pulses/rev):   144.4 RPM & Max: 7,500,000 RPM
  //   RPM resolution:
  //    @300RPM:   ~0.003
  //    @1200RPM:  ~0.048
  //    @10000RPM: ~3.3
#define PRESCALER (0 << CS12) | (1 << CS11) | (1 << CS10)
#define US_TO_COUNT 4.0f

  // clk/256 (100):
  //   0.000,016 sec resolution & Max interval: 1.049 sec
  //   Min pulse frequency: 0.95 Hz  & Max: 62500 Hz
  //   Min(2 pulses/rev):   28.6 RPM & Max: 1,875,000 RPM
  //   RPM resolution:
  //    @300RPM:   ~0.012
  //    @1200RPM:  ~0.192
  //    @10000RPM: ~13.4
// #define PRESCALER (1 << CS12) | (0 << CS11) | (0 << CS10)
// #define US_TO_COUNT 16.0f

  // clk/1024 (101):
  //   0.000,064 sec resolution & Max interval: 4.194 sec
  //   Min pulse frequency: 0.238 Hz & Max: 15625 Hz
  //   Min(2 pulses/rev):   7.15 RPM & Max: 468,750 RPM
  //   RPM resolution:
  //    @300RPM:   ~0.048
  //    @1200RPM:  ~0.769
  //    @10000RPM: ~53.9
// #define PRESCALER (1 << CS12) | (0 << CS11) | (1 << CS10)
// #define US_TO_COUNT 64.0f



volatile unsigned long previousMicros[4] = {0, 0, 0, 0};
// Initialise at slowest fan speed for Noctua NF S12A
volatile float avgInterval[4] = {100.0f, 100.0f, 100.0f, 100.0f};
uint16_t toggleA[4] = {4687, 4687, 4687, 4687};
uint16_t toggleB[4] = {2343, 2343, 2343, 2343};


void setup() {

  #ifdef POT_PIN_0
    pinMode(POT_PIN_0, INPUT);
  #endif
  #ifdef POT_PIN_1
    pinMode(POT_PIN_1, INPUT);
  #endif
  #ifdef POT_PIN_2
    pinMode(POT_PIN_2, INPUT);
  #endif
  #ifdef POT_PIN_3
    pinMode(POT_PIN_3, INPUT);
  #endif

    cli();  // disable global interrupts

  #ifdef IN_PIN_0
    // Timer1
    pinMode(IN_PIN_0, INPUT_PULLUP);
    // Connect OUT_PIN_0 to ground
    pinMode(OUT_PIN_0, OUTPUT);
    digitalWrite(OUT_PIN_0, LOW);

    attachInterrupt(digitalPinToInterrupt(IN_PIN_0), []() {halfRevolution(0);}, RISING);

    TCCR1A = 0;  // Disconnect output pins from timer 1
    TCCR1B = (1 << WGM12);  // Set Timer 1 to CTC mode

    OCR1A = toggleA[0];  // Set 16-bit Output Compare Register A (TOP value / High trigger time)
    OCR1B = toggleB[0];  // Set 16-bit Output Compare Register B (Low trigger time)
    TIMSK1 |= (1 << OCIE1A);  // Enable TIMER1_COMPA interrupt
    TIMSK1 |= (1 << OCIE1B);  // Enable TIMER1_COMPB interrupt
    TCCR1B |= PRESCALER;
  #endif

  #ifdef IN_PIN_1
    // Timer3
    pinMode(IN_PIN_1, INPUT_PULLUP);
    pinMode(OUT_PIN_1, OUTPUT); digitalWrite(OUT_PIN_1, LOW);
    attachInterrupt(digitalPinToInterrupt(IN_PIN_1), []() {halfRevolution(1);}, RISING);
    // Skip to Timer3 since Timer2 seems to usually be 8-bit by convention
    TCCR3A = 0;  // Disconnect output pins from timer 3
    TCCR3B = (1 << WGM32) | PRESCALER;  // Set Timer 3 to CTC mode and add prescaler
    OCR3A = toggleA[1]; OCR3B = toggleB[1];
    TIMSK3 |= (1 << OCIE3A) | (1 << OCIE3B);
  #endif

  #ifdef IN_PIN_2
    // Timer4
    pinMode(IN_PIN_2, INPUT_PULLUP);
    pinMode(OUT_PIN_2, OUTPUT); digitalWrite(OUT_PIN_2, LOW);
    attachInterrupt(digitalPinToInterrupt(IN_PIN_2), []() {halfRevolution(2);}, RISING);
    TCCR4A = 0;  // Disconnect output pins from timer 4
    TCCR4B = (1 << WGM42) | PRESCALER;  // Set Timer 4 to CTC mode and add prescaler
    OCR4A = toggleA[2]; OCR4B = toggleB[2];
    TIMSK4 |= (1 << OCIE4A) | (1 << OCIE4B);
  #endif

  #ifdef IN_PIN_3
    // Timer5
    pinMode(IN_PIN_3, INPUT_PULLUP);
    pinMode(OUT_PIN_3, OUTPUT); digitalWrite(OUT_PIN_3, LOW);
    attachInterrupt(digitalPinToInterrupt(IN_PIN_3), []() {halfRevolution(3);}, RISING);
    TCCR5A = 0;  // Disconnect output pins from timer 5
    TCCR5B = (1 << WGM52) | PRESCALER;  // Set Timer 5 to CTC mode and add prescaler
    OCR5A = toggleA[3]; OCR5B = toggleB[3];
    TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);
  #endif

    sei();  // re-enable global interrupts
}

void loop() {
    delay(UPDATE_INTERVAL_MS);

    float convFactor;

  #ifdef IN_PIN_0
    #ifdef POT_PIN_0
      convFactor = mapFloat(analogRead(POT_PIN_0), 0, 1023, FACTOR_POT_MIN, FACTOR_POT_MAX);
    #else
      convFactor = FACTOR;
    #endif
    calculateOutputInterval(0, convFactor);
  #endif

  #ifdef IN_PIN_1
    #ifdef POT_PIN_1
      convFactor = mapFloat(analogRead(POT_PIN_1), 0, 1023, FACTOR_POT_MIN, FACTOR_POT_MAX);
    #else
      convFactor = FACTOR;
    #endif
    calculateOutputInterval(1, convFactor);
  #endif

  #ifdef IN_PIN_2
    #ifdef POT_PIN_2
      convFactor = mapFloat(analogRead(POT_PIN_2), 0, 1023, FACTOR_POT_MIN, FACTOR_POT_MAX);
    #else
      convFactor = FACTOR;
    #endif
    calculateOutputInterval(2, convFactor);
  #endif

  #ifdef IN_PIN_3
    #ifdef POT_PIN_3
      convFactor = mapFloat(analogRead(POT_PIN_3), 0, 1023, FACTOR_POT_MIN, FACTOR_POT_MAX);
    #else
      convFactor = FACTOR;
    #endif
    calculateOutputInterval(3, convFactor);
  #endif
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow; 
}

void calculateOutputInterval(byte fanIndex, float convFactor) {
    float maxCount;

    maxCount = avgInterval[fanIndex] / (convFactor * US_TO_COUNT);
    maxCount = constrain(maxCount, 2.0f, 65535.0f);

    toggleA[fanIndex] = lround(maxCount);
    toggleB[fanIndex] = toggleA[fanIndex]/2;
}


/* Interrupt Functions */
void halfRevolution(byte fanIndex) {
    // For each rotation of the fan, this interrupt function is triggered twice
    unsigned long currentMicros = micros();
    // Add some smoothing to the speed
    avgInterval[fanIndex] = (ALPHA*(currentMicros-previousMicros[fanIndex])) + (ONE_MINUS_ALPHA*avgInterval[fanIndex]);

    previousMicros[fanIndex] = currentMicros;
}


#ifdef IN_PIN_0
ISR(TIMER1_COMPA_vect) {
    // Interrupt releasing OUT_PIN_0 (Floating/Letting PC pull HIGH)
    pinMode(OUT_PIN_0, INPUT);
    // OCR1A/B are set here to simulate double buffering that avoids
    // setting TOP lower than the current timer and thus missing the interrupt
    if (OCR1A != toggleA[0]) {
      OCR1A = toggleA[0];
      OCR1B = toggleB[0];
    }
}
ISR(TIMER1_COMPB_vect) {
    // Interrupt connecting OUT_PIN_0 to ground (LOW)
    pinMode(OUT_PIN_0, OUTPUT);
}
#endif

#ifdef IN_PIN_1
ISR(TIMER3_COMPA_vect) {
    pinMode(OUT_PIN_1, INPUT);
    if (OCR3A != toggleA[1]) {
      OCR3A = toggleA[1];
      OCR3B = toggleB[1];
    }
}
ISR(TIMER3_COMPB_vect) {
    pinMode(OUT_PIN_1, OUTPUT);
}
#endif

#ifdef IN_PIN_2
ISR(TIMER4_COMPA_vect) {
    pinMode(OUT_PIN_2, INPUT);
    if (OCR4A != toggleA[2]) {
      OCR4A = toggleA[2];
      OCR4B = toggleB[2];
    }
}
ISR(TIMER4_COMPB_vect) {
    pinMode(OUT_PIN_2, OUTPUT);
}
#endif

#ifdef IN_PIN_3
ISR(TIMER5_COMPA_vect) {
    pinMode(OUT_PIN_3, INPUT);
    if (OCR5A != toggleA[3]) {
      OCR5A = toggleA[3];
      OCR5B = toggleB[3];
    }
}
ISR(TIMER5_COMPB_vect) {
    pinMode(OUT_PIN_3, OUTPUT);
}
#endif
