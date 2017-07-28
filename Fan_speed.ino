volatile float avgTime;
unsigned long timeOld;
uint16_t toggleA;
uint16_t toggleB;

#define IN_PIN 9  // Digital pin 9 is INT1 on the Digispark Pro (second physical interrupt pin) (Use Pin 3 on Uno)
#define OUT_PIN 5  // Any other non-important pin
#define UPDATE_RATE 500  // Time in milliseconds between updates (longer than 6 sec can cause overflow)
#define ALPHA 0.5f  // Rate at which average changes
#define ONE_MINUS_ALPHA 0.5f
#define FACTOR 5.333f  // Conversion factor between fan speed and output speed
#define MS_TO_COUNT 0.004f


void setup() {
    timeOld = 0;
    // Start at slowest fan speed for Noctua NF S12A
    avgTime = 100.0f;
    toggleA = 4687;
    toggleB = 2343;

    pinMode(IN_PIN, INPUT_PULLUP);
    // Connect OUT_PIN to ground
    pinMode(OUT_PIN, OUTPUT);
    digitalWrite(OUT_PIN, LOW);

    cli();  // disable global interrupts

    // Attach interrupt to pin INT1 (Digital Pin 9 on Digispark Pro / Pin 3 on Uno)
    // Can't use digitalPinToInterrupt() on Digispark
    attachInterrupt(1, half_revolution, RISING);

    TCCR1A = 0;  // Disconnect output pins from timer 1
    TCCR1B = (1 << WGM12);  // Set Timer 1 to CTC mode

    OCR1A = toggleA;  // Set 16-bit Output Compare Register A (TOP value / High trigger time)
    OCR1B = toggleB;  // Set 16-bit Output Compare Register B (Low trigger time)
    TIMSK1 |= (1 << OCIE1A);  // Enable TIMER1_COMPA interrupt
    TIMSK1 |= (1 << OCIE1B);  // Enable TIMER1_COMPB interrupt

    /*
        clk/1024 (101):
            0.000,064 sec resolution (15625 Hz) (937,500 RPM)
            and max time 4.194 sec (0.238 Hz) (14.3 RPM)
        clk/256 (100):
            0.000,016 sec resolution (62500 Hz) (3,750,000 RPM)
            and max 1.049 sec (0.95 Hz) (57.2 RPM)
        clk/64 (011):
            0.000,004 sec resolution (250,000 Hz) (15,000,000 RPM)
            and max 0.262 sec (3.8 Hz) (228.8 RPM)
    */
    // Set prescaler to clk/64 (0.000,004 sec/count)
    // TCCR1B |= (1 << CS12);
    TCCR1B |= (1 << CS11);
    TCCR1B |= (1 << CS10);

    sei();  // re-enable global interrupts
}

void loop() {
    delay(UPDATE_RATE);

    /*
        Out (Nidec V12E12BS1B570):
        Fan speed 1600?-6400 RPM
        3200?-12800 half_revolutions/min
        53.33?-213.33 Hz
        18.75?-4.69 ms
        **4687?-1172 counts**
    */
    float maxCount = avgTime / (FACTOR * MS_TO_COUNT);

    if (maxCount < 2.0f) maxCount = 2.0f;  // upper limit of ~5,000,000 RPM (at clk/64)
    if (maxCount > 65535.0f) maxCount = 65535.0f;  // lower limit of ~228 RPM (at clk/64)

    toggleA = lround(maxCount);
    toggleB = toggleA/2;
}


/* Interrupt Functions */
void half_revolution() {
    // For each rotation of the fan, this interrupt function is triggered twice
    /*
        In (Noctua NF S12A):
        Fan speed 300-1200 RPM
        600-2400 half_revolutions/min
        10-40 Hz
        **100-25 ms**
    */
    // Add some smoothing to the speed
    avgTime = (ALPHA*(millis()-timeOld)) + (ONE_MINUS_ALPHA*avgTime);

    timeOld = millis();
}

ISR(TIMER1_COMPA_vect) {
    // Interrupt releasing OUT_PIN (HIGH)
    pinMode(OUT_PIN, INPUT);
    // OCR1A/B are set here to simulate double buffering that avoids
    // setting TOP lower than the current timer and thus missing the interrupt
    if (OCR1A != toggleA) {
      OCR1A = toggleA;
      OCR1B = toggleB;
    }
}

ISR(TIMER1_COMPB_vect) {
    // Interrupt connecting OUT_PIN to ground (LOW)
    pinMode(OUT_PIN, OUTPUT);
}
