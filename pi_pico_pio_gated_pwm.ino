#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

#include "gate_pwm.pio.h" // Auto-generated by "pioasm gate_pwm.pio gate_pwm.pio.h"
#include "hardwarePWM.h"  // Custom header for hardware PWM setup

#define LED_PIN LED_BUILTIN
const char STANDARD_DELIMITER = ':';
const int MAX_BUFFER_SIZE = 300;      // Maximum buffer size for input string
const int BAUD_RATE = 115200;         // Baud rate for serial communication
String inputString = "";              // a string to hold incoming data
volatile bool stringComplete = false; // whether the string is complete
// Pin definitions for the test
// - IN_PIN:  The hardware PWM input signal we want to gate
// - SIDE_PIN: The "gated" output (side-set pin in the PIO program)
// - JMP_PIN:  The CPU drives this pin high to interrupt gating mid-way
// - TESTING_PIN: The pin is physically wired to the SIDE_PIN to allow on-chip measurement of the number of pulses seen on the side-set pin
const uint IN_PIN = 0;
const uint SIDE_PIN = 20;
const uint JMP_PIN = 4;
const uint TESTING_PIN = 22;

// PIO and state machine, pio program offset
PIO pio;
uint sm;
uint offset;
int pio_irq_num;

float frequency = 20000.f; // PWM frequency in Hz
float duty_cycle = 50.f;   // PWM duty cycle in percent

// track the PIO's IRQ #0
volatile bool pio_irq_fired = false;

// track the number of pulses counted by the side-pin ISR
volatile uint g_pulseCount = 0;

// PIO interrupt handler
void pioIrqHandler()
{
    bool flag = pio_interrupt_get(pio, sm);
    if (flag) // Check if the interrupt was fired
    {
        Serial.printf("PIO %d, SM %d IRQ fired!\n", pio_get_index(pio), sm);
        pio_interrupt_clear(pio, sm); // Clear the interrupt
        irq_clear(pio_irq_num);       // Clear the IRQ flag
        pio_irq_fired = true;
    }
}

// test pin ISR
void testPinISR()
{
    g_pulseCount++;
}

void setup()
{
    inputString.reserve(MAX_BUFFER_SIZE); // reserve memory for input
    // Initialize Serial output
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    delay(1000);

    pinMode(LED_PIN, OUTPUT); // Set the LED pin as output
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Starting gate_pwm test...");
    Serial.println("Initializing PIO and state machine...");

    // 1) Initialize the jmp pin as output with pull-up enabled
    // This pin is used to forcibly interrupt the PIO program mid-way by setting it high.
    gpio_init(JMP_PIN);
    gpio_set_dir(JMP_PIN, GPIO_OUT);      // Set as output
    gpio_set_pulls(JMP_PIN, true, false); // Pull-up enabled, pull-down disabled
    gpio_put(JMP_PIN, 0);                 // Keep it low to start

    // 2) Generate hardware PWM on IN_PIN => e.g. 2kHz, 50% duty
    bool ok = setPWM(IN_PIN, frequency, duty_cycle, false);
    if (!ok)
    {
        while (true)
        {
            Serial.printf("Failed to set PWM freq=%f, duty=%f%% on pin %u\n",
                          frequency, duty_cycle, IN_PIN);
            delay(1000);
        }
    }
    Serial.printf("PWM output on GPIO %u => freq=%f Hz, duty=%f%%\n",
                  IN_PIN, frequency, duty_cycle);

    // 4) claim a free PIO and state machine and load the program
    ok = pio_claim_free_sm_and_add_program(&gate_pwm_program, &pio, &sm, &offset);
    if (!ok)
    {
        while (1)
            Serial.println("ERROR: Unable to claim a free PIO and SM for gate_pwm program.");
        delay(5000);
    }
    // check if we are using the first 4 state machines (0-3) of PIO0
    if (sm > 3)
    {
        Serial.println("ERROR: gate_pwm program only supports SM0-SM3 of PIO0.");
        while (1)
            delay(5000);
    }

    // 5) Use helper function from gate_pwm.pio.h to configure the SM
    gate_pwm_program_init(pio, sm, offset, IN_PIN, SIDE_PIN, JMP_PIN);

    // 6) Link interrupt from state machine lower 4 interrupts to the CPU
    // ONLY the lower 4 interrupts are visible to the CPU
    // the UPPER 4 are internal to the PIO and not visible to the CPU
    pio_interrupt_source_t source = static_cast<pio_interrupt_source_t>(pis_interrupt0 + sm);
    Serial.printf("Enabling interrupt source %d for SM %d\n", source, sm);
    pio_set_irqn_source_enabled(pio, 0, source, true);

    // 7) Setup exclusive IRQ handler
    pio_irq_num = pio_get_irq_num(pio, 0);
    irq_set_exclusive_handler(pio_irq_num, pioIrqHandler);
    irq_set_enabled(pio_irq_num, true); // Enable the interrupt

    // 5) Enable the state machine
    pio_sm_set_enabled(pio, sm, true);

    // 6) Also set up a CPU interrupt to measure the number of pulses seemed on the test pin
    // NOTE: the test pin need to be physically wired to the side-set pin during the test
    // we use arduino interrupts to count the number of pulses seen on the side-set pin
    pinMode(TESTING_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TESTING_PIN), testPinISR, RISING);

    Serial.printf("Using PIO %d, SM %d, offset %d, IRQ %d\n", pio_get_index(pio), sm, offset, pio_irq_num);
    Serial.println("Setup complete. Type e.g. '100:2' => gate 100 pulses, forcibly exit after 2s");
    Serial.println("We print how many pulses we counted on the testing pin which is wired to the side-set pin.");
}

void loop()
{
    if (stringComplete)
    {
        parseInputString();
        inputString = "";       // clear the input string
        stringComplete = false; // reset the flag
    }
}

void parseInputString()
{
    inputString.trim();
    if (inputString.length() == 0)
    {
        Serial.println("ERROR: Empty command.");
        return;
    }

    // split the input string into values
    int index = 0;                                  // current character index
    int inputStringLength = inputString.length();   // length of the input string
    int maxArrayLength = inputStringLength / 2 + 1; // maximum number of arrays
    String values[maxArrayLength];                  // array to hold the values
    int valueCount = 0;                             // number of values parsed
    while (index < inputStringLength)
    {
        int delimiterIndex = inputString.indexOf(STANDARD_DELIMITER, index);
        if (delimiterIndex == -1)
        {
            values[valueCount++] = inputString.substring(index); // Take remainder of the string
            break;
        }
        else
        {
            values[valueCount++] = inputString.substring(index, delimiterIndex);
            index = delimiterIndex + 1; // move to the next character after the delimiter
        }
    }
    if (valueCount < 2)
    {
        Serial.println("ERROR: Invalid command format, expected format is <N>:<T>, N pulses, T seconds to forcibly exit. T = 0 mean we will skip the forced exit.");
        return;
    }

    // check if N is a string name pwm, if so we will change the pwm frequency and duty cycle
    if (values[0].equalsIgnoreCase("pwm"))
    {
        if (valueCount < 3)
        {
            Serial.println("ERROR: Invalid command format for PWM change. Expected format is 'pwm:<frequency>:<duty_cycle>'");
            return;
        }
        float newFrequency = values[1].toFloat();
        float newDutyCycle = values[2].toFloat();
        if (newFrequency <= 0 || newDutyCycle < 0 || newDutyCycle > 100)
        {
            Serial.println("ERROR: Invalid frequency or duty cycle. Frequency must be > 0 and duty cycle must be between 0 and 100.");
            return;
        }
        frequency = newFrequency;
        duty_cycle = newDutyCycle;
        Serial.printf("Changing PWM to freq=%f Hz, duty=%f%%\n", frequency, duty_cycle);
        setPWM(IN_PIN, frequency, duty_cycle, true); // reconfigure the PWM
        return;
    }
    Serial.println("---------------------------------------------------------------");

    // parse 2 integers "N T"
    int N = values[0].toInt();     // number of pulses to gate
    float T = values[1].toFloat(); // time in seconds to forcibly exit
    if (N <= 0 || T < 0)
    {
        Serial.println("Invalid input. Usage: 'N:T' where N>0 and T>=0");
        return;
    }
    Serial.println("---------------------------------------------------------------");
    Serial.printf("Gating request: %d pulses, forcibly exit after %f s\n", N, T);

    // 2) Start gating
    // Clear the CPU's side-pin pulse count
    g_pulseCount = 0;
    pio_irq_fired = false;

    // push the number of pulses to gate to the PIO state machine
    pio_sm_put_blocking(pio, sm, N);
    unsigned long waitStart = millis();
    Serial.printf("Queued %d pulses to gate...\n", N);

    // 3) If T>0, we'll forcibly exit after T seconds
    if (T > 0)
    {
        Serial.printf("Waiting %f seconds, then forcing exit mid-way\n", T);
        delayMicroseconds((uint)(T * 1000000)); // wait T seconds
        gpio_put(JMP_PIN, 1);                   // forcibly exit
    }
    else
    {
        Serial.println("No forced exit => waiting for normal gating completion...");
    }

    // 4) Wait for pio_irq_fired => gating done
    // But if there's no forced exit, the SM will exit after N pulses.
    // If forced exit, it might exit mid-step.
    unsigned long TIMEOUT_MS = (unsigned long)(T * 1.1f * 1000);
    // in case T is large, we wait T + a little
    while (!pio_irq_fired)
    {
        if (TIMEOUT_MS > 0 and millis() - waitStart > TIMEOUT_MS)
        {
            Serial.printf("ERROR: Timeout waiting for pio signal completion after %lu ms\n", TIMEOUT_MS);
        }
    }
    unsigned long finishTime = millis();
    if (pio_irq_fired)
    {
        pio_irq_fired = false;
        Serial.println("Gating completed or forcibly exited!");
        float waitTime = (float)(finishTime - waitStart) / 1000.0f; // seconds
        float frequency = (float)g_pulseCount / waitTime;           // Hz
        Serial.printf("Counted %d pulses on the test pin, waited %f seconds, frequency = %f Hz\n", g_pulseCount, waitTime, frequency);
    }

    gpio_put(JMP_PIN, 0);
}

void serialEvent()
{
    while (Serial.available())
    {
        digitalWrite(LED_PIN, LOW);
        char inChar = (char)Serial.read();
        inputString += inChar;

        if (inChar == '\n')
        {
            stringComplete = true;
            digitalWrite(LED_PIN, HIGH);
            return;
        }
        else if (inputString.length() >= MAX_BUFFER_SIZE)
        {
            Serial.println("ERROR: Input command too long.");
            inputString = "";
        }
        digitalWrite(LED_PIN, HIGH);
    }
}