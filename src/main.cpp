#include <Arduino.h>

#define NUM_PINS 1
const uint8_t pins[8] = {2, 3, 4, 5, 6, 7, 8, 9}; // Pins to read

volatile unsigned int decay_time[NUM_PINS] = {0}; // Array to store on-time for each pin
volatile unsigned int value[NUM_PINS] = {0}; // Array to store the value of each pin
volatile unsigned int value2[NUM_PINS] = {0}; // Array to store the value of each pin
volatile bool pin_state[NUM_PINS] = {0}; // Array to store the state of each pin
volatile unsigned int isr_counter = 0;
volatile bool ir_turned_off = false;

volatile long event_timestamp = 0;
volatile long on_time = 0;
volatile long off_time = 0;

#define IR_ACTIVE_STATE 	HIGH
#define IR_ENABLE_PIN 10

#define PRINT_INTERVAL_MS 1000 // Print every 1000ms (1 second)
unsigned long lastPrintTime = 0;


bool digital_read_fast(uint8_t pin)
{
	if (pin < 8)
	{
		return (PIND & (1 << pin));
	}
	else if (pin < 14)
	{
		return (PINB & (1 << (pin - 8)));
	}
	return false;
}

void digital_write_fast(uint8_t pin, bool value)
{
	if (pin < 8)
	{
		if (value)
		{
			PORTD |= (1 << pin);
		}
		else
		{
			PORTD &= ~(1 << pin);
		}
	}
	else if (pin < 14)
	{
		if (value)
		{
			PORTB |= (1 << (pin - 8));
		}
		else
		{
			PORTB &= ~(1 << (pin - 8));
		}
	}
}

void setup() {
	Serial.begin(250000);

	// Initialize pins as input
    for (uint8_t i = 0; i < NUM_PINS; i++)
	{
        pinMode(pins[i], INPUT);
    }

	pinMode(IR_ENABLE_PIN, OUTPUT);

    // Configure Timer1 for CTC mode
    TCCR1A = 0; // Clear Timer/Counter Control Register A
    TCCR1B = 0; // Clear Timer/Counter Control Register B
    TCNT1  = 0; // Clear Timer/Counter Register

    // Set compare match register for 100 kHz
    OCR1A = (F_CPU / 1) / 100000 - 1; // Assuming 16 MHz clock and x prescaler

	// Set CTC mode and prescaler to None
	TCCR1B |= (1 << WGM12) | (1 << CS10);

    // Set CTC mode and prescaler to 8
    // TCCR1B |= (1 << WGM12) | (1 << CS11);

    // Enable Timer1 compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // Enable global interrupts
    sei();
}

void loop() 
{
    unsigned long currentMillis = millis();
    if (currentMillis - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = currentMillis;

		unsigned long on_time_copy[NUM_PINS];

		for (uint8_t i = 0; i < NUM_PINS; i++)
		{
			on_time_copy[i] = decay_time[i];
		}

        // Print the on-time for each pin
		Serial.println("On-time for each pin:");
		Serial.print("   Pin\t2\t3\t4\t5\t6\t7\t8\t9\n   \t");
        for (uint8_t i = 0; i < NUM_PINS; i++) {
            Serial.print(on_time_copy[i]);
			Serial.print("\t");
        }
		Serial.print("\n  \t");
		for (uint8_t i = 0; i < NUM_PINS; i++) {
            Serial.print(value[i]);
			Serial.print("\t");
        }
		Serial.print("\n  \t");
		for (uint8_t i = 0; i < NUM_PINS; i++) {
            Serial.print(value2[i]);
			Serial.print("\t");
        }
		Serial.println();
		Serial.print("   on_time: ");
		Serial.println(on_time);
    }
}

ISR(TIMER1_COMPA_vect) 
{
    // Read the state of each pin and measure the decay time
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        if (digital_read_fast(pins[i]) == IR_ACTIVE_STATE)
		{
			if (pin_state[i] == LOW)	// rising edge
			{
				event_timestamp = isr_counter;
				decay_time[i] = 0;		// reset decay time
				pin_state[i] = HIGH;	// remember state
			}
			if (!ir_turned_off)	// if IR is turned off, start measuring decay time
			{
				decay_time[i]++;
			}
        }
		else
		{
			if (pin_state[i] == HIGH)
			{
				on_time = isr_counter - event_timestamp + 20;
				value[i] = decay_time[i];
				pin_state[i] = LOW;
			}
		}
    }

	if (isr_counter >= 100000)
	{
		isr_counter = 0;
	}
	else
	{
		isr_counter++;	
	}

	if (isr_counter == 0)
	{
		digital_write_fast(IR_ENABLE_PIN, HIGH);
		for (uint8_t i = 0; i < NUM_PINS; i++)
		{
			value2[i] = decay_time[i];
			decay_time[i] = 0;
		}
		ir_turned_off = false;
	}

	if (isr_counter == 200)
	{
		ir_turned_off = true;
		digital_write_fast(IR_ENABLE_PIN, LOW);
	}
}