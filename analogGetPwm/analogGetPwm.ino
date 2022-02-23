// analogGetPwm.ino -- int analogGetPwm(uint8_t pin) test program
//  This demos using analogGetPwm(pin) to read the PWM state of a pin
//
//
// DaveX/drf5n 2022-02-22 CC BY-SA


int analogGetPwm(uint8_t pin)
{
  // We need to make sure the PWM output was enabled for those pins
  // that support it before reading their OCRnx register.
  //
  // Derived from https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_analog.c#L96
  // based on https://forum.arduino.cc/t/what-is-the-inverse-of-analogwrite-pin-val/960774
  //
  int val;
  switch (digitalPinToTimer(pin))
  {
      // XXX fix needed for atmega8
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
    case TIMER0A:
      // connect pwm to pin on timer 0
      if ((TCCR0 & _BV( COM00)) {
      val = OCR0 ; // get pwm duty
    } else {
      val = digitalRead(pin) ? 255 : 1;
      }
      break;
#endif

#if defined(TCCR0A) && defined(COM0A1)
    case TIMER0A:
        // connect pwm to pin on timer 0, channel A
        if (TCCR0A & _BV(COM0A1)) {
        val = OCR0A; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR0A) && defined(COM0B1)
    case TIMER0B:
        // connect pwm to pin on timer 0, channel B
        if (TCCR0A & _BV(COM0B1)) {
        val = OCR0B; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR1A) && defined(COM1A1)
    case TIMER1A:
        // connect pwm to pin on timer 1, channel A
        if (TCCR1A & _BV( COM1A1)) {
        val = OCR1A; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR1A) && defined(COM1B1)
    case TIMER1B:
        // connect pwm to pin on timer 1, channel B
        if (TCCR1A & _BV( COM1B1)) {
        val = OCR1B; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR1A) && defined(COM1C1)
    case TIMER1C:
        // connect pwm to pin on timer 1, channel C
        if (TCCR1A & _BV( COM1C1)) {
        val = OCR1C; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR2) && defined(COM21)
    case TIMER2:
        // connect pwm to pin on timer 2
        if (TCCR2 & _BV( COM21)) {
        val = OCR2; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR2A) && defined(COM2A1)
    case TIMER2A:
        // connect pwm to pin on timer 2, channel A
        if (TCCR2A & _BV(COM2A1)) {
        val = OCR2A; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR2A) && defined(COM2B1)
    case TIMER2B:
        // connect pwm to pin on timer 2, channel B
        if (TCCR2A & _BV(COM2B1)) {
        val = OCR2B; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR3A) && defined(COM3A1)
    case TIMER3A:
        // connect pwm to pin on timer 3, channel A
        if (TCCR3A & _BV(COM3A1)) {
        val = OCR3A; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR3A) && defined(COM3B1)
    case TIMER3B:
        // connect pwm to pin on timer 3, channel B
        if (TCCR3A & _BV(COM3B1)) {
        val = OCR3B; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR3A) && defined(COM3C1)
    case TIMER3C:
        // connect pwm to pin on timer 3, channel C
        if (TCCR3A &  _BV(COM3C1)) {
        val = OCR3C; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR4A)
    case TIMER4A:
        //connect pwm to pin on timer 4, channel A
        if (TCCR4A & _BV(COM4A1)) {
        val = OCR4A;  // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR4A) && defined(COM4B1)
    case TIMER4B:
        // connect pwm to pin on timer 4, channel B
        if (TCCR4A & _BV(COM4B1)) {
        val = OCR4B; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR4A) && defined(COM4C1)
    case TIMER4C:
        // connect pwm to pin on timer 4, channel C
        if (TCCR4A & _BV(COM4C1)) {
        val = OCR4C; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR4C) && defined(COM4D1)
    case TIMER4D:
        // connect pwm to pin on timer 4, channel D
        if (TCCR4C & _BV( COM4D1)) {
        val = OCR4D;  // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif


#if defined(TCCR5A) && defined(COM5A1)
    case TIMER5A:
        // connect pwm to pin on timer 5, channel A
        if (TCCR5A & _BV(COM5A1)) {
        val = OCR5A; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR5A) && defined(COM5B1)
    case TIMER5B:
        // connect pwm to pin on timer 5, channel B
        if (TCCR5A & _BV(COM5B1)) {
        val = OCR5B; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

#if defined(TCCR5A) && defined(COM5C1)
    case TIMER5C:
        // connect pwm to pin on timer 5, channel C
        if (TCCR5A & _BV(COM5C1)) {
        val = OCR5C; // get pwm duty
      } else {
        val = digitalRead(pin) ? 255 : 0;
        }
      break;
#endif

    case NOT_ON_TIMER:
      default:
          val = digitalRead(pin) ? 255 : 0;

        }
  return val;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("analogGetPwm.ino -- int analogGetPwm(uint8_t pin)\n"
                 " read the PWM state from a pin"
                );

}

void loop() {
  int val, readVal;
  int errors = 0;
  int timerPins = 0;
  int nonTimerPins = 0;
  // NUM_DIGITAL_PINS from pins_arduino.h
  for ( uint8_t pin = 2; pin < NUM_DIGITAL_PINS ; pin++) {
    if (digitalPinToTimer(pin) == NOT_ON_TIMER) {
      nonTimerPins++;
      //Serial.print(pin);
      //Serial.print(" is NOT_ON_TIMER\n");
    } else {  // test the PWM
      timerPins++;
      for ( val = 0 ; val <= 255; val++) {
        analogWrite(pin, val);
        readVal = analogGetPwm(pin);
        if (val != readVal) {
          errors++;
          Serial.print("Timer ");
          Serial.print(pin);
          Serial.print(": ");
          Serial.print(val);
          Serial.print(" !=  ");
          Serial.println(readVal);
        }
      }
    }
  }
  Serial.print("Errors: ");
  Serial.println(errors);
  Serial.print("Timer Pins: ");
  Serial.print(timerPins);
  Serial.print(" non-Timer Pins: ");
  Serial.print(nonTimerPins);
  Serial.print(" Total Pins: ");
  Serial.print(timerPins + nonTimerPins);
  Serial.println(" tested. (Excluding 0,1 for RxTx)");
  delay(1000);
}
