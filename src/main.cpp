#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#define FLED_PIN PB0
#define BTN_PIN PB1

volatile uint16_t _ticks = 0;

volatile uint16_t _buttonPressedAt = 0;
volatile bool _buttonPressed = false;
volatile uint16_t _buttonPressedCount = 0;

// Is called each 32ms
ISR(WDT_vect) {
  _ticks++;
}

// Is called on button down event
ISR(INT0_vect) {
  _buttonPressedAt = _ticks;
  _buttonPressedCount++;
}

ISR(PCINT0_vect) {
  if (bit_is_clear(PINB, BTN_PIN)) {
    _buttonPressedAt = _ticks;
    _buttonPressed = true;
  } else {
    _buttonPressed = false;
  }

  _buttonPressedCount++;
}

inline void startWatchdogTimer() {
  WDTCR |= _BV(WDTIE);
}

inline void stopWatchdogTimer() {
  WDTCR &= ~_BV(WDTIE);
}

inline void configureWatchdogTimer() {
  WDTCR |= _BV(WDCE); // allow to modify prescaler
  WDTCR |= _BV(WDP0); // set prestaler to 32ms
}

inline void configureButton() {
  //GIMSK |= _BV(INT0); // enable INT0
  PCMSK |= _BV(BTN_PIN);
  GIMSK |= _BV(PCIE); // enable pin change interrupt

  DDRB &= ~_BV(BTN_PIN); // set as INPUT
  PORTB |= _BV(BTN_PIN); // enable PULL_UP
}

inline bool shouldHandleClick() {
  uint16_t currentTicks;
  uint16_t buttonPressedAt;
  bool buttonPressed;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    currentTicks = _ticks;
    buttonPressedAt = _buttonPressedAt;
    buttonPressed = _buttonPressed;
  }

  if (!buttonPressed) {
    return false;
  }

  uint16_t ticksSince = 0;
  // handle overflow
  if (currentTicks < buttonPressedAt) {
    ticksSince += (INT16_MAX - buttonPressedAt);
    ticksSince += currentTicks;
  } else {
    ticksSince = (currentTicks - buttonPressedAt);
  }

  // have 64ms debounce
  if (ticksSince < 2) {
    return false;
  }
  
  // reset click
  _buttonPressed = false;
  return true;
}

int main() {
  configureWatchdogTimer();
  startWatchdogTimer();
  configureButton();

  DDRB |= _BV(FLED_PIN);

  sei();

  while(true) {
    if (shouldHandleClick() && _buttonPressedCount < 1000) {
      PINB |= _BV(FLED_PIN);
    }
  }

  return 0;
}