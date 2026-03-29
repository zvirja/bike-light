#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#define FLED_PIN PB0
#define BTN_PIN PB1


// Use big value and then we don't care about overflows at all
volatile uint32_t _ticks = 0;

volatile bool _buttonPressed = false;
volatile uint32_t _buttonPressedAt = 0;
volatile uint32_t _lastButtonEventAt = 0;

ISR(WDT_vect) {
  _ticks++;
}

ISR(PCINT0_vect) {
  _buttonPressed = true;
  _buttonPressedAt = _ticks;
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
  PCMSK |= _BV(BTN_PIN);
  GIMSK |= _BV(PCIE); // enable pin change interrupt

  DDRB &= ~_BV(BTN_PIN); // set as INPUT
  PORTB |= _BV(BTN_PIN); // enable PULL_UP
}

inline void configureLed() {
  DDRB |= _BV(FLED_PIN); // set pin OUT
}

inline bool shouldHandleClick() {
  uint32_t currentTicks;
  bool buttonPressed;
  uint32_t ticksSincePressed;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    currentTicks = _ticks;
    buttonPressed = _buttonPressed;

    ticksSincePressed = currentTicks - _buttonPressedAt;
  }

  if (!buttonPressed) {
    return false;
  }

  // Give time for the click to stabilize and read only afterwards
  if (ticksSincePressed < 1) {
    return false;
  }

  // reset click notification
  _buttonPressed = false;

  bool isButtonDown = bit_is_clear(PINB, BTN_PIN); // reversed
  if (isButtonDown) {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      _lastButtonEventAt = currentTicks;
    }
  }

  return isButtonDown;
}

int main() {
  configureWatchdogTimer();
  startWatchdogTimer();
  configureButton();
  configureLed();

  sei();

  while(true) {
    if (shouldHandleClick()) {
      PINB |= _BV(FLED_PIN);
    }
  }

  return 0;
}