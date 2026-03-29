#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>

#define FRONT_LED_PIN PB1
#define REAR_LED_PIN PB0
#define BATTERY_LVL_SENSOR PB2
#define BTN_PIN PB4

#define TICK_MS 16

#define MODE_SWITCH_WINDOW_TICKS 600 // Around 10s
static_assert(TICK_MS * MODE_SWITCH_WINDOW_TICKS == 9600);

#define DEBOUNCE_DELAY_TICKS 1
static_assert(TICK_MS * DEBOUNCE_DELAY_TICKS == 16);

// Use big value and then we don't care about overflows at all
volatile uint32_t _ticks = 0;

volatile bool _buttonPressed = false;
volatile uint32_t _buttonPressedAt = 0;
volatile uint32_t _lastButtonEventAt = 0;

enum LIGHT_STATE : uint8_t {
  OFF,
  ON
};

volatile LIGHT_STATE _currentLightState = OFF;

ISR(WDT_vect) {
  _ticks++;
}

ISR(PCINT0_vect) {
  _buttonPressed = true;
  _buttonPressedAt = _ticks;
}

inline void enableWatchdogTimer(bool enable) {
  if (enable) {
    WDTCR |= _BV(WDTIE);
  } else {
    WDTCR &= ~_BV(WDTIE);
  }
}

inline void configureWatchdogTimer() {
  // WDTCR |= _BV(WDCE); // allow to modify prescaler
  // WDTCR |= _BV(WDP0); // set prestaler to 32ms
  static_assert(TICK_MS == 16, "Wrong TICK_MS");
}

inline void configureButton() {
  PCMSK |= _BV(BTN_PIN);
  GIMSK |= _BV(PCIE); // enable pin change interrupt

  DDRB &= ~_BV(BTN_PIN); // set as INPUT
  PORTB |= _BV(BTN_PIN); // enable PULL_UP
}

inline void configureLed() {
  DDRB |= _BV(FRONT_LED_PIN); // set as OUT
  DDRB |= _BV(REAR_LED_PIN); // set as OUT
}

inline void enableFrontLed(bool enable) {
  if (enable) {
    PORTB |= _BV(FRONT_LED_PIN);
  } else {
    PORTB &= ~_BV(FRONT_LED_PIN);
  }
}

inline void enableRearLed(bool enable) {
  // for now we send two clicks to back - to enable
  // then one click - to disable
  // it's hacky - it's just to make it work for now
  if (enable) {
    PORTB |= _BV(REAR_LED_PIN);
    _delay_ms(10);
    PORTB &= ~_BV(REAR_LED_PIN);
    _delay_ms(10);
    PORTB |= _BV(REAR_LED_PIN);
    _delay_ms(10);
    PORTB &= ~_BV(REAR_LED_PIN);
  } else {
    PORTB |= _BV(REAR_LED_PIN);
    _delay_ms(10);
    PORTB &= ~_BV(REAR_LED_PIN);
  }
}

inline void configureBatteryLevelSensor() {
  DDRB |= _BV(BATTERY_LVL_SENSOR); // set as OUT
}

inline void enableBatteryLevelSensor(bool enable) {
  if (enable) {
    PORTB |= _BV(BATTERY_LVL_SENSOR);
  } else {
    PORTB &= ~_BV(BATTERY_LVL_SENSOR);
  }
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
  if (ticksSincePressed < DEBOUNCE_DELAY_TICKS) {
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

inline LIGHT_STATE calculateNextLightState() {
  LIGHT_STATE currentLightState = _currentLightState;
  if (currentLightState == OFF) {
    return ON;
  }

  // Light is ON
  // If we are more than 10ms - then we just off
  bool skipModeCycling;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    skipModeCycling = (_ticks - _lastButtonEventAt) > MODE_SWITCH_WINDOW_TICKS;
  }

  if (skipModeCycling) {
    return OFF;
  }

  // we don't have any other states as of now
  return OFF;
}

inline void enterPowerDownSleep(bool keepWatchdog) {
  if (!keepWatchdog) {
    enableWatchdogTimer(false); // for battery saving, as we don't need to track time anyway
  }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  // sleep_bod_disable(); // is not enabled in fuses
  sleep_cpu();

  enableWatchdogTimer(true);
}

int main() {
  configureWatchdogTimer();
  enableWatchdogTimer(true);
  configureButton();
  configureLed();
  configureBatteryLevelSensor();

  sei();

  while(true) {
    if (shouldHandleClick()) {
      LIGHT_STATE nextState = calculateNextLightState();

      switch (nextState)
      {
        case ON:
          enableFrontLed(true);
          enableRearLed(true);
          enableBatteryLevelSensor(true);

          break;

        case OFF:
          enableFrontLed(false);
          enableRearLed(false);
          enableBatteryLevelSensor(false);

          break;
      
        default:
          break;
      }

      _currentLightState = nextState;
      continue;
    }

    if (_buttonPressed) {
      enterPowerDownSleep(true); // keep watchdog to make button work
      continue;
    }

    enterPowerDownSleep(false);
  }

  return 0;
}