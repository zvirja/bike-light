#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>

#define FRONT_LED_PIN PB1
static_assert(FRONT_LED_PIN == PIN1);
static_assert(FRONT_LED_PIN == DD1);

#define REAR_LED_PIN PB0
static_assert(REAR_LED_PIN == PIN0);
static_assert(REAR_LED_PIN == DD0);

#define BATTERY_LVL_SENSOR_PIN PB2
static_assert(BATTERY_LVL_SENSOR_PIN == PIN2);
static_assert(BATTERY_LVL_SENSOR_PIN == DD2);

#define BTN_PIN PB4
static_assert(BTN_PIN == PIN4);
static_assert(BTN_PIN == DD4);

#define TICK_MS 16

#define MODE_SWITCH_WINDOW_TICKS 600 // Around 10s
static_assert(TICK_MS * MODE_SWITCH_WINDOW_TICKS == 9600);

#define DEBOUNCE_DELAY_TICKS 2
static_assert(TICK_MS * DEBOUNCE_DELAY_TICKS == 32);

#define REAR_LIGHT_BLINK_INTERVAL_TICKS 32 // Around 500ms
static_assert(TICK_MS * REAR_LIGHT_BLINK_INTERVAL_TICKS == 512);

// Use big value and then we don't care about overflows at all
volatile uint32_t _ticks = 0;

volatile bool _pendingButtonPressed = false;
volatile uint32_t _buttonPressedAt = 0;
volatile uint32_t _lastButtonEventAt = 0;

volatile uint32_t _lastRearLightLastBlinkAt = 0;

enum LIGHT_STATE : uint8_t {
  OFF,
  ON
};

volatile LIGHT_STATE _currentLightState = OFF;

ISR(WDT_vect) {
  _ticks++;
}

ISR(PCINT0_vect) {
  _pendingButtonPressed = true;
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
  WDTCR |= _BV(WDCE); // allow to modify prescaler
  WDTCR &= ~(_BV(WDP0) | _BV(WDP1) | _BV(WDP2) | _BV(WDP3)); // set prescaler to 16ms
  static_assert(TICK_MS == 16, "Wrong TICK_MS");
}

inline void configureButton() {
  PCMSK |= _BV(BTN_PIN);
  GIMSK |= _BV(PCIE); // enable pin change interrupt

  DDRB &= ~_BV(BTN_PIN); // set as INPUT
  PORTB |= _BV(BTN_PIN); // enable PULL_UP
}

inline void configureFrontLed(){
  DDRB |= _BV(FRONT_LED_PIN); // set as OUT
  PORTB &= ~_BV(FRONT_LED_PIN); // set LOW
}

inline void enableFrontLed(bool enable) {
  if (enable) {
    PORTB |= _BV(FRONT_LED_PIN);
  } else {
    PORTB &= ~_BV(FRONT_LED_PIN);
  }
}

inline void configureRearLed() {
  DDRB |= _BV(REAR_LED_PIN); // set as OUT
  PORTB &= ~_BV(REAR_LED_PIN); // set LOW
}

inline void enableRearLed(bool enable) {
  if (enable) {
    PORTB |= _BV(REAR_LED_PIN);
  } else {
    PORTB &= ~_BV(REAR_LED_PIN);
  }

  _lastRearLightLastBlinkAt = _ticks;
}

inline void blinkRearLed() {
  if (_currentLightState == OFF) {
    return;
  }

  uint32_t currentTicks;
  uint32_t ticksSinceLastBlink;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    currentTicks = _ticks;
    ticksSinceLastBlink = currentTicks - _lastRearLightLastBlinkAt;
  }

  if (ticksSinceLastBlink < REAR_LIGHT_BLINK_INTERVAL_TICKS) {
    return;
  }

   PINB |= _BV(REAR_LED_PIN);
  _lastRearLightLastBlinkAt = currentTicks;
}

inline void configureBatteryLevelSensor() {
  DDRB |= _BV(BATTERY_LVL_SENSOR_PIN); // set as OUT
}

inline void enableBatteryLevelSensor(bool enable) {
  if (enable) {
    PORTB |= _BV(BATTERY_LVL_SENSOR_PIN);
  } else {
    PORTB &= ~_BV(BATTERY_LVL_SENSOR_PIN);
  }
}

inline bool shouldHandleClick() {
  uint32_t currentTicks;
  bool buttonPressed;
  uint32_t ticksSincePressed;

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    currentTicks = _ticks;
    buttonPressed = _pendingButtonPressed;

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
  _pendingButtonPressed = false;

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
  // If we are long since last click - then we just turn off
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
  configureFrontLed();
  configureRearLed();
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

    blinkRearLed();

    // keep watchdog to make button and blinking
    if (_pendingButtonPressed || _currentLightState != OFF) {
      enterPowerDownSleep(true); 
      continue;
    }

    enterPowerDownSleep(false);
  }

  return 0;
}