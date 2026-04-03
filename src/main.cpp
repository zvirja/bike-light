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

#define MODE_SWITCH_WINDOW_TICKS 1000
static_assert(TICK_MS * MODE_SWITCH_WINDOW_TICKS == 16000);

#define DEBOUNCE_DELAY_TICKS 2
static_assert(TICK_MS * DEBOUNCE_DELAY_TICKS == 32);

#define REAR_LIGHT_BLINK_INTERVAL_TICKS 20
static_assert(TICK_MS * REAR_LIGHT_BLINK_INTERVAL_TICKS == 320);

#define BATTERY_SENSOR_ON_TIMEOUT_TICKS 1875
static_assert(TICK_MS * BATTERY_SENSOR_ON_TIMEOUT_TICKS == 30000);

#define BATTERY_LEVEL_LOW_TRESHOLD 740 // Shall be around 3V
#define BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS 150
#define BATTERY_LEVEL_LOW_BLINK_COUNT 3

// Use big value and then we don't care about overflows at all
volatile uint32_t _ticks = 0;

volatile bool _pendingButtonPressed = false;
volatile uint32_t _buttonPressedAt = 0;
volatile uint32_t _lastButtonEventAt = 0;

volatile uint32_t _lastRearLightLastBlinkAt = 0;

volatile uint32_t _enableBatteryLevelModuleExpireAt = 0;

volatile uint16_t _batteryLevelMeasuringResult = 0;

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

ISR(ADC_vect) {
  _batteryLevelMeasuringResult = ADCL;
  _batteryLevelMeasuringResult |= ADCH << 8;
}

void enableWatchdogTimer(bool enable) {
  if (enable) {
    WDTCR |= _BV(WDTIE);
  } else {
    WDTCR &= ~_BV(WDTIE);
  }
}

void configureWatchdogTimer() {
  WDTCR |= _BV(WDCE); // allow to modify prescaler
  WDTCR &= ~(_BV(WDP0) | _BV(WDP1) | _BV(WDP2) | _BV(WDP3)); // set prescaler to 16ms
  static_assert(TICK_MS == 16);
}

void configureButton() {
  PCMSK |= _BV(BTN_PIN);
  GIMSK |= _BV(PCIE); // enable pin change interrupt

  DDRB &= ~_BV(BTN_PIN); // set as INPUT
  PORTB |= _BV(BTN_PIN); // enable PULL_UP
}

void configureFrontLed(){
  DDRB |= _BV(FRONT_LED_PIN); // set as OUT
  PORTB &= ~_BV(FRONT_LED_PIN); 
}

void enableFrontLed(bool enable) {
  if (enable) {
    PORTB |= _BV(FRONT_LED_PIN);
  } else {
    PORTB &= ~_BV(FRONT_LED_PIN);
  }
}

void configureRearLed() {
  DDRB |= _BV(REAR_LED_PIN); // set as OUT
  PORTB &= ~_BV(REAR_LED_PIN);
}

void enableRearLed(bool enable) {
  if (enable) {
    PORTB |= _BV(REAR_LED_PIN);
  } else {
    PORTB &= ~_BV(REAR_LED_PIN);
  }

  _lastRearLightLastBlinkAt = _ticks;
}

void onTickRearLed() {
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

void configureBatteryLevelSensor() {
  DDRB |= _BV(BATTERY_LVL_SENSOR_PIN); // set as OUT
  PORTB &= ~_BV(BATTERY_LVL_SENSOR_PIN);
}

void enableBatteryLevelModuleTemporarily() {
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    PORTB |= _BV(BATTERY_LVL_SENSOR_PIN);

    _enableBatteryLevelModuleExpireAt = _ticks + BATTERY_SENSOR_ON_TIMEOUT_TICKS;
  }
}

void onTickBatteryLevelModule() {
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    if (_enableBatteryLevelModuleExpireAt == 0 || _enableBatteryLevelModuleExpireAt > _ticks) {
      return;
    }

    _enableBatteryLevelModuleExpireAt = 0;
    PORTB &= ~_BV(BATTERY_LVL_SENSOR_PIN);
  }
}

// wired resistors are 3.3kOm and 10kOm
// connected PB3
void configureBatteryLevelMeasuring() {
  ADMUX |= _BV(REFS0); // internal 1.1V voltage reference
  ADMUX |= _BV(MUX0) | _BV(MUX1); // Pin PB3

  ADCSRA |= _BV(ADIE); // enable ADC Conversion interrupt
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2); // set prescaler to /128 for the best accuracy
}

void startBatteryLevelMeasuring() {
  ADCSRA |= _BV(ADEN); // enable ADC
  _delay_ms(50); // to stabilize and charge the capacitor

  _batteryLevelMeasuringResult = 0;

  // Sleep for better measurements
  // It will automatically start measure
  set_sleep_mode(SLEEP_MODE_ADC);
  sleep_enable();
  sleep_cpu();
}

void onTickBatteryLevelMeasuring() {
  int16_t batteryLevel;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    batteryLevel = _batteryLevelMeasuringResult;
  }

  if (batteryLevel == 0) {
    return;
  }

  ADCSRA &= ~_BV(ADEN); // disable ADC, as we got the result
  _batteryLevelMeasuringResult = 0; // reset, as we handle it here

  if (batteryLevel > BATTERY_LEVEL_LOW_TRESHOLD) {
    return;
  }

  _delay_ms(100); // to give impression that light was off before blinking

  // if battery is discharged, notify with blinking
  for (uint8_t i = 0; i < BATTERY_LEVEL_LOW_BLINK_COUNT; i++) {
    _delay_ms(BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS);
    enableFrontLed(true);
    enableRearLed(true);

    _delay_ms(BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS);
    enableFrontLed(false);
    enableRearLed(false);
  }
}

bool shouldHandleClick() {
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

LIGHT_STATE calculateNextLightState() {
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

void enterPowerDownSleep(bool keepWatchdog) {
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
  configureBatteryLevelMeasuring();

  sei();

  while(true) {
    if (shouldHandleClick()) {
      LIGHT_STATE nextState = calculateNextLightState();

      // enable module for short time after action only to save battery,
      // as it takes around 100uA in standby mode
      enableBatteryLevelModuleTemporarily();

      switch (nextState)
      {
        case ON:
          enableFrontLed(true);
          enableRearLed(true);
          break;

        case OFF:
          enableFrontLed(false);
          enableRearLed(false);
          startBatteryLevelMeasuring();
          break;
      
        default:
          break;
      }

      _currentLightState = nextState;
      continue;
    }

    onTickRearLed();
    onTickBatteryLevelModule();
    onTickBatteryLevelMeasuring();

    // keep watchdog for time-dependent services
    if (_pendingButtonPressed
      || _currentLightState != OFF
      || _enableBatteryLevelModuleExpireAt != 0) {
      enterPowerDownSleep(true); 
      continue;
    }

    enterPowerDownSleep(false);
  }

  return 0;
}