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

#define BATTERY_LVL_MODULE_PIN PB2
static_assert(BATTERY_LVL_MODULE_PIN == PIN2);
static_assert(BATTERY_LVL_MODULE_PIN == DD2);

#define BTN_PIN PB4
static_assert(BTN_PIN == PIN4);
static_assert(BTN_PIN == DD4);

#define TICK_MS 16

/// @brief Convert milliseconds to ticks
constexpr uint16_t MS_TO_TICKS(uint16_t ms) {
  // round up
  return ms / TICK_MS + (ms % TICK_MS != 0 ? 1 : 0);
}

#define MODE_CYCLE_WINDOW_MS 4000

#define BUTTON_DEBOUNCE_DELAY_MS 32

#define REAR_LIGHT_BLINK_INTERVAL_MS 250

#define BATTERY_LEVEL_MODULE_ON_TIMEOUT_MS 30000

#define BATTERY_LEVEL_LOW_TRESHOLD 740 // Shall be around 3V
#define BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS 150
#define BATTERY_LEVEL_LOW_BLINK_COUNT 3

volatile bool _pendingButtonPressed = false;
volatile uint8_t _buttonDebounceTicksRemaining = 0;
static_assert(MS_TO_TICKS(BUTTON_DEBOUNCE_DELAY_MS) <= UINT8_MAX);

volatile uint8_t _modeCycleTicksRemaining = 0;
static_assert(MS_TO_TICKS(MODE_CYCLE_WINDOW_MS) <= UINT8_MAX);

volatile uint8_t _rearLightBlinkTicksRemaining = 0;
static_assert(MS_TO_TICKS(REAR_LIGHT_BLINK_INTERVAL_MS) <= UINT8_MAX);

volatile bool _enableBatteryLevelModule = false;
volatile uint16_t _enableBatteryLevelModuleTicksRemaining = 0;
static_assert(MS_TO_TICKS(BATTERY_LEVEL_MODULE_ON_TIMEOUT_MS) <= UINT16_MAX);

volatile uint16_t _batteryLevelMeasuringResult = 0;

enum LIGHT_STATE : uint8_t {
  OFF,
  ON
};

volatile LIGHT_STATE _currentLightState = OFF;

inline void onTickRearLight();
inline void onTickBatteryLevelModule();
inline void onTickButton();

ISR(WDT_vect) {
  onTickRearLight();
  onTickBatteryLevelModule();
  onTickButton();
}

ISR(PCINT0_vect) {
  _pendingButtonPressed = true;
  _buttonDebounceTicksRemaining = MS_TO_TICKS(BUTTON_DEBOUNCE_DELAY_MS);
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
  // keep default 16ms prescaler
  // WDTCR |= _BV(WDCE); // allow to modify prescaler
  // WDTCR &= ~(_BV(WDP0) | _BV(WDP1) | _BV(WDP2) | _BV(WDP3));
  static_assert(TICK_MS == 16);
}

void configureButton() {
  PCMSK |= _BV(BTN_PIN);
  GIMSK |= _BV(PCIE); // enable pin change interrupt

  DDRB &= ~_BV(BTN_PIN); // set as INPUT
  PORTB |= _BV(BTN_PIN); // enable PULL_UP
}

void onTickButton() {
  if (_modeCycleTicksRemaining > 0) {
    _modeCycleTicksRemaining--;
  }

  if (_buttonDebounceTicksRemaining > 0) {
    _buttonDebounceTicksRemaining--;
  }
}

void configureFrontLight(){
  DDRB |= _BV(FRONT_LED_PIN); // set as OUT
  // PORTB &= ~_BV(FRONT_LED_PIN); 
}

void enableFrontLight(bool enable) {
  if (enable) {
    PORTB |= _BV(FRONT_LED_PIN);
  } else {
    PORTB &= ~_BV(FRONT_LED_PIN);
  }
}

void configureRearLight() {
  DDRB |= _BV(REAR_LED_PIN); // set as OUT
  // PORTB &= ~_BV(REAR_LED_PIN);
}

void enableRearLight(bool enable) {
  if (enable) {
    PORTB |= _BV(REAR_LED_PIN);
  } else {
    PORTB &= ~_BV(REAR_LED_PIN);
  }

  _rearLightBlinkTicksRemaining = MS_TO_TICKS(REAR_LIGHT_BLINK_INTERVAL_MS);
}

void onTickRearLight() {
  if(_rearLightBlinkTicksRemaining > 0){
    _rearLightBlinkTicksRemaining--;
  }
}

void onLoopRearLight() {
  if (_currentLightState == OFF || _rearLightBlinkTicksRemaining > 0) {
    return;
  }

   PINB |= _BV(REAR_LED_PIN);
  _rearLightBlinkTicksRemaining = MS_TO_TICKS(REAR_LIGHT_BLINK_INTERVAL_MS);
}

void setupBatteryLevelModule() {
  DDRB |= _BV(BATTERY_LVL_MODULE_PIN); // set as OUT
  PORTB &= ~_BV(BATTERY_LVL_MODULE_PIN);
}

void enableBatteryLevelModuleTemporarily() {
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    PORTB |= _BV(BATTERY_LVL_MODULE_PIN);

    _enableBatteryLevelModule = true;
    _enableBatteryLevelModuleTicksRemaining = MS_TO_TICKS(BATTERY_LEVEL_MODULE_ON_TIMEOUT_MS);
  }
}

void onTickBatteryLevelModule() {
  if (_enableBatteryLevelModule && _enableBatteryLevelModuleTicksRemaining > 0) {
    _enableBatteryLevelModuleTicksRemaining--; 
  }
}

void onLoopBatteryLevelModule() {
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    if (!_enableBatteryLevelModule || _enableBatteryLevelModuleTicksRemaining > 0) {
      return;
    }

    _enableBatteryLevelModule = false;
    PORTB &= ~_BV(BATTERY_LVL_MODULE_PIN);
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

void onLoopBatteryLevelMeasuring() {
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
    enableFrontLight(true);
    enableRearLight(true);

    _delay_ms(BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS);
    enableFrontLight(false);
    enableRearLight(false);
  }
}

bool shouldHandleClick() {
  // Give time for the click to stabilize and read only afterwards
  if (!_pendingButtonPressed || _buttonDebounceTicksRemaining > 0) {
    return false;
  }

  // reset click notification
  _pendingButtonPressed = false;

  bool isButtonDown = bit_is_clear(PINB, BTN_PIN); // reversed
  if (isButtonDown) {
    _modeCycleTicksRemaining = MS_TO_TICKS(MODE_CYCLE_WINDOW_MS);
  }

  return isButtonDown;
}

LIGHT_STATE calculateNextLightState() {
  LIGHT_STATE currentLightState = _currentLightState;
  if (currentLightState == OFF) {
    return ON;
  }

  // Light is ON
  bool cycleLightModes = _modeCycleTicksRemaining > 0;
  if (!cycleLightModes) {
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
  configureFrontLight();
  configureRearLight();
  setupBatteryLevelModule();
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
          enableFrontLight(true);
          enableRearLight(true);
          break;

        case OFF:
          enableFrontLight(false);
          enableRearLight(false);
          startBatteryLevelMeasuring();
          break;
      
        default:
          break;
      }

      _currentLightState = nextState;
      continue;
    }

    onLoopRearLight();
    onLoopBatteryLevelModule();
    onLoopBatteryLevelMeasuring();

    // keep watchdog for time-dependent services
    if (_pendingButtonPressed
      || _currentLightState != OFF
      || _enableBatteryLevelModule) {
      enterPowerDownSleep(true); 
      continue;
    }

    enterPowerDownSleep(false);
  }

  return 0;
}