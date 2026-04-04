#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>

#define FRONT_LED_PIN PB1
static_assert(FRONT_LED_PIN == PIN1);
static_assert(FRONT_LED_PIN == DD1);

#define FRONT_LED_PWM_LEVEL 150
#define FRONT_LED_PWM_TIMER_COMPARE_REGISTER OCR0B
#define FRONT_LED_PWM_TIMER_TCCR0A_MASK (_BV(COM0B1))

#define REAR_LED_PIN PB0
static_assert(REAR_LED_PIN == PIN0);
static_assert(REAR_LED_PIN == DD0);

#define REAR_LED_PWM_LEVEL 100
#define REAR_LED_PWM_TIMER_COMPARE_REGISTER OCR0A
#define REAR_LED_PWM_TIMER_TCCR0A_MASK (_BV(COM0A1))

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

#define MODE_CYCLE_WINDOW_MS 3000

#define BUTTON_DEBOUNCE_DELAY_MS 32

#define REAR_LIGHT_BLINK_INTERVAL_MS 250

#define BATTERY_LEVEL_MODULE_ON_TIMEOUT_MS 30000

#define BATTERY_LEVEL_LOW_TRESHOLD 740 // Shall be around 3V
#define BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS 150
#define BATTERY_LEVEL_LOW_BLINK_COUNT 3

volatile bool _pendingButtonPressed = false;
volatile uint8_t _buttonDebounceTicksRemaining = 0;
static_assert(MS_TO_TICKS(BUTTON_DEBOUNCE_DELAY_MS) <= UINT8_MAX);

volatile uint8_t _lightClickCycleModeOnTicksRemaining = 0;
static_assert(MS_TO_TICKS(MODE_CYCLE_WINDOW_MS) <= UINT8_MAX);

volatile uint8_t _rearLightBlinkTicksRemaining = 0;
static_assert(MS_TO_TICKS(REAR_LIGHT_BLINK_INTERVAL_MS) <= UINT8_MAX);

volatile bool _enableBatteryLevelModule = false;
volatile uint16_t _enableBatteryLevelModuleTicksRemaining = 0;
static_assert(MS_TO_TICKS(BATTERY_LEVEL_MODULE_ON_TIMEOUT_MS) <= UINT16_MAX);

volatile bool _pendingBatteryLevelMeasuringResult = false;
volatile uint16_t _batteryLevelMeasuringResult = 0;

enum LIGHT_STATE : uint8_t {
  OFF,
  ON,
  DIMMED
};

enum SLEEP_LEVEL : uint8_t {
  FULL_SLEEP,
  WATCHDOG_ONLY,
  IDLE
};

volatile LIGHT_STATE _currentLightState = OFF;

volatile bool _frontLedPwmOn = false;
volatile bool _rearLedPwmOn = false;
volatile bool _rearLedBlinkOn = false;

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
  _pendingBatteryLevelMeasuringResult = true;
}

void configureWatchdogTimer() {
  // keep default 16ms prescaler
  // WDTCR |= _BV(WDCE); // allow to modify prescaler
  // WDTCR &= ~(_BV(WDP0) | _BV(WDP1) | _BV(WDP2) | _BV(WDP3));
  static_assert(TICK_MS == 16);
}

void enableWatchdogTimer(bool enable) {
  if (enable) {
    WDTCR |= _BV(WDTIE);
  } else {
    WDTCR &= ~_BV(WDTIE);
  }
}

void configureTimer0() {
  // WGM0[0,1] - fast PWM, 00-FF
  TCCR0A |= (_BV(WGM00) | _BV(WGM01));
  // CS00 - no prescaling, start timer
  TCCR0B |= (_BV(CS00));
}

void configureButton() {
  PCMSK |= _BV(BTN_PIN);
  GIMSK |= _BV(PCIE); // enable pin change interrupt

  DDRB &= ~_BV(BTN_PIN); // set as INPUT
  PORTB |= _BV(BTN_PIN); // enable PULL_UP
}

void onTickButton() {
  if (_lightClickCycleModeOnTicksRemaining > 0) {
    _lightClickCycleModeOnTicksRemaining--;
  }

  if (_buttonDebounceTicksRemaining > 0) {
    _buttonDebounceTicksRemaining--;
  }
}

bool needTickButton() {
  return _buttonDebounceTicksRemaining > 0 || _lightClickCycleModeOnTicksRemaining > 0;
}

void configureFrontLight(){
  DDRB |= _BV(FRONT_LED_PIN); // set as OUT
  // PORTB &= ~_BV(FRONT_LED_PIN); 
}

void setFrontLightState(LIGHT_STATE state) {
  if (state == OFF) {
    _frontLedPwmOn = false;
    TCCR0A &= ~FRONT_LED_PWM_TIMER_TCCR0A_MASK; // disable PWM output, so it's just OFF
    return;
  }

  if (state == ON) {
    FRONT_LED_PWM_TIMER_COMPARE_REGISTER = 255;
  } else {
    _frontLedPwmOn = true;
    FRONT_LED_PWM_TIMER_COMPARE_REGISTER = FRONT_LED_PWM_LEVEL;
  }

  TCCR0A |= FRONT_LED_PWM_TIMER_TCCR0A_MASK; // enable PWM output
}

bool needTimer0FrontLight() {
  return _frontLedPwmOn;
}

void configureRearLight() {
  DDRB |= _BV(REAR_LED_PIN); // set as OUT
  // PORTB &= ~_BV(REAR_LED_PIN);
}

void setRearLightState(LIGHT_STATE state, bool blinkOn) {
  if (state == OFF) {
    _rearLedPwmOn = false;
    _rearLedBlinkOn = false;
    TCCR0A &= ~REAR_LED_PWM_TIMER_TCCR0A_MASK; // disable PWM output, so it's just OFF

    return;
  }

  if (state == ON) {
    REAR_LED_PWM_TIMER_COMPARE_REGISTER = 255;
  } else {
    _rearLedPwmOn = true;
    REAR_LED_PWM_TIMER_COMPARE_REGISTER = REAR_LED_PWM_LEVEL;
  }

  TCCR0A |= REAR_LED_PWM_TIMER_TCCR0A_MASK; // enable PWM output

  if (blinkOn) {
    _rearLedBlinkOn = true;
    _rearLightBlinkTicksRemaining = MS_TO_TICKS(REAR_LIGHT_BLINK_INTERVAL_MS);
  }
}

void onTickRearLight() {
  if(_rearLightBlinkTicksRemaining > 0){
    _rearLightBlinkTicksRemaining--;
  }
}

bool needTickRearLight() {
  return _rearLightBlinkTicksRemaining > 0;
}

bool needTimer0RearLight() {
  return _rearLedPwmOn;
}

void onLoopRearLight() {
  if (!_rearLedBlinkOn || _rearLightBlinkTicksRemaining > 0) {
    return;
  }

   TCCR0A ^= REAR_LED_PWM_TIMER_TCCR0A_MASK; // toggle
  _rearLightBlinkTicksRemaining = MS_TO_TICKS(REAR_LIGHT_BLINK_INTERVAL_MS);
}

void configureBatteryLevelModule() {
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

bool needTickBatteryLevelModule() {
  return _enableBatteryLevelModule && _enableBatteryLevelModuleTicksRemaining;
}

void onLoopBatteryLevelModule() {
  if (!_enableBatteryLevelModule) {
    return;
  }

  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    if (_enableBatteryLevelModuleTicksRemaining > 0) {
      return;
    }
  }

  _enableBatteryLevelModule = false;
  PORTB &= ~_BV(BATTERY_LVL_MODULE_PIN);
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
  if (!_pendingBatteryLevelMeasuringResult) {
    return;
  }

  _pendingBatteryLevelMeasuringResult = false; // reset, as we handle it here

  ADCSRA &= ~_BV(ADEN); // disable ADC, as we got the result

  if (_batteryLevelMeasuringResult > BATTERY_LEVEL_LOW_TRESHOLD) {
    return;
  }

  _delay_ms(100); // to give impression that light was off before blinking

  // if battery is discharged, notify with blinking
  for (uint8_t i = 0; i < BATTERY_LEVEL_LOW_BLINK_COUNT; i++) {
    _delay_ms(BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS);
    setFrontLightState(ON);
    setRearLightState(ON, false);

    _delay_ms(BATTERY_LEVEL_LOW_BLINK_INTERNAL_MS);
    setFrontLightState(OFF);
    setRearLightState(OFF, false);
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
  return isButtonDown;
}

LIGHT_STATE calculateNextLightState() {
  // Handle expired mode cycling
  if (_currentLightState != OFF && _lightClickCycleModeOnTicksRemaining == 0) {
    return OFF;
  }

  // if we are in mode cycling state, we never turn it off and just loop between ON
  switch (_currentLightState)
  {
    case OFF:
      return ON;

    case ON:
      return DIMMED;

    case DIMMED:
      return ON;
  }

  return OFF; // shall never reach
}

void enterPowerDownSleep(SLEEP_LEVEL sleepLevel) {
  if (sleepLevel == FULL_SLEEP) {
    enableWatchdogTimer(false); // for battery saving, as we don't need to track time anyway
  }

  set_sleep_mode(sleepLevel == IDLE ? SLEEP_MODE_IDLE : SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  // sleep_bod_disable(); // is not enabled in fuses
  sleep_cpu();

  enableWatchdogTimer(true);
}

int main() {
  configureWatchdogTimer();
  configureTimer0();
  configureButton();
  configureFrontLight();
  configureRearLight();
  configureBatteryLevelModule();
  configureBatteryLevelMeasuring();

  enableWatchdogTimer(true);

  sei();

  while(true) {
    if (shouldHandleClick()) {
      LIGHT_STATE nextState = calculateNextLightState();

      // enable module for short time after action only to save battery,
      // as it takes around 100uA in standby mode
      enableBatteryLevelModuleTemporarily();

      _currentLightState = nextState;
      _lightClickCycleModeOnTicksRemaining = MS_TO_TICKS(MODE_CYCLE_WINDOW_MS);

      switch (nextState)
      {
        case ON:
          setFrontLightState(ON);
          setRearLightState(ON, true);
          break;

        case DIMMED:
          setFrontLightState(DIMMED);
          setRearLightState(ON, true);
          break;

        case OFF:
          setFrontLightState(OFF);
          setRearLightState(OFF, false);
          startBatteryLevelMeasuring();
          break;
      
        default:
          break;
      }

      continue;
    }

    onLoopRearLight();
    onLoopBatteryLevelModule();
    onLoopBatteryLevelMeasuring();

    SLEEP_LEVEL sleepLevel = FULL_SLEEP;
    if (needTickButton() || needTickRearLight() || needTickBatteryLevelModule()) {
      sleepLevel = WATCHDOG_ONLY;
    }

    if (needTimer0FrontLight() || needTimer0RearLight()) {
      sleepLevel = IDLE;
    }

    enterPowerDownSleep(sleepLevel);
  }

  return 0;
}