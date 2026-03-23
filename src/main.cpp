#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/pcnt.h"

// ============ WIFI AP SETTINGS ============
const char* ap_ssid     = "Rompack";
const char* ap_password = "Mark1Mark1";

// ============ WEB SERVER ============
WebServer server(80);
Preferences prefs;

// ============ SERVO LABELLER CONSTANTS ============
float SERVO_PULSES_PER_MM = 300.0f;   // 10000 pulses/rev, 5:1 gear, 53mm roller
float SERVO_RUN_SPEED_HZ = 100000.0f; // run speed (Hz) — 100kHz = 333 mm/s
float SERVO_HS_ACCEL = 500000.0f;     // high speed acceleration Hz/s (ramp to run speed)
float SERVO_HS_DECEL = 1000000.0f;   // high speed deceleration Hz/s (run → creep)
float SERVO_LS_ACCEL = 50000.0f;     // low speed acceleration Hz/s (ramp to creep)
float SERVO_LS_DECEL = 100000.0f;    // low speed deceleration Hz/s (creep → stop)
float SERVO_BASE_SPEED_HZ = 10000.0f; // creep speed after slowdown (Hz) = 33 mm/s
uint32_t SERVO_LABEL_LENGTH = 24000;  // label length in pulses (80mm × 300)
float SERVO_SLOWDOWN_PCT = 0.95f;     // slow down at 95% of label length
uint32_t SERVO_MAX_PULSES = 31200;    // safety limit = 130% of label length
uint32_t MAX_VAC_PULSES_NO_BARCODE = 10000;

// Servo GPIO
constexpr int PIN_SERVO_DIR = 12;     // Direction pin (fixed, e.g. HT4 or available GPIO)
constexpr int PIN_SERVO_ENA = 2;      // Servo enable (DI1)

// ============ KC868-A16 PIN MAP ============
// Conveyor PWM outputs (direct GPIO via HT headers and freed RF/RS485 pins)
constexpr int PIN_CONVVAC  = 32;  // HT1 header
constexpr int PIN_CONV2    = 33;  // HT2 header
constexpr int PIN_SCROLL   = 15;  // RF transmitter pad (freed, strapping pin — HIGH at boot)
constexpr int PIN_CONVMAIN = 13;  // RS485 TX terminal (PWM output via transceiver)

// Sensor inputs (input-only analog pins — need external 10K pullup to 3.3V)
constexpr int PIN_REDSENSOR = 34;  // Analog A2
constexpr int PIN_BARCODE   = 35;  // Analog A3

// Labeller GPIO
constexpr int PIN_STEP   = 14;  // HT3 header (stepper pulses)
constexpr int PIN_SENSOR = 39;  // Analog A4 (label gap — needs external pullup)

// Toggle switches — both on I2C PCF8574 inputs

// I2C bus for PCF8574 expanders (KC868-A16 fixed assignment)
constexpr int PIN_I2C_SDA = 4;
constexpr int PIN_I2C_SCL = 5;

// PCF8574 I2C addresses (KC868-A16 board)
constexpr uint8_t PCF_INPUT_1_8  = 0x22;  // X01-X08 digital inputs
constexpr uint8_t PCF_OUTPUT_1_8 = 0x24;  // Y01-Y08 MOSFET outputs

// I2C-controlled I/O bit positions
// Inputs: X01=ACTIVATE, X02=DEACTIVATE, X03=START
constexpr uint8_t BIT_ACTIVATE   = 0;  // X01
constexpr uint8_t BIT_DEACTIVATE = 1;  // X02
constexpr uint8_t BIT_START        = 2;  // X03
constexpr uint8_t BIT_SW_LABELLER  = 3;  // X04
constexpr uint8_t BIT_SW_SCROLL    = 4;  // X05
// Outputs: Y01=LED_ACTIVE
constexpr uint8_t BIT_LED_ACTIVE = 0;  // Y01

// Current state of PCF8574 outputs (active-low: 0=ON, 1=OFF)
uint8_t pcf_output_state = 0xFF;  // all OFF initially

// ============ DEBOUNCE TIMES (microseconds) ============
constexpr uint32_t DEBOUNCE_ACTIVATE_US   = 20000;
constexpr uint32_t DEBOUNCE_DEACTIVATE_US = 20000;
constexpr uint32_t DEBOUNCE_START_US      = 20000;
constexpr uint32_t DEBOUNCE_SENSOR_US     = 50;    // Gap minimum transit time (reduced from 2000 for high-speed)
constexpr uint32_t SENSOR_TIMEOUT_US      = 30000000;  // 30 seconds safety timeout

// ============ STATES ============
enum State { IDLE, ACCEL, RUNNING, CREEP, HOMING, AUTO_DETECT, CALIBRATING };
volatile State state = IDLE;

// ============ FLAGS ============
volatile bool start_flag=false, sensor_triggered=false;
volatile bool system_active=false;    // Master enable (ACTIVATE/DEACTIVATE buttons)
volatile bool labeller_enable=false;  // Labeller toggle (only when system_active)
volatile bool convvac_enable=false;   // Conveyor VAC enable
volatile bool conv2_enable=false;     // Conveyor 2 enable
volatile bool scroll_enable=false;    // Scroll toggle (only when system_active)
volatile bool convmain_enable=false;  // Conveyor Main enable
volatile bool barcode_rising=true;    // true=RISING edge, false=FALLING edge
volatile bool red_rising=false;       // true=RISING edge, false=FALLING edge (default FALLING)
volatile bool gap_rising=true;        // true=RISING edge, false=FALLING edge (default RISING)
volatile bool activate_flag=false, deactivate_flag=false;
volatile bool sw_labeller_flag = false;
volatile bool sw_scroll_flag   = false;

// ============ VAC counting / sensor variables ============
volatile bool vac_counting = false;            // true when RED sensor armed
volatile uint32_t vac_pulse_counter = 0;       // ongoing vac pulse counter
volatile uint32_t saved_vac_count = 0;         // snapshot when BARCODE triggered
volatile uint32_t calculated_vac_target = 0;   // computed target value
volatile bool barcode_calc_pending = false;    // flag to trigger calculation in main loop
volatile bool barcode_locked = false;          // prevent multiple barcode triggers during processing
volatile uint32_t last_sensor_trigger_us = 0;  // timestamp of last label gap sensor trigger
uint32_t barcode_sensor_guard_ms = 100; // guard window in ms: ignore barcode if sensor fired within this window
uint32_t min_vac_for_barcode = 0;      // minimum VAC pulses after RED before accepting barcode (0=disabled)

// ============ INPUT INDICATOR TIMESTAMPS (for web UI dots) ============
volatile uint32_t red_sensor_last_ms = 0;
volatile uint32_t red_trigger_count = 0;  // total RED triggers for BPM calc
volatile uint32_t barcode_last_ms = 0;
volatile uint32_t label_sensor_last_ms = 0;
volatile uint32_t activate_last_ms = 0;
volatile uint32_t deactivate_last_ms = 0;
volatile uint32_t start_last_ms = 0;

// ============ ACQUISITION MODE ============
volatile bool acquisition_mode = false;
volatile uint32_t acq_vac_at_barcode = 0;       // VAC pulses at barcode detection
volatile uint32_t acq_vac_at_label_start = 0;   // VAC pulses when label starts
volatile uint32_t acq_vac_at_label_contact = 0; // VAC pulses when label gap sensor fires
volatile uint32_t acq_cycle_count = 0;           // increments each complete cycle
// VAC mm/pulse = (50.7 m/min / 23325 Hz) * (1000 mm/m / 60 s/min) = 0.03623 mm/pulse
constexpr float VAC_MM_PER_PULSE = 50.7f * 1000.0f / (23325.0f * 60.0f);
// Conv2 mm/pulse = (25.04 m/min / 1244 Hz) * (1000 mm/m / 60 s/min) = 0.33545 mm/pulse
constexpr float CONV2_MM_PER_PULSE = 25.04f * 1000.0f / (1244.0f * 60.0f);

// Label offset (editable from web UI) — arc distance on bottle surface in mm
float label_offset_mm = 0.0f;  // tuned once, auto-scaled by speed ratio
float ratio_adjust_pct = 0.0f; // % adjustment to auto-calculated ratio (-30 to +30)
float main_speed_trim_pct = 0.0f; // % adjustment to auto-calculated main conv speed (-50 to +50)

// ============ SERVO LABELLER VARIABLES (MCPWM + PCNT) ============
float servo_current_freq = 0.0f;      // current output frequency (ramped)
volatile uint32_t servo_pulse_count = 0; // accumulated pulse count (PCNT + overflow)
volatile int16_t pcnt_overflow_count = 0; // PCNT overflow counter
volatile bool gap_detected = false;    // set by sensorISR
volatile bool servo_running = false;   // true when MCPWM is outputting
bool home_requested = false;           // HOME button pressed
bool auto_detect_requested = false;    // Auto-detect label length
uint32_t auto_detect_pulse_start = 0;  // pulse count at start of auto-detect
uint32_t slowdown_pulses = 0;         // computed: SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT

// Calibration
bool calibrate_requested = false;
float calibrate_distance_mm = 60.0f;  // commanded distance for calibration
uint32_t calibrate_pulses = 0;        // pulses to send = distance × SERVO_PULSES_PER_MM
float SERVO_CALIB_SPEED_HZ = 5000.0f; // slow calibration speed (Hz)

// ============ CONVEYOR VARIABLES (4 independent) ===========
// Target frequencies in Hz (set from web UI)
float convvac_target_freq = 2000.0f;
float conv2_target_freq = 2000.0f;
float scroll_target_freq = 2000.0f;
float convmain_target_freq = 2000.0f;

// Current actual frequencies (smoothed by accel/decel)
float convvac_current_freq = 0.0f;
float conv2_current_freq = 0.0f;
float scroll_current_freq = 0.0f;
float convmain_current_freq = 0.0f;

// Last frequency written to LEDC (avoid reconfiguring timer when unchanged)
float convvac_written_freq = -1.0f;
float conv2_written_freq = -1.0f;
float scroll_written_freq = -1.0f;
float convmain_written_freq = -1.0f;

// VAC pulse accumulator (for software counting of output pulses)
volatile float vac_pulse_accumulator = 0.0f;

// Each conveyor accel/decel (Hz/s), editable
float CONVVAC_ACCEL = 80000.0f;
float CONVVAC_DECEL = 50000.0f;
float CONV2_ACCEL = 80000.0f;
float CONV2_DECEL = 50000.0f;
float SCROLL_ACCEL = 80000.0f;
float SCROLL_DECEL = 50000.0f;
float CONVMAIN_ACCEL = 80000.0f;
float CONVMAIN_DECEL = 50000.0f;

// ============ TIMERS ============
hw_timer_t *precision_timer=nullptr;  // 50µs target check timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // Spinlock for ISR critical sections

// ============ I2C HELPERS (PCF8574 on KC868-A16) ============
// Read X01-X08 inputs. Returns byte where bit=0 means input active (optocoupler ON).
uint8_t pcf_read_inputs(){
  Wire.requestFrom(PCF_INPUT_1_8, (uint8_t)1);
  if(Wire.available()) return Wire.read();
  return 0xFF;  // all inactive on error
}

// Write Y01-Y08 outputs. Bit=0 means MOSFET ON (active-low through optocoupler).
void pcf_write_outputs(uint8_t val){
  Wire.beginTransmission(PCF_OUTPUT_1_8);
  Wire.write(val);
  Wire.endTransmission();
}

// Set a single output bit (active-low: on=true clears the bit)
void pcf_set_output(uint8_t bit_pos, bool on){
  if(on){
    pcf_output_state &= ~(1 << bit_pos);  // clear bit = MOSFET ON
  } else {
    pcf_output_state |= (1 << bit_pos);   // set bit = MOSFET OFF
  }
  pcf_write_outputs(pcf_output_state);
}

// Poll I2C buttons with debounce (called from loop)
void poll_i2c_buttons(){
  static uint32_t last_poll_ms = 0;
  static uint8_t prev_inputs = 0xFF;
  uint32_t now_ms = millis();
  if(now_ms - last_poll_ms < 50) return;  // poll every 50ms
  last_poll_ms = now_ms;

  uint8_t inputs = pcf_read_inputs();
  // Detect falling edges (transition from inactive=1 to active=0)
  uint8_t fell = prev_inputs & ~inputs;
  prev_inputs = inputs;

  if(fell & (1 << BIT_ACTIVATE))   { activate_flag = true; activate_last_ms = now_ms; }
  if(fell & (1 << BIT_DEACTIVATE)) { deactivate_flag = true; deactivate_last_ms = now_ms; }
  if(fell & (1 << BIT_START)){
    start_last_ms = now_ms;
    if(system_active && labeller_enable && state == IDLE)
      start_flag = true;
  }
  if(fell & (1 << BIT_SW_LABELLER))  sw_labeller_flag = true;
  if(fell & (1 << BIT_SW_SCROLL))    sw_scroll_flag = true;
}

// ============ INTERRUPTS ============

// Gap sensor ISR — detects end of label (FALLING edge = label→gap)
// Only active during RUNNING state. Stops MCPWM instantly.
void IRAM_ATTR sensorISR(){
  if(!system_active || !labeller_enable) return;
  if(state != RUNNING && state != AUTO_DETECT) return;

  static uint32_t last_trigger_us = 0;
  uint32_t now = micros();
  if(now - last_trigger_us < 5000) return;  // 5ms debounce

  gap_detected = true;
  // Kill MCPWM output instantly
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);

  if(acquisition_mode){
    acq_vac_at_label_contact = vac_pulse_counter;
    acq_cycle_count++;
  }
  last_sensor_trigger_us = now;
  label_sensor_last_ms = millis();
  last_trigger_us = now;
}

// Toggle switch ISRs (with debounce)
// swLabellerISR removed — now polled via I2C (poll_i2c_buttons, X04)
// swScrollISR removed — now polled via I2C (poll_i2c_buttons, X05)


// ============ NEW SENSORS ISRs ============
void IRAM_ATTR redSensorISR(){
  // FIX: Only count when system is active AND labeller is enabled
  if(!system_active) return;      // Respect DEACTIVATE button
  if(!labeller_enable) return;    // Respect labeller toggle OFF

  static uint32_t last=0;
  uint32_t now=micros();
  if(now - last > DEBOUNCE_SENSOR_US && digitalRead(PIN_REDSENSOR)==(red_rising ? HIGH : LOW)){
    // FIX: Critical section prevents task's counter update from overwriting this reset
    portENTER_CRITICAL_ISR(&timerMux);
    vac_counting = true;           // start counting vac pulses
    vac_pulse_counter = 0;         // reset ongoing counter
    vac_pulse_accumulator = 0.0f;  // reset fractional accumulator
    saved_vac_count = 0;
    calculated_vac_target = 0;
    barcode_locked = false;        // Reset lock for new cycle
    portEXIT_CRITICAL_ISR(&timerMux);
    red_sensor_last_ms = millis();
    red_trigger_count++;
    last = now;
  }
}

void IRAM_ATTR barcodeISR(){
  // FIX: Only detect when system is active AND labeller is enabled
  if(!system_active) return;      // Respect DEACTIVATE button
  if(!labeller_enable) return;    // Respect labeller toggle OFF

  static uint32_t last=0;
  uint32_t now=micros();

  // FIX #1: Only accept barcode if armed by RED sensor AND not already processing
  if(!vac_counting || barcode_locked) return;
   // Ignore barcode if not enough VAC pulses since RED sensor (filters bottle barcode interference)
  if(min_vac_for_barcode > 0 && vac_pulse_counter < min_vac_for_barcode) return;

  // Ignore barcode if label sensor fired within guard window (near-simultaneous trigger)
  if(last_sensor_trigger_us != 0 && (now - last_sensor_trigger_us) < (barcode_sensor_guard_ms * 1000UL)) return;

  if(now - last > DEBOUNCE_SENSOR_US && digitalRead(PIN_BARCODE)==(barcode_rising ? HIGH : LOW)){
    // FIX #3: Lock to prevent multiple triggers during processing
    barcode_locked = true;

    // Simple snapshot - no heavy computation in ISR!
    saved_vac_count = vac_pulse_counter;
    if(acquisition_mode) acq_vac_at_barcode = vac_pulse_counter;
    barcode_calc_pending = true;  // Signal main loop to do calculation
    barcode_last_ms = millis();

    last = now;
  }
}

// PCNT overflow ISR — fires when counter reaches 30000
void IRAM_ATTR pcnt_overflow_isr(void *arg){
  pcnt_overflow_count++;
  // clear interrupt flag via register (PCNT peripheral)
  REG_WRITE(0x3FF57004, BIT(PCNT_UNIT_0));  // PCNT_INT_CLR_REG
}

// Get total pulse count (overflow × 30000 + current counter)
uint32_t get_servo_pulses(){
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  return (uint32_t)pcnt_overflow_count * 30000 + (uint32_t)count;
}

// ============ PRECISION ISR (fires every 50µs) ============
// Lightweight: target check + gap sensor polling only (no float math)
void IRAM_ATTR precision_check_isr(){
  // CREEP: poll gap sensor for next label (HIGH = label → stop)
  if(state == CREEP){
    if(digitalRead(PIN_SENSOR) == HIGH){
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
      gap_detected = true;
    }
  }

  // HOMING: poll gap sensor for gap (LOW = gap found → stop)
  if(state == HOMING){
    if(digitalRead(PIN_SENSOR) == LOW){
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
      gap_detected = true;
    }
  }

  // Target checker
  if(calculated_vac_target == 0) return;
  if(!vac_counting) return;
  if(!system_active) return;
  if(!labeller_enable) return;

  if(vac_pulse_counter >= calculated_vac_target){
    portENTER_CRITICAL_ISR(&timerMux);
    if(state == IDLE){
      start_flag = true;
      if(acquisition_mode) acq_vac_at_label_start = vac_pulse_counter;
    }
    vac_counting = false;
    calculated_vac_target = 0;
    barcode_locked = false;
    portEXIT_CRITICAL_ISR(&timerMux);
  }
}

// ============ SERVO HELPERS ============
void servo_start(float freq){
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_overflow_count = 0;
  servo_pulse_count = 0;
  pcnt_counter_resume(PCNT_UNIT_0);
  servo_current_freq = freq;
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, (uint32_t)freq);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0f);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  servo_running = true;
  Serial.print("MCPWM started at ");
  Serial.print((uint32_t)freq);
  Serial.println(" Hz");
}

void servo_stop(){
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  servo_running = false;
  servo_current_freq = 0;
}

void servo_set_freq(float freq){
  if(freq < 1.0f){
    servo_stop();
    return;
  }
  servo_current_freq = freq;
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, (uint32_t)freq);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0f);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

// ============ WEB SERVER TASK (runs on Core 0) ============
void task_web_server(void*){
  for(;;){
    server.handleClient();
    vTaskDelay(1);
  }
}

// ============ CONVEYOR TASK (4 conveyors) ============
void task_conveyor_control(void*){
  static uint32_t last_us_task = micros();
  for(;;){
    uint32_t now_us_task = micros();
    // Handle micros() overflow correctly (wraps every ~71 minutes)
    uint32_t elapsed_us = now_us_task - last_us_task;
    if(elapsed_us == 0) { vTaskDelay(0); continue; }  // Skip if no time passed
    float dt = elapsed_us / 1000000.0f;
    // Sanity check: if dt is absurdly large (system stall/debugger), reset tracking
    if(dt > 1.0f){
      dt = 0.01f;  // Use small dt
      vac_pulse_accumulator = 0.0f;  // Reset accumulator to avoid losing counts
    }
    last_us_task = now_us_task;

    auto updateConv = [dt](float target, float &current, float accel, float decel, int channel, float &written){
      if(target > 0){
        if(current < target)
          current = min(target, current + accel * dt);
        else
          current = max(target, current - decel * dt);
      } else {
        current = max(0.0f, current - decel * dt);
      }

      float freq_out = (current > 1) ? current : 0;
      // Only reconfigure LEDC when frequency actually changes (avoids timer glitches)
      if(fabsf(freq_out - written) > 0.5f){
        ledcWriteTone(channel, freq_out);
        written = freq_out;
      }
    };

    // VAC and CONV2: Always run when system is active
    float t1 = (system_active && convvac_enable) ? convvac_target_freq : 0.0f;
    float t2 = (system_active && conv2_enable) ? conv2_target_freq : 0.0f;
    float t3 = (system_active && scroll_enable) ? scroll_target_freq : 0.0f;
    float t4 = (system_active && convmain_enable) ? convmain_target_freq : 0.0f;

    updateConv(t1, convvac_current_freq, CONVVAC_ACCEL, CONVVAC_DECEL, 0, convvac_written_freq);  // LEDC ch 0
    updateConv(t2, conv2_current_freq, CONV2_ACCEL, CONV2_DECEL, 2, conv2_written_freq);        // LEDC ch 2
    updateConv(t3, scroll_current_freq, SCROLL_ACCEL, SCROLL_DECEL, 4, scroll_written_freq);     // LEDC ch 4
    updateConv(t4, convmain_current_freq, CONVMAIN_ACCEL, CONVMAIN_DECEL, 6, convmain_written_freq); // LEDC ch 6

    // Count VAC output pulses: accumulate pulses = frequency × time
    if(vac_counting && convvac_current_freq > 0.0f){
      float increment = convvac_current_freq * dt;
      portENTER_CRITICAL(&timerMux);
      vac_pulse_accumulator += increment;
      if(vac_pulse_accumulator >= 1.0f){
        uint32_t whole_pulses = (uint32_t)vac_pulse_accumulator;
        vac_pulse_counter += whole_pulses;
        vac_pulse_accumulator -= (float)whole_pulses;
      }
      portEXIT_CRITICAL(&timerMux);
    }

    // Safety: barcode timeout
    if(vac_counting && calculated_vac_target == 0 && vac_pulse_counter > MAX_VAC_PULSES_NO_BARCODE){
      Serial.println("WARNING: Barcode timeout - resetting VAC counting sequence");
      portENTER_CRITICAL(&timerMux);
      vac_counting = false;
      vac_pulse_counter = 0;
      vac_pulse_accumulator = 0.0f;
      saved_vac_count = 0;
      barcode_locked = false;
      portEXIT_CRITICAL(&timerMux);
    }

    // ============ SERVO LABELLER MOTOR CONTROL (MCPWM + PCNT) ============
    if(state == ACCEL || state == RUNNING || state == CREEP){
      // Read hardware pulse count
      servo_pulse_count = get_servo_pulses();

      if(state == ACCEL){
        // Ramp up using high speed acceleration
        servo_current_freq = min(SERVO_RUN_SPEED_HZ, servo_current_freq + SERVO_HS_ACCEL * dt);
        servo_set_freq(servo_current_freq);
        if(servo_current_freq >= SERVO_RUN_SPEED_HZ){
          state = RUNNING;
          Serial.println("Servo: RUNNING");
        }
      }
      else if(state == RUNNING){
        // Check if pulse count reached slowdown threshold or gap detected
        bool do_creep = false;
        if(servo_pulse_count >= slowdown_pulses){
          do_creep = true;
          Serial.print("Servo: Slowdown at pulses=");
          Serial.println(servo_pulse_count);
        }
        if(gap_detected){
          gap_detected = false;
          do_creep = true;
          Serial.print("Servo: Gap detected at pulses=");
          Serial.println(servo_pulse_count);
        }
        if(do_creep){
          // Start decelerating from run speed toward creep speed
          state = CREEP;
        }
        // Safety: max pulses
        if(servo_pulse_count > SERVO_MAX_PULSES){
          Serial.println("ERROR: Max pulses exceeded");
          servo_stop();
          state = IDLE;
        }
      }
      else if(state == CREEP){
        // Ramp down from current speed to creep speed using HS decel
        if(servo_current_freq > SERVO_BASE_SPEED_HZ){
          servo_current_freq = max(SERVO_BASE_SPEED_HZ, servo_current_freq - SERVO_HS_DECEL * dt);
          servo_set_freq(servo_current_freq);
        }
        // Precision ISR polls sensor at 50µs and sets gap_detected + kills MCPWM
        if(gap_detected){
          gap_detected = false;
          // Ramp down from creep to stop using LS decel
          if(SERVO_LS_DECEL > 0 && servo_current_freq > 100.0f){
            // Let it decel over next ticks — for now stop (LS decel is very fast at creep speed)
            servo_current_freq = max(0.0f, servo_current_freq - SERVO_LS_DECEL * 0.01f);
          }
          servo_stop();
          Serial.print("Servo: STOP at pulses=");
          Serial.println(servo_pulse_count);
          state = IDLE;
          start_flag = false;
          sensor_triggered = false;
        }
        // Safety timeout: if creeping too long
        if(servo_pulse_count > SERVO_MAX_PULSES){
          Serial.println("ERROR: Creep exceeded max pulses");
          servo_stop();
          state = IDLE;
        }
      }
    }
    // HOMING mode
    else if(state == HOMING){
      servo_pulse_count = get_servo_pulses();
      if(gap_detected){
        servo_stop();
        gap_detected = false;
        Serial.println("HOME: Gap found, label indexed");
        state = IDLE;
      }
      if(servo_pulse_count > SERVO_MAX_PULSES * 2){
        servo_stop();
        Serial.println("HOME: Timeout - no gap found");
        state = IDLE;
      }
    }
    // AUTO-DETECT mode
    else if(state == AUTO_DETECT){
      servo_pulse_count = get_servo_pulses();
      if(gap_detected){
        gap_detected = false;
        uint32_t measured = servo_pulse_count - auto_detect_pulse_start;
        if(auto_detect_pulse_start == 0){
          // First gap found — start measuring
          auto_detect_pulse_start = servo_pulse_count;
          Serial.println("AUTO-DETECT: First gap found, measuring...");
          gap_detected = false;
          // Restart MCPWM at base speed
          servo_set_freq(SERVO_BASE_SPEED_HZ);
        } else {
          // Second gap found — measurement complete
          SERVO_LABEL_LENGTH = measured;
          slowdown_pulses = (uint32_t)(SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT);
          SERVO_MAX_PULSES = (uint32_t)(SERVO_LABEL_LENGTH * 1.3f);
          servo_stop();
          Serial.print("AUTO-DETECT: Label length = ");
          Serial.print(measured);
          Serial.print(" pulses = ");
          Serial.print((float)measured / SERVO_PULSES_PER_MM);
          Serial.println(" mm");
          state = IDLE;
          // Save to NVS
          prefs.begin("params", false);
          prefs.putUInt("lablen", SERVO_LABEL_LENGTH);
          prefs.end();
        }
      }
      if(servo_pulse_count > SERVO_MAX_PULSES * 3){
        servo_stop();
        Serial.println("AUTO-DETECT: Timeout");
        state = IDLE;
      }
    }
    // CALIBRATION mode — move exact number of pulses at slow speed
    else if(state == CALIBRATING){
      servo_pulse_count = get_servo_pulses();
      if(servo_pulse_count >= calibrate_pulses){
        servo_stop();
        Serial.print("CALIBRATE: Done. Pulses=");
        Serial.print(servo_pulse_count);
        Serial.print(" commanded_mm=");
        Serial.println(calibrate_distance_mm);
        state = IDLE;
      }
    }

    // FIX: Moved outside if(vac_counting) block so target_was_active always updates
    // Prevents stale value from causing false cleanup and losing pulses on new cycle
    static bool target_was_active = false;
    bool target_active = (calculated_vac_target > 0);

    // Detect falling edge: target was set, now cleared (ISR triggered!)
    if(target_was_active && !target_active){
      // ISR already set start_flag, just cleanup counters
      Serial.print("Ultra-precision trigger: Target reached at ");
      Serial.print(vac_pulse_counter);
      Serial.println(" pulses, starting labeller");

      // Reset counters (ISR already reset vac_counting and calculated_vac_target)
      vac_pulse_counter = 0;
      vac_pulse_accumulator = 0.0f;
      // Don't reset saved_vac_count - it's for display/logging
    }
    target_was_active = target_active;

    vTaskDelay(1);  // 1 tick = 1ms; micros()-based dt gives sub-ms precision for pulse accumulation
  }
}

// ============ WEB PAGE ============
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Rompack R&D in Packaging</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html,body { background: #1e1e1e; color: #d4d4d4; }
    body { font-family: 'Segoe UI', Arial, sans-serif; margin: 10px; }
    h2 { margin-top: 10px; text-align: center; color: #9a9a9a; letter-spacing: 2px; font-weight: 400; }
    .row { display: flex; gap: 10px; }
    .col { flex: 0.7; min-width: 0; }
    .col:last-child { flex: 1.6; }
    .group { margin-bottom: 10px; padding: 8px; border: 1px solid #505050; border-radius: 6px; background: #2d2d2d; }
    label { display: block; margin-bottom: 2px; font-size: 13px; color: #9a9a9a; }
    input[type=number] { width: 65%; padding: 3px; margin-bottom: 1px; box-sizing: border-box; font-size: 13px; background: #3c3c3c; border: 1px solid #505050; color: #d4d4d4; border-radius: 3px; }
    .apply-btn { display:none; }
    .blue-bg { background-color: #1a2a3a; color: #5cadff; }
    .en-btn { padding: 10px 28px; font-size: 16px; font-weight: bold; cursor: pointer; border: 1px solid #505050; border-radius: 5px; float: right; }
    .en-btn.on { background: #1a4a1e; border-color: #4ec94e; color: #4ec94e; }
    .en-btn.off { background: #3c3c3c; border-color: #f44747; color: #f44747; }
    strong { font-size: 14px; color: #d4d4d4; }
    p { margin: 3px 0; font-size: 13px; color: #9a9a9a; }
    .slot-btn { padding:4px 10px; font-size:12px; font-weight:bold; cursor:pointer; border:1px solid #505050; border-radius:4px; background:#3c3c3c; color:#5cadff; margin-bottom:2px; }
    .slot-btn:active { background:#1a2a3a; }
    .slot-btn.load { background:#2a1a4a; border-color:#bc8cff; color:#bc8cff; }
    .slot-btn.load:active { background:#3a2a5a; }
    #kbApply { display:none; position:fixed; right:10px; z-index:10000; padding:12px 30px; font-size:22px; font-weight:bold; background:#1a4a1e; border:2px solid #4ec94e; color:#4ec94e; border-radius:8px; cursor:pointer; box-shadow:0 4px 12px rgba(0,0,0,0.6); }
    #kbApply:active { background:#0d2a10; }
  </style>
</head>
<body>
  <a href="/" style="position:fixed;top:10px;left:10px;z-index:100;padding:12px 24px;font-size:20px;background:#3c3c3c;color:#5cadff;border:1px solid #5cadff;border-radius:8px;text-decoration:none;box-shadow:0 4px 12px rgba(0,0,0,0.5);">&#9664; Back</a>
  <button id="kbApply">APPLY</button>
  <h2>Rompack R&D in Packaging</h2>
  <div class="row">
    <div class="col">

    <div class="group">
      <strong>Labeller</strong>
      <button class="en-btn" id="labBtn" onclick="tog('lab_en','labBtn')">...</button>
      <div style="clear:both"></div>
      <table style="width:100%;font-size:13px;border-collapse:collapse;">
      <tr><td>Pulses per mm</td><td><input type="number" step="0.01" name="servoppmm" value="%SERVOPPMM%" class="blue-bg"><button class="apply-btn" onclick="ap('servoppmm',this)">Apply</button></td></tr>
      <tr><td>Label Length (pulses)</td><td><input type="number" step="1" name="lablen" value="%LABLEN%" class="blue-bg"><button class="apply-btn" onclick="ap('lablen',this)">Apply</button> <span style="font-size:12px;color:#9a9a9a;" id="labLenMm">%LABLENMM% mm</span></td></tr>
      <tr><td>Run Speed (Hz)</td><td><input type="number" step="100" name="servorun" value="%SERVORUN%" class="blue-bg"><button class="apply-btn" onclick="ap('servorun',this)">Apply</button> <span style="font-size:12px;color:#9a9a9a;">%SERVORUNMM% mm/s</span></td></tr>
      <tr><td>HS Accel (Hz/s)</td><td><input type="number" step="1000" name="hsacc" value="%HSACC%" class="blue-bg"><button class="apply-btn" onclick="ap('hsacc',this)">Apply</button></td></tr>
      <tr><td>HS Decel (Hz/s)</td><td><input type="number" step="1000" name="hsdec" value="%HSDEC%" class="blue-bg"><button class="apply-btn" onclick="ap('hsdec',this)">Apply</button></td></tr>
      <tr><td>LS Accel (Hz/s)</td><td><input type="number" step="1000" name="lsacc" value="%LSACC%" class="blue-bg"><button class="apply-btn" onclick="ap('lsacc',this)">Apply</button></td></tr>
      <tr><td>LS Decel (Hz/s)</td><td><input type="number" step="1000" name="lsdec" value="%LSDEC%" class="blue-bg"><button class="apply-btn" onclick="ap('lsdec',this)">Apply</button></td></tr>
      <tr><td>Creep Speed (Hz)</td><td><input type="number" step="100" name="servobase" value="%SERVOBASE%" class="blue-bg"><button class="apply-btn" onclick="ap('servobase',this)">Apply</button> <span style="font-size:12px;color:#9a9a9a;">%SERVOBASEMM% mm/s</span></td></tr>
      <tr><td>Slowdown %</td><td><input type="number" step="1" min="80" max="99" name="slowpct" value="%SLOWPCT%" class="blue-bg"><button class="apply-btn" onclick="ap('slowpct',this)">Apply</button></td></tr>
      <tr><td>Safety Max Pulses</td><td><input type="number" step="100" name="servomaxp" value="%SERVOMAXP%" class="blue-bg"><button class="apply-btn" onclick="ap('servomaxp',this)">Apply</button></td></tr>
      </table>
      <div style="margin-top:10px;padding:8px;background:#2a2a2a;border:1px solid #444;border-radius:6px;">
        <strong style="color:#e0c040;">Calibration</strong>
        <table style="width:100%;font-size:13px;border-collapse:collapse;margin-top:4px;">
        <tr><td>Index Distance (mm)</td><td>
          <input type="number" step="0.1" min="1" max="500" id="calDist" value="60" style="width:70px;" class="blue-bg">
          <button class="en-btn on" onclick="fetch('/set?calibrate='+document.getElementById('calDist').value)" style="padding:5px 12px;">INDEX</button>
        </td></tr>
        <tr><td>Measured Distance (mm)</td><td>
          <input type="number" step="0.01" min="0.1" max="500" id="calMeasured" value="" placeholder="enter real mm" style="width:100px;" class="blue-bg">
          <button class="en-btn on" onclick="calcCalib()" style="padding:5px 12px;">CALCULATE</button>
        </td></tr>
        </table>
        <div id="calibResult" style="margin-top:6px;font-size:12px;color:#9a9a9a;display:none;">
          <div style="background:#1a1a2e;padding:8px;border-radius:4px;border:1px solid #555;">
            <div>Commanded: <b id="calCmd">--</b> mm | Measured: <b id="calMeas">--</b> mm</div>
            <div>Current pulses/mm: <b id="calCurPPM">--</b></div>
            <div>Corrected pulses/mm: <b style="color:#4ec9b0;" id="calNewPPM">--</b></div>
            <div style="margin-top:8px;color:#e0c040;">
              <b>P100S Drive Parameters (PA Group):</b>
              <table style="width:100%;font-size:12px;margin-top:4px;border-collapse:collapse;">
              <tr style="border-bottom:1px solid #444;"><td>PA4</td><td>Control mode</td><td><b>0</b> (position)</td></tr>
              <tr style="border-bottom:1px solid #444;"><td>PA14</td><td>Pulse input mode</td><td><b>0</b> (pulse+dir)</td></tr>
              <tr style="border-bottom:1px solid #444;"><td>PA19</td><td>Smooth filter</td><td><b>0</b> (disabled)</td></tr>
              <tr style="border-bottom:1px solid #444;"><td>PA53</td><td>Force servo enable</td><td><b>1</b></td></tr>
              <tr style="border-bottom:1px solid #444;">
                <td>PA11</td><td>Pulses/rev</td>
                <td>Current: <b id="calPA11old">--</b> → Corrected: <b style="color:#4ec9b0;" id="calPA11new">--</b></td>
              </tr>
              </table>
              <div style="margin-top:6px;font-size:11px;color:#888;">
                <b>Option A (recommended):</b> Keep PA11=<span id="calPA11keep">--</span>, change ESP32 pulses/mm<br>
                <b>Option B (drive-side):</b> Set PA11=<b style="color:#4ec9b0;" id="calPA11alt">--</b>, keep ESP32 pulses/mm=<span id="calPPMkeep">--</span>
              </div>
              <div style="margin-top:4px;font-size:11px;color:#666;">
                PA12/PA13 e-gear: only used if PA11=0. PA78/PA79: alternative e-gear numerators (not needed).
              </div>
            </div>
            <div style="margin-top:6px;">
              <button class="en-btn on" onclick="applyCalib()" style="padding:5px 12px;">APPLY Option A (update ESP32 pulses/mm)</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="group">
      <strong>Barcode Timeout</strong>
      <label>Max VAC pulses for barcode:</label>
      <input type="number" step="100" name="maxvacnobarcode" value="%MAXVACNOBARCODE%">
      <button class="apply-btn" onclick="ap('maxvacnobarcode',this)">Apply</button>
      <p style="font-size: 11px; color: #9a9a9a;">Resets if barcode not detected within this many pulses.</p>
    </div>

    <div class="group">
      <strong>RED / BARCODE Logic</strong>
      <label>Guard Window (ms):</label>
      <input type="number" step="1" min="0" max="5000" name="guardms" value="%GUARDMS%">
      <button class="apply-btn" onclick="ap('guardms',this)">Apply</button>
      <p style="font-size: 11px; color: #9a9a9a;">Ignore barcode if label sensor fired within this window.</p>
      <label>Min VAC pulses before Barcode:</label>
      <input type="number" step="1" min="0" max="50000" name="minvacbc" value="%MINVACBC%">
      <button class="apply-btn" onclick="ap('minvacbc',this)">Apply</button>
      <p style="font-size: 11px; color: #9a9a9a;">Ignore barcode if fewer VAC pulses since RED sensor. 0=disabled. (27.6 pulses/mm)</p>
      <label>Label Offset (mm arc on bottle):</label>
      <input type="number" step="0.01" min="0" max="300" name="labeloff" value="%LABELOFF%" class="blue-bg">
      <button class="apply-btn" onclick="ap('labeloff',this)">Apply</button>
      <p style="font-size: 11px; color: #9a9a9a;">Ratio: <span id="effRatio">--</span> | Z: <span id="effPulses">--</span> pulses</p>
      <label>Barcode Trigger Edge:</label>
      <button id="btnBcEdgeSett" class="en-btn" onclick="togBcEdgeSett()">BARCODE: RISING</button>
      <label>Red Sensor Edge:</label>
      <button id="btnRedEdgeSett" class="en-btn" onclick="togRedEdgeSett()">RED: FALLING</button>
      <label>Gap Sensor Edge:</label>
      <button id="btnGapEdgeSett" class="en-btn" onclick="togGapEdgeSett()">GAP: RISING</button>
    </div>

    <div class="group">
      <strong>Live Status</strong>
      <p>System Active: <span id="systemActive" style="font-weight: bold;">%SYSTEMACTIVE%</span></p>
      <p>Vac counter: <span id="vacCount">%VACCOUNT%</span></p>
      <p>Saved VAC count: <span id="savedVac">%SAVEDVAC%</span></p>
      <p>Calculated target: <span id="vacTarget">%VACTARGET%</span></p>
      <p>State: <span id="stateLabel">%STATELABEL%</span></p>
    </div>

    </div>
    <div class="col">

    <div class="group">
      <strong>Scroll</strong>
      <button class="en-btn" id="scrollBtn" onclick="tog('scroll_en','scrollBtn')">...</button>
      <div style="clear:both"></div>
      <label>Target freq (Hz):</label>
      <input type="number" step="10" name="scrollfreq" value="%SCROLLFREQ%" class="blue-bg">
      <button class="apply-btn" onclick="ap('scrollfreq',this)">Apply</button>
      <label>Accel (Hz/s):</label>
      <input type="number" step="1000" name="scrollacc" value="%SCROLLACC%">
      <button class="apply-btn" onclick="ap('scrollacc',this)">Apply</button>
      <label>Decel (Hz/s):</label>
      <input type="number" step="1000" name="scrolldec" value="%SCROLLDEC%">
      <button class="apply-btn" onclick="ap('scrolldec',this)">Apply</button>
    </div>

    <div class="group">
      <strong>All Conveyors</strong>
      <button class="en-btn" id="allConvBtn" onclick="togAllConv()">...</button>
      <div style="clear:both"></div>
    </div>

    <div class="group">
      <strong>Conveyor VAC</strong>
      <button class="en-btn" id="cvacBtn2" onclick="tog('cvac_en','cvacBtn2')">...</button>
      <label>Speed (m/min):</label>
      <input type="number" step="0.01" id="cvacMmin" value="%CVACMMIN%" oninput="cvacMminToHz()" class="blue-bg">
      <button class="apply-btn" onclick="apCvacMmin(this)">Apply</button>
      <label>Target freq (Hz):</label>
      <input type="number" step="10" name="cvacfreq" value="%CVACFREQ%" oninput="cvacHzToMmin()">
      <button class="apply-btn" onclick="ap('cvacfreq',this)">Apply</button>
      <label>Accel (Hz/s):</label>
      <input type="number" step="1000" name="cvacacc" value="%CVACACC%">
      <button class="apply-btn" onclick="ap('cvacacc',this)">Apply</button>
      <label>Decel (Hz/s):</label>
      <input type="number" step="1000" name="cvacdec" value="%CVACDEC%">
      <button class="apply-btn" onclick="ap('cvacdec',this)">Apply</button>
    </div>

    <div class="group">
      <strong>Conveyor 2</strong>
      <button class="en-btn" id="c2Btn2" onclick="tog('c2_en','c2Btn2')">...</button>
      <label>Speed (m/min):</label>
      <input type="number" step="0.01" id="c2Mmin" value="%C2MMIN%" oninput="c2MminToHz()" class="blue-bg">
      <button class="apply-btn" onclick="apC2Mmin(this)">Apply</button>
      <label>Target freq (Hz):</label>
      <input type="number" step="10" name="c2freq" value="%C2FREQ%" oninput="c2HzToMmin()">
      <button class="apply-btn" onclick="ap('c2freq',this)">Apply</button>
      <label>Accel (Hz/s):</label>
      <input type="number" step="1000" name="c2acc" value="%C2ACC%">
      <button class="apply-btn" onclick="ap('c2acc',this)">Apply</button>
      <label>Decel (Hz/s):</label>
      <input type="number" step="1000" name="c2dec" value="%C2DEC%">
      <button class="apply-btn" onclick="ap('c2dec',this)">Apply</button>
    </div>

    <div class="group">
      <strong>Conveyor Main</strong>
      <button class="en-btn" id="cmainBtn2" onclick="tog('cmain_en','cmainBtn2')">...</button>
      <label>Speed (m/min):</label>
      <input type="number" step="0.01" id="convmainMmin" value="%CONVMAINMMIN%" oninput="cmMminToHz()" class="blue-bg">
      <button class="apply-btn" onclick="apCmMmin(this)">Apply</button>
      <label>Target freq (Hz):</label>
      <input type="number" step="10" name="convmainfreq" value="%CONVMAINFREQ%" oninput="cmHzToMmin()">
      <button class="apply-btn" onclick="ap('convmainfreq',this)">Apply</button>
      <label>Accel (Hz/s):</label>
      <input type="number" step="1000" name="convmainacc" value="%CONVMAINACC%">
      <button class="apply-btn" onclick="ap('convmainacc',this)">Apply</button>
      <label>Decel (Hz/s):</label>
      <input type="number" step="1000" name="convmaindec" value="%CONVMAINDEC%">
      <button class="apply-btn" onclick="ap('convmaindec',this)">Apply</button>
      <p>Runs automatically when system is activated.</p>
    </div>

    </div>
    <div class="col">

    <div class="group">
      <strong>Data Acquisition</strong>
      <button class="en-btn" id="acqBtn2" onclick="toggleAcq2()" style="min-width:160px;">...</button>
      <div style="float:right;margin-left:4px;">
        <button class="en-btn off" onclick="resetAcq2()" style="min-width:60px;background:#FF9800;">Reset</button>
      </div>
      <div style="float:right;margin-right:8px;">
        <div style="display:flex;gap:3px;">
          <div><button class="slot-btn" onclick="saveAcqSlot(1)">S1</button><br><button class="slot-btn load" onclick="loadAcqSlot(1)">L1</button></div>
          <div><button class="slot-btn" onclick="saveAcqSlot(2)">S2</button><br><button class="slot-btn load" onclick="loadAcqSlot(2)">L2</button></div>
          <div><button class="slot-btn" onclick="saveAcqSlot(3)">S3</button><br><button class="slot-btn load" onclick="loadAcqSlot(3)">L3</button></div>
          <div><button class="slot-btn" onclick="saveAcqSlot(4)">S4</button><br><button class="slot-btn load" onclick="loadAcqSlot(4)">L4</button></div>
        </div>
      </div>
      <div style="clear:both"></div>
      <div style="overflow-x:auto;">
        <table id="acqTable2" style="width:100%;border-collapse:collapse;font-size:13px;margin-top:6px;">
          <tr style="background:#21262d;color:#8b949e;"><th style="padding:4px;">Bottle</th><th>Barcode (mm)</th><th>Label Start (mm)</th><th>Label Contact (mm)</th><th>Value</th></tr>
        </table>
      </div>
    </div>

    </div>
  </div>
  <script>
    function ap(name, btn) {
      var val = document.querySelector('input[name="'+name+'"]').value;
      fetch('/set?'+name+'='+encodeURIComponent(val))
        .then(function(r){ btn.classList.add('ok'); btn.textContent='OK'; setTimeout(function(){ btn.classList.remove('ok'); btn.textContent='Apply'; },1000); })
        .catch(function(){ btn.textContent='Error'; setTimeout(function(){ btn.textContent='Apply'; },1500); });
    }
    function calcCalib(){
      var cmd = parseFloat(document.getElementById('calDist').value);
      var meas = parseFloat(document.getElementById('calMeasured').value);
      if(!meas || meas < 0.1){ alert('Enter the measured distance'); return; }
      var curPPM = %SERVOPPMM%;
      var newPPM = cmd * curPPM / meas;
      // PA11 = pulses per motor revolution (default 10000)
      // mm_per_rev = PI * roller_dia / gear_ratio = PI * 53 / 5 = 33.301 mm
      var mmPerRev = Math.PI * 53.0 / 5.0;
      var pa11current = Math.round(curPPM * mmPerRev);
      // Option A: keep PA11, change ESP32 pulses/mm
      // Option B: keep ESP32 pulses/mm, change PA11
      var pa11new = Math.round(newPPM * mmPerRev);
      document.getElementById('calCmd').textContent = cmd.toFixed(1);
      document.getElementById('calMeas').textContent = meas.toFixed(2);
      document.getElementById('calCurPPM').textContent = curPPM.toFixed(2);
      document.getElementById('calNewPPM').textContent = newPPM.toFixed(2);
      document.getElementById('calPA11old').textContent = pa11current;
      document.getElementById('calPA11new').textContent = pa11new;
      document.getElementById('calPA11keep').textContent = pa11current;
      document.getElementById('calPA11alt').textContent = pa11new;
      document.getElementById('calPPMkeep').textContent = curPPM.toFixed(2);
      document.getElementById('calibResult').style.display = 'block';
      window._calNewPPM = newPPM;
    }
    function applyCalib(){
      if(!window._calNewPPM){ alert('Calculate first'); return; }
      fetch('/set?servoppmm=' + window._calNewPPM.toFixed(4))
        .then(function(){ alert('Pulses/mm updated to ' + window._calNewPPM.toFixed(2)); location.reload(); })
        .catch(function(){ alert('Error'); });
    }
    function tog(param, btnId) {
      var btn = document.getElementById(btnId);
      var cur = btn.classList.contains('on');
      var nv = cur ? 0 : 1;
      fetch('/set?'+param+'='+nv).then(function(){ updBtn(btnId, !cur); });
    }
    function togBcEdgeSett(){
      var btn=document.getElementById('btnBcEdgeSett');
      var cur=btn.dataset.rising==='1';
      fetch('/set?barcode_edge='+(cur?0:1));
    }
    function togRedEdgeSett(){
      var btn=document.getElementById('btnRedEdgeSett');
      var cur=btn.dataset.rising==='1';
      fetch('/set?red_edge='+(cur?0:1));
    }
    function togGapEdgeSett(){
      var btn=document.getElementById('btnGapEdgeSett');
      var cur=btn.dataset.rising==='1';
      fetch('/set?gap_edge='+(cur?0:1));
    }
    function togAllConv() {
      var btn = document.getElementById('allConvBtn');
      var cur = btn.classList.contains('on');
      var nv = cur ? 0 : 1;
      fetch('/set?cvac_en='+nv+'&c2_en='+nv+'&cmain_en='+nv)
        .then(function(){ updBtn('allConvBtn', !cur); });
    }
    function updBtn(id, on) {
      var b = document.getElementById(id);
      b.className = 'en-btn ' + (on ? 'on' : 'off');
      b.textContent = on ? 'ENABLED' : 'DISABLED';
    }
    var cvacRatio = 50.7 / 23325;
    function cvacMminToHz() {
      var mmin = parseFloat(document.getElementById('cvacMmin').value);
      if(!isNaN(mmin)) document.querySelector('input[name="cvacfreq"]').value = Math.round(mmin / cvacRatio);
    }
    function cvacHzToMmin() {
      var hz = parseFloat(document.querySelector('input[name="cvacfreq"]').value);
      if(!isNaN(hz)) document.getElementById('cvacMmin').value = (hz * cvacRatio).toFixed(2);
    }
    function apCvacMmin(btn) {
      cvacMminToHz();
      var hz = document.querySelector('input[name="cvacfreq"]').value;
      fetch('/set?cvacfreq='+encodeURIComponent(hz))
        .then(function(r){ btn.classList.add('ok'); btn.textContent='OK'; setTimeout(function(){ btn.classList.remove('ok'); btn.textContent='Apply'; },1000); })
        .catch(function(){ btn.textContent='Error'; setTimeout(function(){ btn.textContent='Apply'; },1500); });
    }
    var c2Ratio = 25.04 / 1244;
    function c2MminToHz() {
      var mmin = parseFloat(document.getElementById('c2Mmin').value);
      if(!isNaN(mmin)) document.querySelector('input[name="c2freq"]').value = Math.round(mmin / c2Ratio);
    }
    function c2HzToMmin() {
      var hz = parseFloat(document.querySelector('input[name="c2freq"]').value);
      if(!isNaN(hz)) document.getElementById('c2Mmin').value = (hz * c2Ratio).toFixed(2);
    }
    function apC2Mmin(btn) {
      c2MminToHz();
      var hz = document.querySelector('input[name="c2freq"]').value;
      fetch('/set?c2freq='+encodeURIComponent(hz))
        .then(function(r){ btn.classList.add('ok'); btn.textContent='OK'; setTimeout(function(){ btn.classList.remove('ok'); btn.textContent='Apply'; },1000); })
        .catch(function(){ btn.textContent='Error'; setTimeout(function(){ btn.textContent='Apply'; },1500); });
    }
    var cmRatio = 31.6 / 5946;
    function cmMminToHz() {
      var mmin = parseFloat(document.getElementById('convmainMmin').value);
      if(!isNaN(mmin)) document.querySelector('input[name="convmainfreq"]').value = Math.round(mmin / cmRatio);
    }
    function cmHzToMmin() {
      var hz = parseFloat(document.querySelector('input[name="convmainfreq"]').value);
      if(!isNaN(hz)) document.getElementById('convmainMmin').value = (hz * cmRatio).toFixed(2);
    }
    function apCmMmin(btn) {
      cmMminToHz();
      var hz = document.querySelector('input[name="convmainfreq"]').value;
      fetch('/set?convmainfreq='+encodeURIComponent(hz))
        .then(function(r){ btn.classList.add('ok'); btn.textContent='OK'; setTimeout(function(){ btn.classList.remove('ok'); btn.textContent='Apply'; },1000); })
        .catch(function(){ btn.textContent='Error'; setTimeout(function(){ btn.textContent='Apply'; },1500); });
    }
    document.querySelectorAll('input[type=number]').forEach(function(el){ el.addEventListener('focus', function(){ this.select(); }); });

    function updateEffPulses(data) {
      var ep = document.getElementById('effPulses');
      if(ep && data && data.z_pulses !== undefined) ep.textContent = data.z_pulses;
      var er = document.getElementById('effRatio');
      if(er && data && data.ratio !== undefined) er.textContent = data.ratio;
    }

    // Keyboard APPLY button logic
    var kbBtn = document.getElementById('kbApply');
    var activeInput = null;
    function positionKbBtn(){
      if(!activeInput) return;
      if(window.visualViewport){
        var vv = window.visualViewport;
        kbBtn.style.bottom = (window.innerHeight - vv.height - vv.offsetTop + 10) + 'px';
      } else {
        kbBtn.style.bottom = '10px';
      }
    }
    if(window.visualViewport){
      window.visualViewport.addEventListener('resize', positionKbBtn);
      window.visualViewport.addEventListener('scroll', positionKbBtn);
    }
    document.querySelectorAll('input[type=number]').forEach(function(el){
      el.addEventListener('focus', function(){
        activeInput = this;
        kbBtn.style.display = 'block';
        positionKbBtn();
      });
    });
    kbBtn.addEventListener('click', function(){
      if(!activeInput) return;
      var sibling = activeInput.nextElementSibling;
      if(sibling && sibling.classList.contains('apply-btn')){
        sibling.click();
      }
      kbBtn.style.display = 'none';
      activeInput.blur();
      activeInput = null;
    });
    document.addEventListener('click', function(e){
      if(e.target === kbBtn) return;
      if(e.target.tagName === 'INPUT') return;
      if(e.target.classList && e.target.classList.contains('apply-btn')) return;
      kbBtn.style.display = 'none';
      activeInput = null;
    });

    // Auto-refresh live values every second
    setInterval(function() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('vacCount').textContent = data.vac_count;
          document.getElementById('vacTarget').textContent = data.vac_target;
          document.getElementById('savedVac').textContent = data.saved_vac;
          var stateLabels = ['IDLE','ACCEL','RUNNING','CREEP','HOMING','AUTO-DETECT','CALIBRATING'];
          document.getElementById('stateLabel').textContent = stateLabels[data.state] || 'UNKNOWN';

          var sysActive = document.getElementById('systemActive');
          sysActive.textContent = data.system_active ? 'ON' : 'OFF';
          sysActive.style.color = data.system_active ? 'green' : 'red';

          updBtn('labBtn', data.lab_en);
          updBtn('scrollBtn', data.scroll_en);
          updBtn('allConvBtn', data.cvac_en && data.c2_en && data.cmain_en);
          updBtn('cvacBtn2', data.cvac_en);
          updBtn('c2Btn2', data.c2_en);
          updBtn('cmainBtn2', data.cmain_en);
          var bbs=document.getElementById('btnBcEdgeSett');
          if(bbs){ bbs.dataset.rising=data.bc_rising?'1':'0'; bbs.className='en-btn '+(data.bc_rising?'on':'off'); bbs.textContent='BARCODE: '+(data.bc_rising?'RISING':'FALLING'); }
          var brs=document.getElementById('btnRedEdgeSett');
          if(brs){ brs.dataset.rising=data.red_rising?'1':'0'; brs.className='en-btn '+(data.red_rising?'on':'off'); brs.textContent='RED: '+(data.red_rising?'RISING':'FALLING'); }
          var bgs=document.getElementById('btnGapEdgeSett');
          if(bgs){ bgs.dataset.rising=data.gap_rising?'1':'0'; bgs.className='en-btn '+(data.gap_rising?'on':'off'); bgs.textContent='GAP: '+(data.gap_rising?'RISING':'FALLING'); }

          acqOn2 = data.acq_mode;
          var ab2 = document.getElementById('acqBtn2');
          ab2.className = 'en-btn ' + (acqOn2 ? 'on' : 'off');
          ab2.textContent = 'Acquisition: ' + (acqOn2 ? 'ON' : 'OFF');
          updateEffPulses(data);
          if(data.acq_cycle > lastCycle2 && data.acq_cycle > 0 && bottleCount2 < 100){
            lastCycle2 = data.acq_cycle;
            bottleCount2++;
            var tbl2 = document.getElementById('acqTable2');
            var row2 = tbl2.insertRow(-1);
            row2.innerHTML = '<td style="padding:4px;border-bottom:1px solid #ddd;">Bottle ' + bottleCount2 + '</td>' +
              '<td style="padding:4px;border-bottom:1px solid #ddd;">' + data.acq_barcode_mm + '</td>' +
              '<td style="padding:4px;border-bottom:1px solid #ddd;">' + data.acq_start_mm + '</td>' +
              '<td style="padding:4px;border-bottom:1px solid #ddd;">' + data.acq_contact_mm + '</td>' +
              '<td style="padding:4px;border-bottom:1px solid #ddd;"><input type="text" inputmode="decimal" style="width:60px;font-size:12px;padding:2px;" class="acq-val"></td>';
          }
        })
        .catch(err => console.error('Status fetch failed:', err));
    }, 1000);

    var acqOn2 = false;
    var lastCycle2 = 0;
    var bottleCount2 = 0;
    function toggleAcq2(){
      acqOn2 = !acqOn2;
      fetch('/set?acq_mode=' + (acqOn2 ? 1 : 0));
      var ab2 = document.getElementById('acqBtn2');
      ab2.className = 'en-btn ' + (acqOn2 ? 'on' : 'off');
      ab2.textContent = 'Acquisition: ' + (acqOn2 ? 'ON' : 'OFF');
    }
    function resetAcq2(){
      fetch('/set?acq_reset=1');
      lastCycle2 = 0;
      bottleCount2 = 0;
      var tbl2 = document.getElementById('acqTable2');
      while(tbl2.rows.length > 1) tbl2.deleteRow(1);
    }
    function saveAcqSlot(slot){
      var tbl2 = document.getElementById('acqTable2');
      var rows = [];
      for(var i=1; i<tbl2.rows.length; i++){
        var r = tbl2.rows[i];
        var val = r.querySelector('.acq-val');
        rows.push({
          b: r.cells[0].textContent,
          bc: r.cells[1].textContent,
          st: r.cells[2].textContent,
          ct: r.cells[3].textContent,
          v: val ? val.value : ''
        });
      }
      localStorage.setItem('acqSlot'+slot, JSON.stringify(rows));
      alert('Saved to slot '+slot+' ('+rows.length+' rows)');
    }
    function loadAcqSlot(slot){
      var data = localStorage.getItem('acqSlot'+slot);
      if(!data){ alert('Slot '+slot+' is empty'); return; }
      var rows = JSON.parse(data);
      var tbl2 = document.getElementById('acqTable2');
      while(tbl2.rows.length > 1) tbl2.deleteRow(1);
      bottleCount2 = rows.length;
      lastCycle2 = 0;
      for(var i=0; i<rows.length; i++){
        var row2 = tbl2.insertRow(-1);
        row2.innerHTML = '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].b+'</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].bc+'</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].st+'</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].ct+'</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;"><input type="text" inputmode="decimal" style="width:60px;font-size:12px;padding:2px;" class="acq-val" value="'+rows[i].v+'"></td>';
      }
    }
  </script>
</body>
</html>
)rawliteral";

String fmtFloat(float v, int decimals) {
  char buf[32];
  dtostrf(v,0,decimals,buf);
  return String(buf);
}

// ============ NVS SAVE / LOAD ============
void saveParams(){
  if(!prefs.begin("params", false)){
    Serial.println("ERROR: NVS open for write failed!");
    return;
  }
  prefs.putFloat("servoppmm", SERVO_PULSES_PER_MM);
  prefs.putFloat("servorun", SERVO_RUN_SPEED_HZ);
  prefs.putFloat("hsacc", SERVO_HS_ACCEL);
  prefs.putFloat("hsdec", SERVO_HS_DECEL);
  prefs.putFloat("lsacc", SERVO_LS_ACCEL);
  prefs.putFloat("lsdec", SERVO_LS_DECEL);
  prefs.putFloat("servobase", SERVO_BASE_SPEED_HZ);
  prefs.putUInt("lablen", SERVO_LABEL_LENGTH);
  prefs.putFloat("slowpct", SERVO_SLOWDOWN_PCT);
  prefs.putUInt("servomaxp", SERVO_MAX_PULSES);
  prefs.putUInt("maxvacnobc", MAX_VAC_PULSES_NO_BARCODE);
  prefs.putFloat("cvacfreq", convvac_target_freq);
  prefs.putFloat("cvacacc", CONVVAC_ACCEL);
  prefs.putFloat("cvacdec", CONVVAC_DECEL);
  prefs.putFloat("c2freq", conv2_target_freq);
  prefs.putFloat("c2acc", CONV2_ACCEL);
  prefs.putFloat("c2dec", CONV2_DECEL);
  prefs.putFloat("scrollfreq", scroll_target_freq);
  prefs.putFloat("scrollacc", SCROLL_ACCEL);
  prefs.putFloat("scrolldec", SCROLL_DECEL);
  prefs.putFloat("cmainfreq", convmain_target_freq);
  prefs.putFloat("cmainacc", CONVMAIN_ACCEL);
  prefs.putFloat("cmaindec", CONVMAIN_DECEL);
  prefs.putFloat("labeloff", label_offset_mm);
  prefs.putFloat("rtrim", ratio_adjust_pct);
  prefs.putFloat("mstrim", main_speed_trim_pct);
  prefs.putUInt("guardms", barcode_sensor_guard_ms);
  prefs.putUInt("minvacbc", min_vac_for_barcode);
  prefs.putBool("lab_en", labeller_enable);
  prefs.putBool("cvac_en", convvac_enable);
  prefs.putBool("c2_en", conv2_enable);
  prefs.putBool("scroll_en", scroll_enable);
  prefs.putBool("cmain_en", convmain_enable);
  prefs.putBool("bc_rising", barcode_rising);
  prefs.putBool("red_rising", red_rising);
  prefs.putBool("gap_rising", gap_rising);
  prefs.end();
  Serial.println("NVS: Parameters saved");
}

void loadParams(){
  if(!prefs.begin("params", true)){
    Serial.println("NVS: Open failed, using defaults");
    return;
  }
  SERVO_PULSES_PER_MM = prefs.getFloat("servoppmm", SERVO_PULSES_PER_MM);
  SERVO_RUN_SPEED_HZ  = prefs.getFloat("servorun", SERVO_RUN_SPEED_HZ);
  SERVO_HS_ACCEL      = prefs.getFloat("hsacc", SERVO_HS_ACCEL);
  SERVO_HS_DECEL      = prefs.getFloat("hsdec", SERVO_HS_DECEL);
  SERVO_LS_ACCEL      = prefs.getFloat("lsacc", SERVO_LS_ACCEL);
  SERVO_LS_DECEL      = prefs.getFloat("lsdec", SERVO_LS_DECEL);
  SERVO_BASE_SPEED_HZ = prefs.getFloat("servobase", SERVO_BASE_SPEED_HZ);
  SERVO_LABEL_LENGTH  = prefs.getUInt("lablen", SERVO_LABEL_LENGTH);
  SERVO_SLOWDOWN_PCT  = prefs.getFloat("slowpct", SERVO_SLOWDOWN_PCT);
  SERVO_MAX_PULSES    = prefs.getUInt("servomaxp", SERVO_MAX_PULSES);
  slowdown_pulses = (uint32_t)(SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT);
  MAX_VAC_PULSES_NO_BARCODE = prefs.getUInt("maxvacnobc", MAX_VAC_PULSES_NO_BARCODE);
  convvac_target_freq    = prefs.getFloat("cvacfreq", convvac_target_freq);
  CONVVAC_ACCEL          = prefs.getFloat("cvacacc", CONVVAC_ACCEL);
  CONVVAC_DECEL          = prefs.getFloat("cvacdec", CONVVAC_DECEL);
  conv2_target_freq      = prefs.getFloat("c2freq", conv2_target_freq);
  CONV2_ACCEL            = prefs.getFloat("c2acc", CONV2_ACCEL);
  CONV2_DECEL            = prefs.getFloat("c2dec", CONV2_DECEL);
  scroll_target_freq     = prefs.getFloat("scrollfreq", scroll_target_freq);
  SCROLL_ACCEL           = prefs.getFloat("scrollacc", SCROLL_ACCEL);
  SCROLL_DECEL           = prefs.getFloat("scrolldec", SCROLL_DECEL);
  convmain_target_freq   = prefs.getFloat("cmainfreq", convmain_target_freq);
  CONVMAIN_ACCEL         = prefs.getFloat("cmainacc", CONVMAIN_ACCEL);
  CONVMAIN_DECEL         = prefs.getFloat("cmaindec", CONVMAIN_DECEL);
  label_offset_mm        = prefs.getFloat("labeloff", label_offset_mm);
  ratio_adjust_pct       = prefs.getFloat("rtrim", ratio_adjust_pct);
  main_speed_trim_pct    = prefs.getFloat("mstrim", main_speed_trim_pct);
  barcode_sensor_guard_ms = prefs.getUInt("guardms", barcode_sensor_guard_ms);
  min_vac_for_barcode    = prefs.getUInt("minvacbc", min_vac_for_barcode);
  labeller_enable        = prefs.getBool("lab_en", false);
  convvac_enable         = prefs.getBool("cvac_en", false);
  conv2_enable           = prefs.getBool("c2_en", false);
  scroll_enable          = prefs.getBool("scroll_en", false);
  convmain_enable        = prefs.getBool("cmain_en", false);
  barcode_rising         = prefs.getBool("bc_rising", true);
  red_rising             = prefs.getBool("red_rising", false);
  gap_rising             = prefs.getBool("gap_rising", true);
  prefs.end();
  Serial.print("NVS: Loaded OK — VAC freq=");
  Serial.print(convvac_target_freq);
  Serial.print(" LabelOffset=");
  Serial.print(label_offset_mm);
  Serial.println("mm");
}

void handleHome() {
  String html = R"rawhtml(<!DOCTYPE html><html><head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Rompack R&D in Packaging</title>
  <style>
    * { box-sizing: border-box; }
    html { background: #1e1e1e; }
    body { font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 8px; background: #1e1e1e; color: #d4d4d4; }
    h2 { text-align: center; margin: 4px 0 10px 0; font-size: 18px; letter-spacing: 3px; color: #9a9a9a; font-weight: 400; text-transform: uppercase; }
    .panel { background: #2d2d2d; border: 1px solid #505050; border-left: 3px solid #5cadff; border-radius: 3px; padding: 5px 10px; margin-bottom: 3px; display: flex; align-items: center; justify-content: space-between; }
    .panel-label { font-size: 13px; font-weight: 600; color: #d4d4d4; letter-spacing: 0.5px; }
    .en-btn { padding: 6px 14px; font-size: 12px; font-weight: bold; cursor: pointer; border: 1px solid #505050; border-radius: 3px; min-width: 90px; letter-spacing: 0.5px; transition: all 0.15s; }
    .en-btn.on { background: #1a4a1e; border-color: #4ec94e; color: #4ec94e; }
    .en-btn.off { background: #3c3c3c; border-color: #f44747; color: #f44747; }
    .nav-btn { position: fixed; padding: 10px 20px; font-size: 14px; background: #3c3c3c; color: #5cadff; border: 1px solid #5cadff; border-radius: 4px; cursor: pointer; text-decoration: none; z-index: 100; letter-spacing: 1px; }
    .nav-btn:active { background: #2d2d2d; }
    .slot-section { margin-top: 10px; background: #2d2d2d; border: 1px solid #505050; border-radius: 4px; padding: 10px; }
    .slot-section strong { font-size: 11px; display: block; margin-bottom: 6px; color: #9a9a9a; letter-spacing: 1px; text-transform: uppercase; }
    .slot-row { display: flex; flex-wrap: wrap; gap: 4px; margin-bottom: 6px; }
    .slot-btn { padding: 8px 4px; font-size: 11px; font-weight: bold; cursor: pointer; border: 1px solid #505050; border-radius: 3px; color: #d4d4d4; flex: 1; min-width: 44px; text-align: center; background: #3c3c3c; }
    .slot-save { background: #3a2800; border-color: #ffcc00; color: #ffcc00; }
    .slot-save:active { background: #4a3400; }
    .slot-load { background: #1a2a3a; border-color: #5cadff; color: #5cadff; }
    .slot-load:active { background: #1a3050; }
    .slot-btn.ok { background: #1a4a1e; border-color: #4ec94e; color: #4ec94e; }
    .main-row { display: flex; gap: 8px; align-items: flex-start; }
    .main-left { flex: 0.2; min-width: 0; }
    .main-right { flex: 0.8; min-width: 0; }
    .act-section { text-align: center; margin: 8px 0; }
    .act-btn { padding: 14px 24px; font-size: 15px; font-weight: bold; cursor: pointer; border: 2px solid #505050; border-radius: 4px; background: #3c3c3c; color: #9a9a9a; margin: 4px; transition: all 0.15s; letter-spacing: 1px; text-transform: uppercase; box-shadow: 0 2px 4px rgba(0,0,0,0.5); }
    .act-btn.activate { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.activate.lit { background: #1a4a1e; border-color: #4ec94e; color: #4ec94e; box-shadow: 0 0 12px rgba(78,201,78,0.4); }
    .act-btn.deactivate { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.deactivate.lit { background: #4a1a1a; border-color: #f44747; color: #f44747; box-shadow: 0 0 12px rgba(244,71,71,0.4); }
    .act-btn.manual { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.manual.lit { background: #3a2800; border-color: #ffcc00; color: #ffcc00; box-shadow: 0 0 12px rgba(255,204,0,0.4); }
    .act-btn.labhead { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.labhead.lit { background: #1a2a3a; border-color: #5cadff; color: #5cadff; box-shadow: 0 0 12px rgba(92,173,255,0.4); }
    .act-btn.scroll { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.scroll.lit { background: #2a1a4a; border-color: #bc8cff; color: #bc8cff; box-shadow: 0 0 12px rgba(188,140,255,0.4); }
    .act-btn.bcedge { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.bcedge.lit { background: #1a3a3a; border-color: #4ec94e; color: #4ec94e; box-shadow: 0 0 12px rgba(78,201,78,0.4); }
    .act-btn.rededge { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.rededge.lit { background: #4a1a1a; border-color: #f44747; color: #f44747; box-shadow: 0 0 12px rgba(244,71,71,0.4); }
    .act-btn.gapedge { background: #3c3c3c; border-color: #505050; color: #9a9a9a; }
    .act-btn.gapedge.lit { background: #1a2a3a; border-color: #5cadff; color: #5cadff; box-shadow: 0 0 12px rgba(92,173,255,0.4); }
    .status-indicator { font-size: 20px; font-weight: bold; margin: 8px 0; padding: 10px; background: #2d2d2d; border: 1px solid #505050; border-radius: 4px; text-align: center; letter-spacing: 2px; font-family: monospace; }
    .status-indicator.active { color: #4ec94e; border-color: #4ec94e; }
    .status-indicator.inactive { color: #f44747; border-color: #f44747; }
    .io-panel { background: #2d2d2d; border: 1px solid #505050; border-radius: 4px; padding: 8px; margin: 6px 0; }
    .io-panel strong { display: block; font-size: 10px; margin-bottom: 6px; color: #9a9a9a; letter-spacing: 1px; text-transform: uppercase; }
    .io-row { display: flex; flex-wrap: wrap; justify-content: center; gap: 8px; }
    .io-dot { display: flex; flex-direction: column; align-items: center; gap: 2px; }
    .io-dot .dot { width: 18px; height: 18px; border-radius: 50%; background: #3c3c3c; border: 2px solid #505050; }
    .io-dot .dot.on { background: #4ec94e; border-color: #4ec94e; box-shadow: 0 0 6px rgba(78,201,78,0.8); }
    .io-dot .lbl { font-size: 9px; color: #9a9a9a; text-align: center; }
    .sens-panel { display: flex; gap: 6px; margin: 6px 0; }
    .sens-ind { flex: 1; padding: 12px 4px; text-align: center; font-size: 14px; font-weight: bold; border-radius: 4px; background: #2d2d2d; color: #9a9a9a; border: 2px solid #505050; transition: all 0.2s; letter-spacing: 1px; }
    .sens-ind.red-on { background: #4a1a1a; border-color: #f44747; color: #f44747; box-shadow: 0 0 14px rgba(244,71,71,0.6); }
    .sens-ind.bar-on { background: #3a2800; border-color: #ffcc00; color: #ffcc00; box-shadow: 0 0 14px rgba(255,204,0,0.6); }
    .sens-ind.gap-on { background: #1a2a3a; border-color: #5cadff; color: #5cadff; box-shadow: 0 0 14px rgba(92,173,255,0.6); }
    table { border-collapse: collapse; width: 100%; font-size: 12px; }
    th { background: #3c3c3c; color: #9a9a9a; padding: 6px 4px; border-bottom: 1px solid #505050; letter-spacing: 0.5px; }
    td { padding: 4px; border-bottom: 1px solid #3c3c3c; color: #d4d4d4; }
    input[type=text] { background: #3c3c3c; border: 1px solid #505050; color: #d4d4d4; border-radius: 3px; padding: 2px 4px; }
  </style>
</head><body>
  <a class="nav-btn" style="top:10px;left:10px;" href="/settings">Settings &#9881;</a>
  <h2>Rompack R&D in Packaging</h2>
  <div class="main-row">
  <div class="main-left">
  <div class="panel"><span class="panel-label">Labeller</span>
    <button class="en-btn" id="labBtn" onclick="tog('lab_en','labBtn')">...</button></div>
  <div class="panel"><span class="panel-label">Scroll</span>
    <button class="en-btn" id="scrollBtn" onclick="tog('scroll_en','scrollBtn')">...</button></div>
  <div class="panel"><span class="panel-label">All Conveyors</span>
    <button class="en-btn" id="allConvBtn" onclick="togAllConv()">...</button></div>
  <div class="panel"><span class="panel-label">Conveyor Main</span>
    <button class="en-btn" id="cmainBtn" onclick="tog('cmain_en','cmainBtn')">...</button></div>
  <div class="panel"><span class="panel-label">Conveyor VAC</span>
    <button class="en-btn" id="cvacBtn" onclick="tog('cvac_en','cvacBtn')">...</button></div>
  <div class="panel"><span class="panel-label">Conveyor 2</span>
    <button class="en-btn" id="c2Btn" onclick="tog('c2_en','c2Btn')">...</button></div>
  <div class="io-panel">
    <strong>Inputs</strong>
    <div class="io-row">
      <div class="io-dot"><div class="dot" id="dAct"></div><div class="lbl">ACTIVATE<br>X01</div></div>
      <div class="io-dot"><div class="dot" id="dDeact"></div><div class="lbl">DEACTIVATE<br>X02</div></div>
      <div class="io-dot"><div class="dot" id="dStart"></div><div class="lbl">ONE LABEL<br>X03</div></div>
      <div class="io-dot"><div class="dot" id="dSwLab"></div><div class="lbl">LAB ON/OFF<br>X04</div></div>
      <div class="io-dot"><div class="dot" id="dScr"></div><div class="lbl">SCROLL<br>X05</div></div>
    </div>
  </div>
  <div class="sens-panel">
    <div class="sens-ind" id="sRed">RED</div>
    <div class="sens-ind" id="sBar">BAR</div>
    <div class="sens-ind" id="sGap">GAP</div>
  </div>
  </div>
  <div class="main-right">
  <div class="act-section">
    <div class="status-indicator" id="sysStatus">---</div>
    <div style="text-align:center;margin-bottom:8px;padding:6px;background:#2d2d2d;border:1px solid #505050;border-radius:4px;">
      <span style="font-size:12px;color:#9a9a9a;letter-spacing:1px;">BOTTLES/MIN</span>
      <span id="bpmVal" style="font-size:22px;font-weight:bold;color:#5cadff;margin-left:8px;">0</span>
    </div>
    <button id="btnAct" class="act-btn activate" onclick="flashBtn('btnAct');fetch('/set?activate=1').then(function(){pollStatus();})">ACTIVATE</button>
    <button id="btnDeact" class="act-btn deactivate" onclick="flashBtn('btnDeact');fetch('/set?deactivate=1').then(function(){pollStatus();})">DEACTIVATE</button>
    <button id="btnManual" class="act-btn manual" onclick="flashBtn('btnManual');fetch('/set?manual_label=1').then(function(){pollStatus();})">ONE LABEL</button>
    <button id="btnLabHead" class="act-btn labhead" onclick="togAct('lab_en','btnLabHead')">LABEL HEAD OFF</button>
    <button id="btnScroll" class="act-btn scroll" onclick="togAct('scroll_en','btnScroll')">SCROLL OFF</button>
    <button id="btnBcEdge" class="act-btn bcedge" onclick="togBcEdge()">BARCODE: RISING</button>
    <button id="btnRedEdge" class="act-btn rededge" onclick="togRedEdge()">RED: FALLING</button>
    <button id="btnGapEdge" class="act-btn gapedge" onclick="togGapEdge()">GAP: RISING</button>
  </div>
  <div class="io-panel" style="margin:8px 0;">
    <strong>Label Offset</strong>
    <div style="display:flex;align-items:center;gap:6px;margin-top:4px;">
      <input type="number" step="0.01" min="0" max="300" id="homeOffset" style="width:80px;padding:6px;font-size:16px;background:#3c3c3c;border:1px solid #505050;color:#5cadff;border-radius:3px;">
      <span style="font-size:13px;color:#9a9a9a;">mm</span>
      <button class="en-btn on" id="offApply" onclick="applyOffset()" style="padding:6px 14px;">APPLY</button>
      <span style="font-size:12px;color:#9a9a9a;">Z:<span id="homeEffP">--</span>p</span>
    </div>
  </div>
  <div class="io-panel" style="margin:8px 0;">
    <strong>Ratio Adjust</strong>
    <div style="display:flex;align-items:center;gap:6px;margin-top:4px;">
      <input type="number" step="0.1" min="-30" max="30" id="homeTrim" style="width:80px;padding:6px;font-size:16px;background:#3c3c3c;border:1px solid #505050;color:#d29922;border-radius:3px;">
      <span style="font-size:13px;color:#9a9a9a;">%</span>
      <button class="en-btn on" id="trimApply" onclick="applyTrim()" style="padding:6px 14px;">APPLY</button>
      <span style="font-size:12px;color:#9a9a9a;">R=<span id="homeRatio">--</span></span>
    </div>
  </div>
  <div class="io-panel" style="margin:8px 0;">
    <strong>Main Conv Speed Trim</strong>
    <div style="display:flex;align-items:center;gap:6px;margin-top:4px;">
      <input type="number" step="0.5" min="-50" max="50" id="homeMsTrim" style="width:80px;padding:6px;font-size:16px;background:#3c3c3c;border:1px solid #505050;color:#3fb950;border-radius:3px;">
      <span style="font-size:13px;color:#9a9a9a;">%</span>
      <button class="en-btn on" id="msTrimApply" onclick="applyMsTrim()" style="padding:6px 14px;">APPLY</button>
      <span style="font-size:12px;color:#9a9a9a;">Speed: <span id="homeBotSpeed">--</span> m/min</span>
    </div>
  </div>
  <div class="io-panel" style="margin:8px 0;">
    <strong>Servo Labeller</strong>
    <div style="display:flex;align-items:center;gap:6px;margin-top:4px;">
      <span style="font-size:13px;">Label: <span id="homeLabelLen">%LABLEN%</span> pulses (<span id="homeLabelMm">%LABLENMM%</span> mm)</span>
    </div>
    <div style="display:flex;align-items:center;gap:6px;margin-top:4px;">
      <button class="en-btn on" onclick="fetch('/set?home=1')" style="padding:6px 14px;">HOME</button>
      <button class="en-btn on" onclick="if(confirm('Auto-detect label length?'))fetch('/set?autodetect=1')" style="padding:6px 14px;">AUTO-DETECT</button>
      <span style="font-size:12px;color:#9a9a9a;">Pulses: <span id="homeServoPulses">--</span></span>
    </div>
  </div>
  <div class="slot-section">
    <strong>Save Settings to Country:</strong>
    <div class="slot-row" id="saveRow"></div>
    <strong>Load Settings from Country:</strong>
    <div class="slot-row" id="loadRow"></div>
  </div>
  <div class="slot-section" style="margin-top:12px;">
    <strong>Data Acquisition</strong>
    <div style="margin:6px 0;">
      <button class="en-btn" id="acqBtn" onclick="toggleAcq()" style="min-width:160px;">...</button>
      <button class="en-btn off" onclick="resetAcq()" style="min-width:80px;background:#FF9800;">Reset</button>
      <button class="en-btn off" onclick="emailAcq()" style="min-width:80px;background:#2196F3;">Email</button>
    </div>
    <div style="overflow-x:auto;">
      <table id="acqTable" style="width:100%;border-collapse:collapse;font-size:13px;margin-top:6px;">
        <tr style="background:#21262d;color:#8b949e;"><th style="padding:4px;">Bottle</th><th>Barcode (mm)</th><th>Label Start (mm)</th><th>Label Contact (mm)</th><th>Value</th></tr>
      </table>
    </div>
  </div>
  </div>
  </div>

<script>
  function updBtn(id, on) {
    var b = document.getElementById(id);
    b.className = 'en-btn ' + (on ? 'on' : 'off');
    b.textContent = on ? 'ENABLED' : 'DISABLED';
  }
  function tog(param, btnId) {
    var btn = document.getElementById(btnId);
    var cur = btn.classList.contains('on');
    var nv = cur ? 0 : 1;
    fetch('/set?'+param+'='+nv).then(function(){ updBtn(btnId, !cur); });
  }
  function togAllConv() {
    var btn = document.getElementById('allConvBtn');
    var cur = btn.classList.contains('on');
    var nv = cur ? 0 : 1;
    fetch('/set?cvac_en='+nv+'&c2_en='+nv+'&cmain_en='+nv)
      .then(function(){
        updBtn('allConvBtn', !cur);
        updBtn('cvacBtn', !cur);
        updBtn('c2Btn', !cur);
        updBtn('cmainBtn', !cur);
      });
  }
  // Build Save and Load button rows with custom names
  var saveRow = document.getElementById('saveRow');
  var loadRow = document.getElementById('loadRow');
  var slotNames = [];
  fetch('/slotnames').then(function(r){ return r.json(); }).then(function(names){
    slotNames = names;
    for(var i = 0; i < 10; i++) {
      (function(idx){
        var slot = idx + 1;
        var sb = document.createElement('button');
        sb.className = 'slot-btn slot-save';
        sb.textContent = slotNames[idx];
        sb.onclick = function(){
          var newName = prompt('Name for Country ' + slot + ':', slotNames[idx]);
          if(newName === null) return;
          if(newName === '') newName = String(slot);
          fetch('/saveslot?slot=' + slot + '&name=' + encodeURIComponent(newName)).then(function(r){ return r.text(); }).then(function(t){
            slotNames[idx] = newName;
            var tbl = document.getElementById('acqTable');
            var rows = [];
            for(var i=1; i<tbl.rows.length; i++){
              var r2 = tbl.rows[i];
              var val = r2.querySelector('.acq-val');
              rows.push({ b:r2.cells[0].textContent, bc:r2.cells[1].textContent,
                st:r2.cells[2].textContent, ct:r2.cells[3].textContent,
                v: val ? val.value : '' });
            }
            localStorage.setItem('countryAcq'+slot, JSON.stringify(rows));
            sb.classList.add('ok'); sb.textContent = 'OK';
            setTimeout(function(){ sb.classList.remove('ok'); sb.textContent = newName; }, 1500);
            lb.textContent = newName;
          }).catch(function(){ sb.textContent = 'ERR'; setTimeout(function(){ sb.textContent = slotNames[idx]; }, 1500); });
        };
        saveRow.appendChild(sb);

        var lb = document.createElement('button');
        lb.className = 'slot-btn slot-load';
        lb.textContent = slotNames[idx];
        lb.onclick = function(){
          if(confirm('Load settings from "' + slotNames[idx] + '"?')){
            fetch('/loadslot?slot=' + slot).then(function(r){ return r.text(); }).then(function(t){
              if(t === 'empty'){ alert('Country ' + slot + ' is empty. Save settings first.'); return; }
              var acqData = localStorage.getItem('countryAcq'+slot);
              if(acqData){
                var rows = JSON.parse(acqData);
                var tbl = document.getElementById('acqTable');
                while(tbl.rows.length > 1) tbl.deleteRow(1);
                bottleCount = 0;
                for(var i=0; i<rows.length; i++){
                  bottleCount++;
                  var row = tbl.insertRow(-1);
                  row.innerHTML = '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].b+'</td>'+
                    '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].bc+'</td>'+
                    '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].st+'</td>'+
                    '<td style="padding:4px;border-bottom:1px solid #ddd;">'+rows[i].ct+'</td>'+
                    '<td style="padding:4px;border-bottom:1px solid #ddd;"><input type="text" inputmode="decimal" class="acq-val" style="width:60px;padding:2px 4px;font-size:12px;border:1px solid #ccc;border-radius:3px;" value="'+(rows[i].v||'')+'"></td>';
                }
              }
              pollStatus();
              lb.classList.add('ok'); lb.textContent = 'OK';
              setTimeout(function(){ lb.classList.remove('ok'); lb.textContent = slotNames[idx]; }, 1500);
            }).catch(function(){ lb.textContent = 'ERR'; setTimeout(function(){ lb.textContent = slotNames[idx]; }, 1500); });
          }
        };
        loadRow.appendChild(lb);
      })(i);
    }
  });
  var acqOn = false;
  var lastCycle = 0;
  var bottleCount = 0;
  function toggleAcq() {
    acqOn = !acqOn;
    fetch('/set?acq_mode=' + (acqOn ? 1 : 0));
    var ab = document.getElementById('acqBtn');
    ab.className = 'en-btn ' + (acqOn ? 'on' : 'off');
    ab.textContent = 'Acquisition: ' + (acqOn ? 'ON' : 'OFF');
  }
  function resetAcq() {
    fetch('/set?acq_reset=1');
    lastCycle = 0;
    bottleCount = 0;
    var tbl = document.getElementById('acqTable');
    while(tbl.rows.length > 1) tbl.deleteRow(1);
  }
  function emailAcq() {
    var tbl = document.getElementById('acqTable');
    if(tbl.rows.length < 2){ alert('No data to email'); return; }
    var body = 'Data Acquisition Report\n\n';
    body += 'Bottle\tBarcode(mm)\tLabel Start(mm)\tLabel Contact(mm)\tValue\n';
    body += '-----\t-----------\t---------------\t-----------------\t-----\n';
    for(var i=1; i<tbl.rows.length; i++){
      var r = tbl.rows[i];
      var val = r.querySelector('.acq-val');
      body += r.cells[0].textContent + '\t' +
              r.cells[1].textContent + '\t' +
              r.cells[2].textContent + '\t' +
              r.cells[3].textContent + '\t' +
              (val ? val.value : '') + '\n';
    }
    var subject = 'Data Acquisition - ' + new Date().toLocaleDateString();
    window.location.href = 'mailto:?subject=' + encodeURIComponent(subject) + '&body=' + encodeURIComponent(body);
  }
  function flashBtn(id){
    var b=document.getElementById(id); if(!b) return;
    b.classList.add('lit');
    setTimeout(function(){ b.classList.remove('lit'); }, 500);
  }
  function togAct(param, btnId){
    var btn=document.getElementById(btnId);
    var cur=btn.classList.contains('lit');
    var nv=cur?0:1;
    fetch('/set?'+param+'='+nv).then(function(){pollStatus();});
  }
  function togBcEdge(){
    var btn=document.getElementById('btnBcEdge');
    var cur=btn.dataset.rising==='1';
    fetch('/set?barcode_edge='+(cur?0:1)).then(function(){pollStatus();});
  }
  function togRedEdge(){
    var btn=document.getElementById('btnRedEdge');
    var cur=btn.dataset.rising==='1';
    fetch('/set?red_edge='+(cur?0:1)).then(function(){pollStatus();});
  }
  function togGapEdge(){
    var btn=document.getElementById('btnGapEdge');
    var cur=btn.dataset.rising==='1';
    fetch('/set?gap_edge='+(cur?0:1)).then(function(){pollStatus();});
  }
  var offLoaded = false;
  var trimLoaded = false;
  var msTrimLoaded = false;
  function applyOffset(){
    var v = document.getElementById('homeOffset').value;
    var btn = document.getElementById('offApply');
    fetch('/set?labeloff='+encodeURIComponent(v))
      .then(function(){ btn.textContent='OK'; btn.className='en-btn on'; setTimeout(function(){ btn.textContent='APPLY'; },1000); })
      .catch(function(){ btn.textContent='ERR'; setTimeout(function(){ btn.textContent='APPLY'; },1500); });
  }
  function applyTrim(){
    var v = document.getElementById('homeTrim').value;
    var btn = document.getElementById('trimApply');
    fetch('/set?rtrim='+encodeURIComponent(v))
      .then(function(){ btn.textContent='OK'; btn.className='en-btn on'; setTimeout(function(){ btn.textContent='APPLY'; },1000); pollStatus(); })
      .catch(function(){ btn.textContent='ERR'; setTimeout(function(){ btn.textContent='APPLY'; },1500); });
  }
  function applyMsTrim(){
    var v = document.getElementById('homeMsTrim').value;
    var btn = document.getElementById('msTrimApply');
    fetch('/set?mstrim='+encodeURIComponent(v))
      .then(function(){ btn.textContent='OK'; btn.className='en-btn on'; setTimeout(function(){ btn.textContent='APPLY'; },1000); pollStatus(); })
      .catch(function(){ btn.textContent='ERR'; setTimeout(function(){ btn.textContent='APPLY'; },1500); });
  }
  function pollStatus() {
    fetch('/status').then(r => r.json()).then(data => {
      updBtn('labBtn', data.lab_en);
      updBtn('scrollBtn', data.scroll_en);
      updBtn('cvacBtn', data.cvac_en);
      updBtn('c2Btn', data.c2_en);
      updBtn('cmainBtn', data.cmain_en);
      var allOn = data.cvac_en && data.c2_en && data.cmain_en;
      updBtn('allConvBtn', allOn);
      var si = document.getElementById('sysStatus');
      if(data.system_active){ si.textContent='SYSTEM ACTIVE'; si.className='status-indicator active'; }
      else { si.textContent='SYSTEM INACTIVE'; si.className='status-indicator inactive'; }
      if(!offLoaded && data.label_off !== undefined){
        document.getElementById('homeOffset').value = data.label_off;
        offLoaded = true;
      }
      if(!trimLoaded && data.rtrim !== undefined){
        document.getElementById('homeTrim').value = data.rtrim;
        trimLoaded = true;
      }
      if(!msTrimLoaded && data.mstrim !== undefined){
        document.getElementById('homeMsTrim').value = data.mstrim;
        msTrimLoaded = true;
      }
      var hep = document.getElementById('homeEffP');
      if(hep && data.z_pulses !== undefined) hep.textContent = data.z_pulses;
      var hr = document.getElementById('homeRatio');
      if(hr && data.ratio !== undefined) hr.textContent = data.ratio;
      var bv = document.getElementById('bpmVal');
      if(bv && data.bpm !== undefined) bv.textContent = data.bpm;
      var bs = document.getElementById('homeBotSpeed');
      if(bs && data.bot_mmin !== undefined) bs.textContent = data.bot_mmin;
      var hsp = document.getElementById('homeServoPulses');
      if(hsp && data.servo_pulses !== undefined) hsp.textContent = data.servo_pulses;
      var hll = document.getElementById('homeLabelLen');
      if(hll && data.lab_len !== undefined) hll.textContent = data.lab_len;
      var hlm = document.getElementById('homeLabelMm');
      if(hlm && data.lab_len_mm !== undefined) hlm.textContent = data.lab_len_mm;
      acqOn = data.acq_mode;
      var ab = document.getElementById('acqBtn');
      ab.className = 'en-btn ' + (acqOn ? 'on' : 'off');
      ab.textContent = 'Acquisition: ' + (acqOn ? 'ON' : 'OFF');
      if(data.acq_cycle > lastCycle && data.acq_cycle > 0 && bottleCount < 100){
        lastCycle = data.acq_cycle;
        bottleCount++;
        var tbl = document.getElementById('acqTable');
        var row = tbl.insertRow(-1);
        row.innerHTML = '<td style="padding:4px;border-bottom:1px solid #ddd;">Bottle ' + bottleCount + '</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;">' + data.acq_barcode_mm + '</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;">' + data.acq_start_mm + '</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;">' + data.acq_contact_mm + '</td>' +
          '<td style="padding:4px;border-bottom:1px solid #ddd;"><input type="text" inputmode="decimal" class="acq-val" style="width:60px;padding:2px 4px;font-size:12px;border:1px solid #ccc;border-radius:3px;"></td>';
      }
      // Update input indicator dots
      function setDot(id, on){ var d=document.getElementById(id); if(d) d.className=on?'dot on':'dot'; }
      var sr=document.getElementById('sRed');
      if(sr) sr.className = data.red_sens ? 'sens-ind red-on' : 'sens-ind';
      var sb2=document.getElementById('sBar');
      if(sb2) sb2.className = data.barcode ? 'sens-ind bar-on' : 'sens-ind';
      var sg=document.getElementById('sGap');
      if(sg) sg.className = data.label_sens ? 'sens-ind gap-on' : 'sens-ind';
      setDot('dScr', data.sw_scroll);
      var ib = data.i2c_in;
      setDot('dAct',   !(ib & (1<<0)));
      setDot('dDeact', !(ib & (1<<1)));
      setDot('dStart', !(ib & (1<<2)));
      setDot('dSwLab', !(ib & (1<<3)));
      // Highlight action buttons
      var ba=document.getElementById('btnAct');
      if(ba){ if(data.system_active || data.act_btn){ ba.classList.add('lit'); ba.textContent='ACTIVATED'; } else { ba.classList.remove('lit'); ba.textContent='ACTIVATE'; } }
      var bd=document.getElementById('btnDeact');
      if(bd){ if(!data.system_active || data.deact_btn){ bd.classList.add('lit'); bd.textContent='DEACTIVATED'; } else { bd.classList.remove('lit'); bd.textContent='DEACTIVATE'; } }
      var bm=document.getElementById('btnManual');
      if(bm){ if(data.start_btn) bm.classList.add('lit'); else bm.classList.remove('lit'); }
      var bl=document.getElementById('btnLabHead');
      if(bl){ if(data.lab_en){ bl.classList.add('lit'); bl.textContent='LABEL HEAD ON'; } else { bl.classList.remove('lit'); bl.textContent='LABEL HEAD OFF'; } }
      var bs=document.getElementById('btnScroll');
      if(bs){ if(data.scroll_en){ bs.classList.add('lit'); bs.textContent='SCROLL ON'; } else { bs.classList.remove('lit'); bs.textContent='SCROLL OFF'; } }
      var bb=document.getElementById('btnBcEdge');
      if(bb){ bb.dataset.rising=data.bc_rising?'1':'0'; if(data.bc_rising){ bb.classList.add('lit'); bb.textContent='BARCODE: RISING'; } else { bb.classList.remove('lit'); bb.textContent='BARCODE: FALLING'; } }
      var br=document.getElementById('btnRedEdge');
      if(br){ br.dataset.rising=data.red_rising?'1':'0'; if(data.red_rising){ br.classList.add('lit'); br.textContent='RED: RISING'; } else { br.classList.remove('lit'); br.textContent='RED: FALLING'; } }
      var bg=document.getElementById('btnGapEdge');
      if(bg){ bg.dataset.rising=data.gap_rising?'1':'0'; if(data.gap_rising){ bg.classList.add('lit'); bg.textContent='GAP: RISING'; } else { bg.classList.remove('lit'); bg.textContent='GAP: FALLING'; } }
    });
  }
  pollStatus();
  setInterval(pollStatus, 1000);
</script>
</body></html>)rawhtml";
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "text/html", html);
}

void handleRoot() {
  // Read volatile variables directly — atomic on 32-bit ESP32
  uint32_t vac_count = vac_pulse_counter;
  uint32_t saved_vac = saved_vac_count;
  uint32_t vac_target = calculated_vac_target;
  State current_state = state;
  bool sys_active = system_active;

  String html = INDEX_HTML;
  html.replace("%SERVOPPMM%", fmtFloat(SERVO_PULSES_PER_MM, 2));
  html.replace("%LABLEN%", String(SERVO_LABEL_LENGTH));
  html.replace("%LABLENMM%", fmtFloat((float)SERVO_LABEL_LENGTH / SERVO_PULSES_PER_MM, 1));
  html.replace("%SERVORUN%", fmtFloat(SERVO_RUN_SPEED_HZ, 0));
  html.replace("%SERVORUNMM%", fmtFloat(SERVO_RUN_SPEED_HZ / SERVO_PULSES_PER_MM, 1));
  html.replace("%HSACC%", fmtFloat(SERVO_HS_ACCEL, 0));
  html.replace("%HSDEC%", fmtFloat(SERVO_HS_DECEL, 0));
  html.replace("%LSACC%", fmtFloat(SERVO_LS_ACCEL, 0));
  html.replace("%LSDEC%", fmtFloat(SERVO_LS_DECEL, 0));
  html.replace("%SERVOBASE%", fmtFloat(SERVO_BASE_SPEED_HZ, 0));
  html.replace("%SERVOBASEMM%", fmtFloat(SERVO_BASE_SPEED_HZ / SERVO_PULSES_PER_MM, 1));
  html.replace("%SLOWPCT%", String((int)(SERVO_SLOWDOWN_PCT * 100)));
  html.replace("%SERVOMAXP%", String(SERVO_MAX_PULSES));
  html.replace("%MAXVACNOBARCODE%", String(MAX_VAC_PULSES_NO_BARCODE));

  html.replace("%CVACMMIN%", fmtFloat(convvac_target_freq * 50.7f / 23325.0f, 2));
  html.replace("%CVACFREQ%", fmtFloat(convvac_target_freq, 1));
  html.replace("%CVACACC%",  fmtFloat(CONVVAC_ACCEL, 1));
  html.replace("%CVACDEC%",  fmtFloat(CONVVAC_DECEL, 1));

  html.replace("%C2MMIN%",  fmtFloat(conv2_target_freq * 25.04f / 1244.0f, 2));
  html.replace("%C2FREQ%", fmtFloat(conv2_target_freq, 1));
  html.replace("%C2ACC%",  fmtFloat(CONV2_ACCEL, 1));
  html.replace("%C2DEC%",  fmtFloat(CONV2_DECEL, 1));

  html.replace("%SCROLLFREQ%", fmtFloat(scroll_target_freq, 1));
  html.replace("%SCROLLACC%",  fmtFloat(SCROLL_ACCEL, 1));
  html.replace("%SCROLLDEC%",  fmtFloat(SCROLL_DECEL, 1));

  html.replace("%CONVMAINMMIN%", fmtFloat(convmain_target_freq * 31.6f / 5946.0f, 2));
  html.replace("%CONVMAINFREQ%", fmtFloat(convmain_target_freq, 1));
  html.replace("%CONVMAINACC%",  fmtFloat(CONVMAIN_ACCEL, 1));
  html.replace("%CONVMAINDEC%",  fmtFloat(CONVMAIN_DECEL, 1));

  // RED/BARCODE params and live values
  html.replace("%GUARDMS%", String(barcode_sensor_guard_ms));
  html.replace("%MINVACBC%", String(min_vac_for_barcode));
  html.replace("%LABELOFF%", fmtFloat(label_offset_mm, 2));
  html.replace("%VACCOUNT%", String(vac_count));
  html.replace("%SAVEDVAC%", String(saved_vac));
  html.replace("%VACTARGET%", String(vac_target));

  // System status
  html.replace("%SYSTEMACTIVE%", sys_active ? "ON" : "OFF");

  // State label
  const char* stateLabelsArr[] = {"IDLE", "ACCEL", "RUNNING", "CREEP", "HOMING", "AUTO-DETECT", "CALIBRATING"};
  html.replace("%STATELABEL%", stateLabelsArr[current_state]);

  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "text/html", html);
}

void handleSet() {
  // Servo labeller parameters with bounds checking
  if (server.hasArg("servoppmm")) SERVO_PULSES_PER_MM = constrain(server.arg("servoppmm").toFloat(), 1.0f, 10000.0f);
  if (server.hasArg("lablen")) { SERVO_LABEL_LENGTH = constrain(server.arg("lablen").toInt(), 100, 200000); slowdown_pulses = (uint32_t)(SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT); SERVO_MAX_PULSES = (uint32_t)(SERVO_LABEL_LENGTH * 1.3f); }
  if (server.hasArg("servorun")) SERVO_RUN_SPEED_HZ = constrain(server.arg("servorun").toFloat(), 100.0f, 200000.0f);
  if (server.hasArg("hsacc")) SERVO_HS_ACCEL = constrain(server.arg("hsacc").toFloat(), 1000.0f, 5000000.0f);
  if (server.hasArg("hsdec")) SERVO_HS_DECEL = constrain(server.arg("hsdec").toFloat(), 1000.0f, 5000000.0f);
  if (server.hasArg("lsacc")) SERVO_LS_ACCEL = constrain(server.arg("lsacc").toFloat(), 1000.0f, 5000000.0f);
  if (server.hasArg("lsdec")) SERVO_LS_DECEL = constrain(server.arg("lsdec").toFloat(), 1000.0f, 5000000.0f);
  if (server.hasArg("servobase")) SERVO_BASE_SPEED_HZ = constrain(server.arg("servobase").toFloat(), 10.0f, 50000.0f);
  if (server.hasArg("slowpct")) { SERVO_SLOWDOWN_PCT = constrain(server.arg("slowpct").toFloat(), 80.0f, 99.0f) / 100.0f; slowdown_pulses = (uint32_t)(SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT); }
  if (server.hasArg("servomaxp")) SERVO_MAX_PULSES = constrain(server.arg("servomaxp").toInt(), 1000, 500000);
  if (server.hasArg("home")) home_requested = true;
  if (server.hasArg("autodetect")) auto_detect_requested = true;
  if (server.hasArg("calibrate")) { calibrate_distance_mm = constrain(server.arg("calibrate").toFloat(), 1.0f, 500.0f); calibrate_requested = true; }
  if (server.hasArg("maxvacnobarcode")) MAX_VAC_PULSES_NO_BARCODE = constrain(server.arg("maxvacnobarcode").toInt(), 100, 100000);

  // Conveyor VAC with bounds
  if (server.hasArg("cvacfreq"))    convvac_target_freq   = constrain(server.arg("cvacfreq").toFloat(), 0.0f, 100000.0f);
  if (server.hasArg("cvacacc"))     CONVVAC_ACCEL         = constrain(server.arg("cvacacc").toFloat(), 1000.0f, 200000.0f);
  if (server.hasArg("cvacdec"))     CONVVAC_DECEL         = constrain(server.arg("cvacdec").toFloat(), 1000.0f, 200000.0f);

  // Conveyor 2 with bounds
  if (server.hasArg("c2freq"))    conv2_target_freq   = constrain(server.arg("c2freq").toFloat(), 0.0f, 100000.0f);
  if (server.hasArg("c2acc"))     CONV2_ACCEL         = constrain(server.arg("c2acc").toFloat(), 1000.0f, 200000.0f);
  if (server.hasArg("c2dec"))     CONV2_DECEL         = constrain(server.arg("c2dec").toFloat(), 1000.0f, 200000.0f);

  // Auto-sync main conveyor to bottle forward speed when VAC, Belt2, or trim changes
  // Belt2 opposes VAC: bottle advance = (V_vac - V_belt2) / 2
  if (server.hasArg("cvacfreq") || server.hasArg("c2freq") || server.hasArg("mstrim")){
    float V_vac_mmin   = convvac_target_freq * (50.7f / 23325.0f);
    float V_belt2_mmin = conv2_target_freq   * (25.04f / 1244.0f);
    float bottle_mmin  = (V_vac_mmin - V_belt2_mmin) / 2.0f;
    if(bottle_mmin < 0.0f) bottle_mmin = 0.0f;
    bottle_mmin *= (1.0f + main_speed_trim_pct / 100.0f);
    convmain_target_freq = bottle_mmin * (5946.0f / 31.6f);
  }

  // Scroll with bounds
  if (server.hasArg("scrollfreq"))    scroll_target_freq   = constrain(server.arg("scrollfreq").toFloat(), 0.0f, 100000.0f);
  if (server.hasArg("scrollacc"))     SCROLL_ACCEL         = constrain(server.arg("scrollacc").toFloat(), 1000.0f, 200000.0f);
  if (server.hasArg("scrolldec"))     SCROLL_DECEL         = constrain(server.arg("scrolldec").toFloat(), 1000.0f, 200000.0f);

  // Label speed auto-computed from VAC speed at each cycle start

  // Conveyor Main with bounds
  if (server.hasArg("convmainfreq"))    convmain_target_freq   = constrain(server.arg("convmainfreq").toFloat(), 0.0f, 100000.0f);
  if (server.hasArg("convmainacc"))     CONVMAIN_ACCEL         = constrain(server.arg("convmainacc").toFloat(), 1000.0f, 200000.0f);
  if (server.hasArg("convmaindec"))     CONVMAIN_DECEL         = constrain(server.arg("convmaindec").toFloat(), 1000.0f, 200000.0f);

  // RED / BARCODE params
  if (server.hasArg("guardms")) barcode_sensor_guard_ms = constrain(server.arg("guardms").toInt(), 0, 5000);
  if (server.hasArg("minvacbc")) min_vac_for_barcode = constrain(server.arg("minvacbc").toInt(), 0, 50000);
  if (server.hasArg("labeloff")) label_offset_mm = constrain(server.arg("labeloff").toFloat(), 0.0f, 300.0f);
  if (server.hasArg("rtrim")) ratio_adjust_pct = constrain(server.arg("rtrim").toFloat(), -30.0f, 30.0f);
  if (server.hasArg("mstrim")) main_speed_trim_pct = constrain(server.arg("mstrim").toFloat(), -50.0f, 50.0f);

  // Enable/disable toggles from web UI
  if (server.hasArg("lab_en"))     labeller_enable = server.arg("lab_en").toInt() != 0;
  if (server.hasArg("cvac_en"))    convvac_enable  = server.arg("cvac_en").toInt() != 0;
  if (server.hasArg("c2_en"))      conv2_enable    = server.arg("c2_en").toInt() != 0;
  if (server.hasArg("scroll_en"))  scroll_enable   = server.arg("scroll_en").toInt() != 0;
  if (server.hasArg("cmain_en"))   convmain_enable = server.arg("cmain_en").toInt() != 0;
  if (server.hasArg("barcode_edge")){
    barcode_rising = server.arg("barcode_edge").toInt() != 0;
    detachInterrupt(digitalPinToInterrupt(PIN_BARCODE));
    attachInterrupt(digitalPinToInterrupt(PIN_BARCODE), barcodeISR, barcode_rising ? RISING : FALLING);
    saveParams();
  }
  if (server.hasArg("red_edge")){
    red_rising = server.arg("red_edge").toInt() != 0;
    detachInterrupt(digitalPinToInterrupt(PIN_REDSENSOR));
    attachInterrupt(digitalPinToInterrupt(PIN_REDSENSOR), redSensorISR, red_rising ? RISING : FALLING);
    saveParams();
  }
  if (server.hasArg("gap_edge")){
    gap_rising = server.arg("gap_edge").toInt() != 0;
    detachInterrupt(digitalPinToInterrupt(PIN_SENSOR));
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), sensorISR, FALLING);
    saveParams();
  }
  if (server.hasArg("activate"))   activate_flag   = true;
  if (server.hasArg("acq_mode"))   acquisition_mode = server.arg("acq_mode").toInt() != 0;
  if (server.hasArg("acq_reset")){ acq_vac_at_barcode=0; acq_vac_at_label_start=0; acq_vac_at_label_contact=0; acq_cycle_count=0; }
  if (server.hasArg("deactivate")) deactivate_flag = true;
  if (server.hasArg("manual_label")){
    if(system_active && labeller_enable && state == IDLE)
      start_flag = true;
  }

  // Save to NVS (skip for activate/deactivate, acquisition, and manual label — those handle saving themselves)
  if (!server.hasArg("activate") && !server.hasArg("deactivate")
      && !server.hasArg("acq_mode") && !server.hasArg("acq_reset")
      && !server.hasArg("manual_label")) {
    saveParams();
  }

  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ============ SETUP ============
void setup(){
  btStop();
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  loadParams();

  // I2C bus for PCF8574 expanders (must init before any I2C calls)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  pcf_write_outputs(0xFF);  // all MOSFET outputs OFF

  pinMode(PIN_STEP, OUTPUT);

  // Sensor inputs — GPIO34,35,39 are input-only, no internal pullup
  // (need external 10K pullup resistors to 3.3V)
  pinMode(PIN_SENSOR,    INPUT);
  pinMode(PIN_REDSENSOR, INPUT);
  pinMode(PIN_BARCODE,   INPUT);

  // ACTIVATE, DEACTIVATE, START, SW_LABELLER, SW_SCROLL are all I2C-polled via PCF8574
  // LED_ACTIVE is I2C output via PCF8574 (pcf_set_output)
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR),     sensorISR,     gap_rising ? RISING : FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_REDSENSOR),  redSensorISR,  red_rising ? RISING : FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BARCODE),    barcodeISR,    barcode_rising ? RISING : FALLING);

  // VAC pulse counting is done in SOFTWARE by integrating output frequency over time
  // in the task_conveyor_control() function. No hardware interrupt needed.
  Serial.println("VAC pulse counting: Software mode (counting output pulses)");

  // LEDC for 4 conveyors — use channels 0,2,4,6 so each gets its own timer
  // (channels sharing a timer pair cannot have independent frequencies)
  ledcSetup(0, 1000, 8);   // Timer 0
  ledcSetup(2, 1000, 8);   // Timer 1
  ledcSetup(4, 1000, 8);   // Timer 2
  ledcSetup(6, 1000, 8);   // Timer 3
  ledcAttachPin(PIN_CONVVAC, 0);   // ch 0 = Timer 0
  ledcAttachPin(PIN_CONV2, 2);     // ch 2 = Timer 1
  ledcAttachPin(PIN_SCROLL, 4);    // ch 4 = Timer 2
  ledcAttachPin(PIN_CONVMAIN, 6);  // ch 6 = Timer 3


  // ============ MCPWM SETUP (servo labeller) ============
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_STEP);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;
  pwm_config.cmpr_a = 50.0f;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);  // Start stopped

  // Direction pin (fixed direction)
  pinMode(PIN_SERVO_DIR, OUTPUT);
  digitalWrite(PIN_SERVO_DIR, LOW);  // Forward direction

  // Servo enable
  pinMode(PIN_SERVO_ENA, OUTPUT);
  digitalWrite(PIN_SERVO_ENA, HIGH);  // Enable servo

  // ============ PCNT SETUP (pulse counter on same pin) ============
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = PIN_STEP;
  pcnt_config.ctrl_gpio_num = -1;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DIS;
  pcnt_config.counter_h_lim = 30000;
  pcnt_config.counter_l_lim = 0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_unit_config(&pcnt_config);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);

  // PCNT overflow interrupt
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_overflow_isr, NULL);
  pcnt_counter_resume(PCNT_UNIT_0);

  // Compute slowdown threshold
  slowdown_pulses = (uint32_t)(SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT);
  Serial.println("Labeller: MCPWM + PCNT servo mode");

  // Timer 1: Target checker (1 MHz tick, fires every 50µs)
  precision_timer = timerBegin(1, 80, true);  // Timer 1, 80 prescaler = 1 MHz
  if(precision_timer == nullptr){
    Serial.println("ERROR: Failed to initialize precision timer!");
    while(1); // halt
  }
  timerAttachInterrupt(precision_timer, &precision_check_isr, true);
  timerAlarmWrite(precision_timer, 50, true);  // 50 µs period (20 kHz check rate)
  timerAlarmEnable(precision_timer);
  Serial.println("Target check ISR: 50µs interval (accuracy depends on VAC freq)");

  // Start conveyor control task with high priority for timing precision
  BaseType_t task_created = xTaskCreatePinnedToCore(task_conveyor_control,"ConveyorControl",4096,nullptr,3,nullptr,1);
  if(task_created != pdPASS){
    Serial.println("ERROR: Failed to create conveyor control task!");
    while(1); // halt
  }

  pcf_set_output(BIT_LED_ACTIVE, false);
  Serial.println("System ready – press ACTIVATE (X01) or labeller switch to start.");

  // WiFi AP mode
  WiFi.mode(WIFI_AP);
  bool ap_started = WiFi.softAP(ap_ssid, ap_password);
  if(!ap_started){
    Serial.println("ERROR: Failed to start WiFi AP!");
  }
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);  // usually 192.168.4.1

  if (MDNS.begin("labeller")) {
    Serial.println("mDNS responder started");
  }

  server.on("/", HTTP_GET, handleHome);
  server.on("/settings", HTTP_GET, handleRoot);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/status", HTTP_GET, [](){
    // Read volatile variables directly — each 32-bit read is atomic on ESP32.
    // No noInterrupts() needed: display-only values don't require perfect consistency,
    // and disabling interrupts on Core 0 disrupts WiFi/TCP causing machine stops.
    uint32_t vac_count = vac_pulse_counter;
    uint32_t vac_target = calculated_vac_target;
    uint32_t saved_vac = saved_vac_count;
    State current_state = state;
    bool sys_active = system_active;
    bool b_lab = labeller_enable;
    bool b_cvac = convvac_enable;
    bool b_c2 = conv2_enable;
    bool b_scroll = scroll_enable;
    bool b_cmain = convmain_enable;
    uint32_t a_barcode = acq_vac_at_barcode;
    uint32_t a_start = acq_vac_at_label_start;
    uint32_t a_contact = acq_vac_at_label_contact;
    uint32_t a_cycle = acq_cycle_count;
    bool a_mode = acquisition_mode;

    String json = "{\"vac_count\":";
    json += String(vac_count);
    json += ",\"vac_target\":";
    json += String(vac_target);
    json += ",\"saved_vac\":";
    json += String(saved_vac);
    json += ",\"state\":";
    json += String((int)current_state);
    json += ",\"system_active\":";
    json += sys_active ? "true" : "false";
    json += ",\"lab_en\":";
    json += b_lab ? "true" : "false";
    json += ",\"cvac_en\":";
    json += b_cvac ? "true" : "false";
    json += ",\"c2_en\":";
    json += b_c2 ? "true" : "false";
    json += ",\"scroll_en\":";
    json += b_scroll ? "true" : "false";
    json += ",\"cmain_en\":";
    json += b_cmain ? "true" : "false";
    json += ",\"acq_mode\":";
    json += a_mode ? "true" : "false";
    json += ",\"acq_cycle\":";
    json += String(a_cycle);
    json += ",\"acq_barcode_mm\":";
    json += String(a_barcode * VAC_MM_PER_PULSE, 1);
    json += ",\"acq_start_mm\":";
    json += String(a_start * VAC_MM_PER_PULSE, 1);
    json += ",\"acq_contact_mm\":";
    json += String(a_contact * VAC_MM_PER_PULSE, 1);

    // Input indicators (no noInterrupts needed — atomic reads)
    uint32_t now_ms = millis();
    json += ",\"red_sens\":";
    json += (now_ms - red_sensor_last_ms < 2000) ? "1" : "0";
    json += ",\"barcode\":";
    json += (now_ms - barcode_last_ms < 2000) ? "1" : "0";
    json += ",\"label_sens\":";
    json += (now_ms - label_sensor_last_ms < 2000) ? "1" : "0";
    uint8_t i2c_in = pcf_read_inputs();
    json += ",\"sw_scroll\":";
    json += (!(i2c_in & (1 << BIT_SW_SCROLL))) ? "1" : "0";
    json += ",\"i2c_in\":";
    json += String(i2c_in);
    json += ",\"act_btn\":";
    json += (now_ms - activate_last_ms < 500) ? "1" : "0";
    json += ",\"deact_btn\":";
    json += (now_ms - deactivate_last_ms < 500) ? "1" : "0";
    json += ",\"start_btn\":";
    json += (now_ms - start_last_ms < 500) ? "1" : "0";
    json += ",\"bc_rising\":";
    json += barcode_rising ? "1" : "0";
    json += ",\"red_rising\":";
    json += red_rising ? "1" : "0";
    json += ",\"gap_rising\":";
    json += gap_rising ? "1" : "0";

    // Compute ratio and Z_pulses for display (Belt2 opposes VAC)
    {
      float Vv = convvac_target_freq * VAC_MM_PER_PULSE;
      float Vb = conv2_target_freq * CONV2_MM_PER_PULSE;
      float ratio = (Vv + Vb) / (2.0f * Vv) * (1.0f + ratio_adjust_pct / 100.0f);
      float Z_pulses = label_offset_mm / VAC_MM_PER_PULSE;
      json += ",\"ratio\":";
      json += String(ratio, 4);
      json += ",\"rtrim\":";
      json += String(ratio_adjust_pct, 1);
      json += ",\"z_pulses\":";
      json += String((uint32_t)(Z_pulses + 0.5f));
      json += ",\"label_off\":";
      json += String(label_offset_mm, 2);
      json += ",\"mstrim\":";
      json += String(main_speed_trim_pct, 1);
      json += ",\"servo_pulses\":";
      json += String(get_servo_pulses());
      json += ",\"servo_freq\":";
      json += String((uint32_t)servo_current_freq);
      json += ",\"lab_len\":";
      json += String(SERVO_LABEL_LENGTH);
      json += ",\"lab_len_mm\":";
      json += String((float)SERVO_LABEL_LENGTH / SERVO_PULSES_PER_MM, 1);
      json += ",\"gap_pin\":";
      json += digitalRead(PIN_SENSOR) ? "1" : "0";
      float bot_mmin = (Vv - Vb) / 2.0f * 60.0f / 1000.0f;  // mm/s to m/min
      if(bot_mmin < 0.0f) bot_mmin = 0.0f;
      bot_mmin *= (1.0f + main_speed_trim_pct / 100.0f);
      json += ",\"bot_mmin\":";
      json += String(bot_mmin, 2);
    }

    // BPM calculation (runs on Core 0, no impact on labeller)
    {
      static uint32_t bpm_prev_count = 0;
      static uint32_t bpm_prev_ms = 0;
      static float bpm_value = 0.0f;
      uint32_t cur_count = red_trigger_count;
      uint32_t cur_ms = millis();
      uint32_t dt = cur_ms - bpm_prev_ms;
      uint32_t dn = cur_count - bpm_prev_count;
      if(dt >= 5000){  // update every 5 seconds
        if(dn > 0 && dt > 0){
          bpm_value = dn * 60000.0f / dt;
        } else {
          bpm_value = 0.0f;
        }
        bpm_prev_count = cur_count;
        bpm_prev_ms = cur_ms;
      } else if(dt > 0 && bpm_value > 0.0f && cur_ms - red_sensor_last_ms > 10000){
        // No bottle for 10s, reset to 0
        bpm_value = 0.0f;
      }
      json += ",\"bpm\":";
      json += String(bpm_value, 1);
    }

    json += "}";
    server.send(200, "application/json", json);
  });
  server.on("/saveslot", HTTP_GET, [](){
    int slot = server.hasArg("slot") ? server.arg("slot").toInt() : 0;
    if(slot < 1 || slot > 10){ server.send(400, "text/plain", "bad slot"); return; }
    String ns = "slot" + String(slot);
    Preferences sp;
    if(!sp.begin(ns.c_str(), false)){ server.send(500, "text/plain", "nvs error"); return; }
    sp.putFloat("servoppmm", SERVO_PULSES_PER_MM);
    sp.putFloat("servorun", SERVO_RUN_SPEED_HZ);
    sp.putFloat("hsacc", SERVO_HS_ACCEL);
    sp.putFloat("hsdec", SERVO_HS_DECEL);
    sp.putFloat("lsacc", SERVO_LS_ACCEL);
    sp.putFloat("lsdec", SERVO_LS_DECEL);
    sp.putFloat("servobase", SERVO_BASE_SPEED_HZ);
    sp.putUInt("lablen", SERVO_LABEL_LENGTH);
    sp.putFloat("slowpct", SERVO_SLOWDOWN_PCT);
    sp.putUInt("servomaxp", SERVO_MAX_PULSES);
    sp.putUInt("maxvacnobc", MAX_VAC_PULSES_NO_BARCODE);
    sp.putFloat("cvacfreq", convvac_target_freq);
    sp.putFloat("cvacacc", CONVVAC_ACCEL);
    sp.putFloat("cvacdec", CONVVAC_DECEL);
    sp.putFloat("c2freq", conv2_target_freq);
    sp.putFloat("c2acc", CONV2_ACCEL);
    sp.putFloat("c2dec", CONV2_DECEL);
    sp.putFloat("scrollfreq", scroll_target_freq);
    sp.putFloat("scrollacc", SCROLL_ACCEL);
    sp.putFloat("scrolldec", SCROLL_DECEL);
    sp.putFloat("cmainfreq", convmain_target_freq);
    sp.putFloat("cmainacc", CONVMAIN_ACCEL);
    sp.putFloat("cmaindec", CONVMAIN_DECEL);
    sp.putFloat("labeloff", label_offset_mm);
    sp.putFloat("rtrim", ratio_adjust_pct);
    sp.putFloat("mstrim", main_speed_trim_pct);
    sp.putUInt("guardms", barcode_sensor_guard_ms);
    sp.putUInt("minvacbc", min_vac_for_barcode);
    sp.putBool("sysact", system_active);
    sp.putBool("laben", labeller_enable);
    sp.putBool("scrollen", scroll_enable);
    sp.putBool("cvacen", convvac_enable);
    sp.putBool("c2en", conv2_enable);
    sp.putBool("cmainen", convmain_enable);
    if(server.hasArg("name")){
      sp.putString("name", server.arg("name"));
    }
    sp.putBool("used", true);
    sp.end();
    Serial.printf("Saved settings to Country %d\n", slot);
    server.send(200, "text/plain", "ok");
  });

  server.on("/loadslot", HTTP_GET, [](){
    int slot = server.hasArg("slot") ? server.arg("slot").toInt() : 0;
    if(slot < 1 || slot > 10){ server.send(400, "text/plain", "bad slot"); return; }
    String ns = "slot" + String(slot);
    Preferences sp;
    if(!sp.begin(ns.c_str(), true)){ server.send(200, "text/plain", "empty"); return; }
    if(!sp.getBool("used", false)){ sp.end(); server.send(200, "text/plain", "empty"); return; }
    SERVO_PULSES_PER_MM = sp.getFloat("servoppmm", SERVO_PULSES_PER_MM);
    SERVO_RUN_SPEED_HZ  = sp.getFloat("servorun", SERVO_RUN_SPEED_HZ);
    SERVO_HS_ACCEL      = sp.getFloat("hsacc", SERVO_HS_ACCEL);
    SERVO_HS_DECEL      = sp.getFloat("hsdec", SERVO_HS_DECEL);
    SERVO_LS_ACCEL      = sp.getFloat("lsacc", SERVO_LS_ACCEL);
    SERVO_LS_DECEL      = sp.getFloat("lsdec", SERVO_LS_DECEL);
    SERVO_BASE_SPEED_HZ = sp.getFloat("servobase", SERVO_BASE_SPEED_HZ);
    SERVO_LABEL_LENGTH  = sp.getUInt("lablen", SERVO_LABEL_LENGTH);
    SERVO_SLOWDOWN_PCT  = sp.getFloat("slowpct", SERVO_SLOWDOWN_PCT);
    SERVO_MAX_PULSES    = sp.getUInt("servomaxp", SERVO_MAX_PULSES);
    slowdown_pulses = (uint32_t)(SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT);
    MAX_VAC_PULSES_NO_BARCODE = sp.getUInt("maxvacnobc", MAX_VAC_PULSES_NO_BARCODE);
    convvac_target_freq = sp.getFloat("cvacfreq", convvac_target_freq);
    CONVVAC_ACCEL       = sp.getFloat("cvacacc", CONVVAC_ACCEL);
    CONVVAC_DECEL       = sp.getFloat("cvacdec", CONVVAC_DECEL);
    conv2_target_freq   = sp.getFloat("c2freq", conv2_target_freq);
    CONV2_ACCEL         = sp.getFloat("c2acc", CONV2_ACCEL);
    CONV2_DECEL         = sp.getFloat("c2dec", CONV2_DECEL);
    scroll_target_freq  = sp.getFloat("scrollfreq", scroll_target_freq);
    SCROLL_ACCEL        = sp.getFloat("scrollacc", SCROLL_ACCEL);
    SCROLL_DECEL        = sp.getFloat("scrolldec", SCROLL_DECEL);
    convmain_target_freq= sp.getFloat("cmainfreq", convmain_target_freq);
    CONVMAIN_ACCEL      = sp.getFloat("cmainacc", CONVMAIN_ACCEL);
    CONVMAIN_DECEL      = sp.getFloat("cmaindec", CONVMAIN_DECEL);
    label_offset_mm     = sp.getFloat("labeloff", label_offset_mm);
    ratio_adjust_pct    = sp.getFloat("rtrim", ratio_adjust_pct);
    main_speed_trim_pct = sp.getFloat("mstrim", main_speed_trim_pct);
    barcode_sensor_guard_ms = sp.getUInt("guardms", barcode_sensor_guard_ms);
    min_vac_for_barcode    = sp.getUInt("minvacbc", min_vac_for_barcode);
    system_active       = sp.getBool("sysact", system_active);
    labeller_enable     = sp.getBool("laben", labeller_enable);
    scroll_enable       = sp.getBool("scrollen", scroll_enable);
    convvac_enable      = sp.getBool("cvacen", convvac_enable);
    conv2_enable        = sp.getBool("c2en", conv2_enable);
    convmain_enable     = sp.getBool("cmainen", convmain_enable);
    sp.end();
    saveParams();
    Serial.printf("Loaded settings from Country %d\n", slot);
    server.send(200, "text/plain", "ok");
  });

  server.on("/slotnames", HTTP_GET, [](){
    String json = "[";
    for(int i = 1; i <= 10; i++){
      String ns = "slot" + String(i);
      Preferences sp;
      String name = String(i);
      if(sp.begin(ns.c_str(), true)){
        if(sp.getBool("used", false)){
          name = sp.getString("name", String(i));
        }
        sp.end();
      }
      if(i > 1) json += ",";
      json += "\"" + name + "\"";
    }
    json += "]";
    server.send(200, "application/json", json);
  });

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  // Web server on Core 0 — separate from labeller state machine on Core 1
  xTaskCreatePinnedToCore(task_web_server, "WebServer", 8192, nullptr, 1, nullptr, 0);
  Serial.println("Web server running on Core 0");
}

// ============ LOOP ============
void loop(){
  // Poll I2C buttons (ACTIVATE, DEACTIVATE, START via PCF8574)
  poll_i2c_buttons();

  // Capture time FIRST — before any web/IO — for accurate state machine timing
  static uint32_t t0_us, last_upd_us;
  uint32_t now_us = micros();

  // ============ ACTIVATE / DEACTIVATE BUTTONS ============
  if(activate_flag){
    activate_flag=false;
    system_active=true;
    loadParams();  // Restore saved enable states from NVS
    pcf_set_output(BIT_LED_ACTIVE, true);
    Serial.println("System ACTIVATED - enable states restored from NVS");
  }

  if(deactivate_flag){
    deactivate_flag=false;
    saveParams();  // Save current enable states before deactivating
    system_active=false;
    state = IDLE;
    servo_stop();
    gap_detected = false;
    pcf_set_output(BIT_LED_ACTIVE, false);

    vac_counting = false;
    vac_pulse_counter = 0;
    vac_pulse_accumulator = 0.0f;
    calculated_vac_target = 0;
    barcode_locked = false;
    barcode_calc_pending = false;

    Serial.println("System DEACTIVATING - enable states saved");
  }

  // ============ TOGGLE SWITCHES (only work when system is active) ============
  if (sw_labeller_flag) {
    sw_labeller_flag = false;
    if(system_active){
      labeller_enable = !labeller_enable;
      if(!labeller_enable){
        state = IDLE;
        servo_stop();
        gap_detected = false;
        start_flag = false;
        sensor_triggered = false;
        vac_counting = false;
        vac_pulse_counter = 0;
        vac_pulse_accumulator = 0.0f;
        calculated_vac_target = 0;
        barcode_calc_pending = false;
        barcode_locked = false;
      }
      Serial.print("Labeller toggle: ");
      Serial.println(labeller_enable ? "ON" : "OFF");
    } else {
      Serial.println("Labeller toggle ignored - system not activated");
    }
  }

  if (sw_scroll_flag) {
    sw_scroll_flag = false;
    if(system_active){
      scroll_enable = !scroll_enable;
      Serial.print("Scroll conveyor toggle: ");
      Serial.println(scroll_enable ? "ON" : "OFF");
    } else {
      Serial.println("Scroll toggle ignored - system not activated");
    }
  }

  // ============ SYSTEM ACTIVE PROCESSING ============
  if(!system_active){
    start_flag=false;
    sensor_triggered=false;
    pcf_set_output(BIT_LED_ACTIVE, false);
  } else {
    // Sync LED with system_active state
    if(pcf_output_state & (1 << BIT_LED_ACTIVE)){
      pcf_set_output(BIT_LED_ACTIVE, true);
    }

    // Handle barcode calculation (moved from ISR for performance)
    // Formula: target = saved_vac × ratio + Z_pulses
    //   ratio = (V_vac + V_belt2) / (2 × V_vac)  — Belt2 opposes VAC direction
    //   Z_pulses = label_offset_mm / VAC_MM_PER_PULSE — fixed offset for label placement tuning
    // Belt2 runs OPPOSITE to VAC belt, so bottle rotation = (V_vac + V_belt2)/2
    // and bottle advance = (V_vac - V_belt2)/2. The ratio maps VAC pulses to surface arc.
    if(barcode_calc_pending){
      portENTER_CRITICAL(&timerMux);
      uint32_t saved_vac = saved_vac_count;
      float current_freq = convvac_current_freq;
      barcode_calc_pending = false;
      portEXIT_CRITICAL(&timerMux);

      if(saved_vac > 0){
        // Compute belt speeds in mm/s
        float V_vac   = convvac_target_freq * VAC_MM_PER_PULSE;
        float V_belt2 = conv2_target_freq   * CONV2_MM_PER_PULSE;

        // Belt2 opposes VAC: rotation speed = (V_vac + V_belt2)/2
        // Advance speed = (V_vac - V_belt2)/2
        // Ratio = rotation_speed / V_vac = (V_vac + V_belt2) / (2 * V_vac)
        float speed_sum = V_vac + V_belt2;
        float ratio = speed_sum / (2.0f * V_vac) * (1.0f + ratio_adjust_pct / 100.0f);

        // Fixed offset: label_offset_mm converted to VAC pulses
        float Z_pulses = label_offset_mm / VAC_MM_PER_PULSE;

        uint32_t target = (uint32_t)(saved_vac * ratio + Z_pulses + 0.5f);

        // Delay compensation for processing latency
        uint32_t delay_compensation = (uint32_t)(current_freq * 0.0007f);
        if(delay_compensation < 2) delay_compensation = 2;
        if(delay_compensation > 100) delay_compensation = 100;

        if(target > delay_compensation + 10){
          target -= delay_compensation;
        } else if(target > 10){
          target = 10;
        } else {
          target = 10;
        }

        bool target_set = false;
        portENTER_CRITICAL(&timerMux);
        if(saved_vac_count == saved_vac && vac_counting){
          calculated_vac_target = target;
          barcode_locked = false;
          target_set = true;
        } else {
          barcode_locked = false;
          target_set = false;
        }
        portEXIT_CRITICAL(&timerMux);

        if(target_set){
          Serial.print("Barcode! VAC:");
          Serial.print(saved_vac);
          Serial.print(" Ratio:");
          Serial.print(ratio, 4);
          Serial.print(" Z:");
          Serial.print((uint32_t)(Z_pulses + 0.5f));
          Serial.print("p (");
          Serial.print(label_offset_mm, 1);
          Serial.print("mm) Target:");
          Serial.println(target);
        } else {
          Serial.println("Barcode calc discarded - new RED cycle started");
        }
      } else {
        barcode_locked = false;
      }
    }

    // ============ START LABELLER ============
    if(start_flag && state == IDLE){
      start_flag = false;
      gap_detected = false;
      sensor_triggered = false;
      slowdown_pulses = (uint32_t)(SERVO_LABEL_LENGTH * SERVO_SLOWDOWN_PCT);
      servo_start(SERVO_BASE_SPEED_HZ);  // Start at base speed, ramp up
      state = ACCEL;
      Serial.println("Servo: ACCEL");
    }

  } // end system_active

  // ============ HOME SEQUENCE (works without system_active) ============
  if(home_requested && state == IDLE){
    home_requested = false;
    servo_start(SERVO_BASE_SPEED_HZ);
    state = HOMING;
    Serial.println("HOME: Searching for gap...");
  }

  // ============ AUTO-DETECT LABEL LENGTH (works without system_active) ============
  if(auto_detect_requested && state == IDLE){
    auto_detect_requested = false;
    auto_detect_pulse_start = 0;
    servo_start(SERVO_BASE_SPEED_HZ);
    state = AUTO_DETECT;
    Serial.println("AUTO-DETECT: Starting...");
  }

  // ============ CALIBRATION INDEX (works without system_active) ============
  if(calibrate_requested && state == IDLE){
    calibrate_requested = false;
    calibrate_pulses = (uint32_t)(calibrate_distance_mm * SERVO_PULSES_PER_MM + 0.5f);
    servo_start(SERVO_CALIB_SPEED_HZ);
    state = CALIBRATING;
    Serial.print("CALIBRATE: Indexing ");
    Serial.print(calibrate_distance_mm);
    Serial.print("mm = ");
    Serial.print(calibrate_pulses);
    Serial.println(" pulses at slow speed");
  }
}
