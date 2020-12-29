/*
 * ----------------------------------------------------------------------------
 * "THE PROP-WARE LICENSE" (Revision 42):
 * <https://github.com/JyeSmith> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me some props in return.   Jye Smith
 * ----------------------------------------------------------------------------
 */

/* Some of the below code is taken from examples provided by Felix on RCGroups.com
 *
 * KISS ESC 24A Serial Example Code for Arduino.
 * https://www.rcgroups.com/forums/showthread.php?2555162-KISS-ESC-24A-Race-Edition-Flyduino-32bit-ESC
 * https://www.rcgroups.com/forums/showatt.php?attachmentid=8521072&d=1450345654 *
 */

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// www.miniquadtestbench.com
// Uncommenting the below define will start the test sequence defined on MQTB
// WARNING - THE MOTOR WILL START TO SPIN AUTOMATICALLY!!!
//#define MINIQUADTESTBENCH
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "WebUpdater.h"

#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <Arduino.h>
#include <esp32-hal.h>

#define MOTOR_POLES 14

// HX711
#if LOADCELL_ENABLED
#include <HX711.h>

#define LOADCELL_DOUT_PIN     25
#define LOADCELL_SCK_PIN      26
#define LOADCELL_CALIBRATION  345.0
HX711 loadcell;
#endif // LOADCELL_ENABLED

long thrust = 0;

#ifdef MINIQUADTESTBENCH
static uint32_t runMQTBSequence_last;
#endif

rmt_obj_t* rmt_send = NULL;

HardwareSerial tlm_serial(1);

#define LCD_I2C_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

volatile bool requestTelemetry = false;
bool printTelemetry = true;
volatile uint16_t dshotUserInputValue = 0;
constexpr uint32_t dshotmin = 48;
constexpr uint32_t dshotmax = 2047;
const uint32_t dshotidle = dshotmin + round(3.5*(dshotmax-dshotmin)/100); // 3.5%
const uint32_t dshot50 =   dshotmin + round(50*(dshotmax-dshotmin)/100); // 50%
const uint32_t dshot75 =   dshotmin + round(75*(dshotmax-dshotmin)/100); // 75%
bool runMQTBSequence = false;

uint32_t currentTime;
uint32_t temperature = 0;
uint32_t temperatureMax = 0;
float voltage = 0;
float voltageMin = 99;
uint32_t current = 0;
uint32_t currentMax = 0;
uint32_t erpm = 0;
uint32_t erpmMax = 0;
uint32_t rpm = 0;
uint32_t rpmMAX = 0;
int32_t kv = 0;
int32_t kvMax = 0;


#define LCD_write_value(X, Y, VAL) \
    do { \
        display.setCursor(X, Y); display.println(VAL); \
    } while(0);
#define LCD_clear()     display.clearDisplay();
#define LCD_display()   display.display();

#define ERPM_TO_RPM(erpm) ((erpm) / (MOTOR_POLES / 2))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

void dshotOutput(uint16_t value, bool telemetry);
void receiveTelemtrie(void);
void updateDisplay(void);

void error_driver(const char * str)
{
    Serial.printf("ERR: %s\n", str);
    LCD_clear();
    LCD_write_value(10, 0, "!! ERROR !!");
    LCD_write_value(0, 10, str);
    LCD_display();
    while(1)
        ;
}

void gotTouch_stop(void)
{
    dshotUserInputValue = 0;
    runMQTBSequence = false;
    printTelemetry = true;
}

void resetMaxMinValues(void)
{
    gotTouch_stop();
    temperatureMax = 0;
    voltageMin = 99;
    currentMax = 0;
    erpmMax = 0;
    rpmMAX = 0;
    kv = kvMax = 0;
}

void gotTouch_10(void) // 10%
{
    resetMaxMinValues();
    dshotUserInputValue = 247;
}

void gotTouch_20(void) // 20%
{
    resetMaxMinValues();
    dshotUserInputValue = 447;
}

void gotTouch_50(void) // 50%
{
    resetMaxMinValues();
    dshotUserInputValue = 1047;
}

void gotTouch_100(void) // 100%
{
    resetMaxMinValues();
    dshotUserInputValue = 2047;
}

void gotTouch_rst(void)
{
    resetMaxMinValues();
}

void IRAM_ATTR getTelemetry(void)
{
    requestTelemetry = true;
}

void startTelemetryTimer(void)
{
    static hw_timer_t * timer;
    timer = timerBegin(0, 80, true); // timer_id = 0; divider=80; countUp = true;
    timerAttachInterrupt(timer, &getTelemetry, true); // edge = true
    timerAlarmWrite(timer, 20000, true);  //1000 = 1 ms
    timerAlarmEnable(timer);
}

// Second core used to handle dshot packets
void secondCoreTask(void * pvParameters)
{
    while (1) {
        dshotOutput(dshotUserInputValue, requestTelemetry);
        if (requestTelemetry) {
            requestTelemetry = false;
        }
        delay(1);
    }
}

void setup()
{
    char temp_str[16];

    Serial.begin(115200);
    tlm_serial.begin(115200, SERIAL_8N1, 16, 17);

    /* Initialize display */
    display.begin(SSD1306_SWITCHCAPVCC, LCD_I2C_ADDR);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

#if LOADCELL_ENABLED
    loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    loadcell.set_scale(LOADCELL_CALIBRATION);
    loadcell.tare();
#endif // LOADCELL_ENABLED

    rmt_send = rmtInit(5, true, RMT_MEM_64);
    if (rmt_send == NULL) {
        error_driver("init sender failed");
    }

    float realTick = rmtSetTick(rmt_send, 12.5); // 12.5ns sample rate
    Serial.printf("rmt_send tick set to: %fns\n", realTick);

    uint32_t start = millis(), now = start;
    while ((now - start) < 3500) {
        uint32_t distance = now - start;

        dshotOutput(0, false);
        delay(1);

        LCD_clear();
        display.setTextSize(1);
        LCD_write_value(0, 0, "Initialising ESC...");
        display.setTextSize(2);
        if (distance < 500) {
            LCD_write_value(30, 30, "4s");
        } else if (distance < 1500) {
            LCD_write_value(30, 30, "3s");
        } else if (distance < 2500) {
            LCD_write_value(30, 30, "2s");
        } else {
            LCD_write_value(30, 30, "1s");
        }
        LCD_display();

        now = millis();
    }
    display.setTextSize(1);

    touchAttachInterrupt(T4, gotTouch_rst, 40);
    touchAttachInterrupt(T5, gotTouch_100, 40);
    touchAttachInterrupt(T6, gotTouch_50, 40);
    touchAttachInterrupt(T7, gotTouch_20, 40);
    touchAttachInterrupt(T9, gotTouch_10, 40);
    touchAttachInterrupt(T8, gotTouch_stop, 40);

    // Empty Rx Serial of garbage telemtry
    while (0 <= tlm_serial.read());

    getTelemetry();

    BeginWebUpdate();

    // Timer used to request tlm continually in case ESC rcv bad packet
    startTelemetryTimer();

    TaskHandle_t Task1;
    xTaskCreatePinnedToCore(secondCoreTask, "Task1", 10000, NULL, 1, &Task1, 0);

    Serial.print("Time (ms)");
    Serial.print(",");
    Serial.print("dshot");
    Serial.print(",");
    Serial.print("Voltage (V)");
    Serial.print(",");
    Serial.print("Current (A)");
    Serial.print(",");
    Serial.print("RPM");
    Serial.print(",");
    Serial.println("Thrust (g)");

    resetMaxMinValues();

    LCD_clear();
#ifdef MINIQUADTESTBENCH
    dshotUserInputValue = dshotidle;
    runMQTBSequence = true;
    LCD_write_value(20, 10, "Running");
    LCD_write_value(20, 20, " MQTB");
    LCD_write_value(20, 30, "Sequence");
    runMQTBSequence_last = millis();
#else // !MINIQUADTESTBENCH
    runMQTBSequence = false;
    LCD_write_value(20, 10, "READY");
#endif // MINIQUADTESTBENCH
    LCD_display();
}

void loop()
{
    HandleWebUpdate();

#if LOADCELL_ENABLED
    if (loadcell.is_ready()) {
        thrust = loadcell.get_units(1);
    }
#endif // LOADCELL_ENABLED

    if (!requestTelemetry) {
        receiveTelemtrie();
    }

#ifdef MINIQUADTESTBENCH
    if (runMQTBSequence) {
        currentTime = millis();
        uint32_t const distance = (currentTime - runMQTBSequence_last);
        if (distance >= 4000 && distance < 6000) {
            dshotUserInputValue = dshot50;
        } else if (distance >= 6000 && distance < 8000) {
            dshotUserInputValue = dshotidle;
        } else if (distance >= 8000 && distance < 10000) {
            dshotUserInputValue = dshot75;
        } else if (distance >= 10000 && distance < 12000) {
            dshotUserInputValue = dshotidle;
        } else if (distance >= 12000 && distance < 14000) {
            dshotUserInputValue = dshotmax;
        } else if (distance >= 14000 && distance < 16000) {
            dshotUserInputValue = dshotmin;
        } else if (distance >= 16000 && distance < 22000) {
            dshotUserInputValue = dshotmin + (distance - 16000) * (dshotmax - dshotmin) / 6000.0;
        } else if (distance >= 24000 && distance < 26000) {
            dshotUserInputValue = dshotidle;
        } else if (distance >= 26000 && distance < 28000) {
            printTelemetry = false;
            dshotUserInputValue = 0;
        }
        runMQTBSequence_last = currentTime;
    }
#endif // MINIQUADTESTBENCH
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++)
    crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++)
    crc = update_crc8(Buf[i], crc);
  return (crc);
}

void receiveTelemtrie(void)
{
    uint8_t SerialBuf[10];
    if (tlm_serial.available() >= sizeof(SerialBuf)) { // transmission complete

        for (uint_fast8_t iter = 0; iter < sizeof(SerialBuf); iter++)
            SerialBuf[iter] = tlm_serial.read();

        // get the 8 bit CRC
        uint8_t crc8 = get_crc8(SerialBuf, (sizeof(SerialBuf) - 1));

        getTelemetry();

        if (crc8 != SerialBuf[(sizeof(SerialBuf) - 1)]) {
            // Serial.println("CRC transmission failure");

            // Empty Rx Serial of garbage telemtry
            while (0 <= tlm_serial.read());

            return; // transmission failure
        }

        // compute the received values
        //  [0]     Temperature
        //  [1,2]   Voltage
        //  [3,4]   Current
        //  [5,6]   used mAh
        //  [7,8]   eRpM
        uint8_t _temperature = SerialBuf[0];
        uint32_t _voltage = ((uint16_t)SerialBuf[1] << 8) + SerialBuf[2];
        _voltage /= 100;
        uint32_t _current = ((uint16_t)SerialBuf[3] << 8) + SerialBuf[4];
        //uint32_t _mah = ((uint16_t)SerialBuf[5] << 8) + SerialBuf[6];
        uint32_t _erpm = ((uint16_t)SerialBuf[7] << 8) + SerialBuf[8];
        _erpm *= 100;

        if (!runMQTBSequence) { // Do not update during MQTB sequence.  Slows serial output.
            updateDisplay();
        }

        if (printTelemetry) {
            Serial.print(millis());
            Serial.print(",");
            Serial.print(dshotUserInputValue);
            Serial.print(",");
            Serial.print(_voltage);
            Serial.print(",");
            Serial.print(_current / 10.0);
            Serial.print(",");
            Serial.print(ERPM_TO_RPM(_erpm));
            Serial.print(",");
            // Thrust
            Serial.println(thrust);
        }

        temperature = (9 * temperature) / 10 + (_temperature / 10);
        if (temperature > temperatureMax) {
            temperatureMax = temperature;
        }

        voltage = (9 * voltage) / 10 + (_voltage / 10);
        if (voltage < voltageMin) {
            voltageMin = voltage;
        }

        current = (9 * current) / 10 + (_current * 10);
        if (current > currentMax) {
            currentMax = current;
        }

        erpm = (9 * erpm) / 10 + (_erpm / 10);
        if (erpm > erpmMax) {
            erpmMax = erpm;
        }

        uint32_t _rpm = ERPM_TO_RPM(erpm);
        if (_rpm > rpmMAX) {
            rpmMAX = _rpm;
        }

        if (_rpm && voltage && dshotUserInputValue) { // Stops weird numbers :|
            kv = _rpm / voltage / ((float)(dshotUserInputValue - dshotmin) / (dshotmax - dshotmin));
            if (kv > kvMax) {
                kvMax = kv;
            }
        }

        rpm = _rpm;

#if 0
        Serial.print("TELE: ");
        Serial.print(_temperature);
        Serial.print("C, ");
        Serial.print(_voltage);
        Serial.print("V, ");
        Serial.print(_current * 100);
        Serial.print("mA, eRPM:");
        Serial.print(_erpm);
        Serial.print(", RPM:");
        Serial.print(ERPM_TO_RPM(_erpm));
        Serial.print(", KV:");
        Serial.println(ERPM_TO_RPM(_erpm) / _voltage);
#endif
    }
}

void dshotOutput(uint16_t value, bool telemetry)
{
    rmt_data_t dshotPacket[16];
    uint16_t packet;
    uint_fast8_t size = ARRAY_SIZE(dshotPacket);

    // telemetry bit
    if (telemetry) {
        packet = (value << 1) | 1;
    } else {
        packet = (value << 1) | 0;
    }

    // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
    int csum = 0;
    int csum_data = packet;
    for (uint_fast8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    packet = (packet << 4) | csum;

    // durations are for dshot600
    // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
    // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
    // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
    // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
    for (uint_fast8_t i = 0; i < size; i++) {
        dshotPacket[i].level0 = 1;
        dshotPacket[i].level1 = 0;
        if (packet & 0x8000) {
            dshotPacket[i].duration0 = 100;
            dshotPacket[i].duration1 = 34;
        } else {
            dshotPacket[i].duration0 = 50;
            dshotPacket[i].duration1 = 84;
        }
        packet <<= 1;
    }

    rmtWrite(rmt_send, dshotPacket, size);
}

void updateDisplay(void)
{
    LCD_clear();

#define OFFSET_1 45
#define OFFSET_2 90

    LCD_write_value(0,  0, "Dshot Packet");
    LCD_write_value(0, 10, "Temp C");
    LCD_write_value(0, 20, "Volt");
    LCD_write_value(0, 30, "mA");
    LCD_write_value(0, 40, "eRPM");
    LCD_write_value(0, 50, "KV");

    LCD_write_value(OFFSET_1, 10, temperature);
    LCD_write_value(OFFSET_1, 20, voltage);
    LCD_write_value(OFFSET_1, 30, current);
    LCD_write_value(OFFSET_1, 40, erpm);
    LCD_write_value(OFFSET_1, 50, kv);

    LCD_write_value(OFFSET_2,  0, dshotUserInputValue);
    LCD_write_value(OFFSET_2, 10, temperatureMax);
    LCD_write_value(OFFSET_2, 20, voltageMin);
    LCD_write_value(OFFSET_2, 30, currentMax);
    LCD_write_value(OFFSET_2, 40, erpmMax);
    LCD_write_value(OFFSET_2, 50, kvMax);

    LCD_display();
}
