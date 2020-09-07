#include <BH1750.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <stdio.h>
#include <Adafruit_ADS1015.h>

#define SERIAL_BAUD 9600              // Serial port baud rate
#define CSV_OUTPUT false                // Set to true for CSV
#define CSV_MV_ONLY false               // Set to true for mV only in CSV
#define IRRADIANCE_POLLING_LOOPS 10
#define TEMP_POLLING_LOOPS 5
#define REQUIRE_STABLE_TEMP true
#define MAX_STABLE_TEMP_ERR_PPM 5000    // 5000 = 0.5%
#define REQUIRE_STABLE_IRRAD true
#define MAX_STABLE_IRRAD_ERR_PPM 10000  // 10000 = 1%
#define MS_DELAY_BETWEEN_LOOPS 250
#define INVALID_TEMP -9999.0

// ADS1115 and TMP36 constants (do not modify)
#define ADS1115_UNITY_GAIN_MAX_MILLIVOLTS 4096
#define ADS1115_NON_SIGN_BITS 15
#define ADS1115_MIN_VALUE ((long)1 << ADS1115_NON_SIGN_BITS)
#define ADS1115_MAX_VALUE (((long)1 << ADS1115_NON_SIGN_BITS) - 1)
#define ADS1115_PGA_GAIN_TMP36 2
#define ADS1115_PGA_GAIN_PDB_C139 8
#define TMP36_OFFSET_MILLIVOLTS 500     // from datasheet
#define TMP36_MV_PER_DEG_C 10           // from datasheet

// Calibration constants (may be modified)
#define PYRANO_CAL 4.3                // X coefficient (slope if A=0): W/m^2/mV
#define PYRANO_CAL_A 0.0              // X^2 coefficient: W/m^2/mV^2
#define PHOTODIODE_NOMINAL_DEG_C 25.0
#define PHOTODIODE_PCT_PER_DEG_C 0.16 // determined empirically, YMMV

Adafruit_ADS1115 ads1115;

float photodiode_temp_scaling_factor = 1.0;

// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  10000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!

#define SYNC_INTERVAL 10000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);

}

void setup(void)
{
  Wire.begin();
  Serial.begin(SERIAL_BAUD);
  ads1115.begin();
  Serial.println();
  
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin,OUTPUT);
  
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) 
  {
    error("Card failed, or not present");
  }
  
  Serial.println("Card initialized!");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 1000; i++)
  {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) 
    {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) 
  {
    error("Couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) 
  {
    logfile.println("RTC failed");
  #if ECHO_TO_SERIAL
    Serial.println("RTC failed");
  #endif  //ECHO_TO_SERIAL
  }
  

  logfile.println("millis,stamp,datetime,Irradiance");    
  #if ECHO_TO_SERIAL
    Serial.println("millis,stamp,datetime,Irradiance");
  #endif //ECHO_TO_SERIAL
 
}

void loop(void)
{
  int16_t ads1115_val;
  long ads1115_val_sum, ads1115_val_avg, ppm_error_from_avg;
  bool ads1115_present, tmp36_present, found_stable_value;
  float photodiode_temp;

    
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  digitalWrite(greenLEDpin, HIGH);
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
  #if ECHO_TO_SERIAL
    Serial.print(m);         // milliseconds since start
    Serial.print(", ");  
  #endif

  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  #if ECHO_TO_SERIAL
    Serial.print(now.unixtime()); // seconds since 1/1/1970
    Serial.print(", ");
    Serial.print('"');
    Serial.print(now.year(), DEC);
    Serial.print("/");
    Serial.print(now.month(), DEC);
    Serial.print("/");
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    Serial.print(now.second(), DEC);
    Serial.print('"');
  #endif //ECHO_TO_SERIAL

  // Pyranometer temperature (TMP36)
  ads1115.setGain(GAIN_TWO);  // -2 V to 2 V
  ads1115_val_sum = 0;
  ads1115_val_avg = 0;
  ads1115_present = true;
  tmp36_present = true;
  found_stable_value = false;
  while (!found_stable_value) {
    for (int ii = 0; ii < TEMP_POLLING_LOOPS; ii++) {
      ads1115_val = ads1115.readADC_SingleEnded(2);
      if (ads1115_val == -1) {
        // Value of -1 indicates no ADS1115
        ads1115_present = false;
        found_stable_value = true;
        break;
      }
      if (ads1115_val < 4000) {
        // Values less than 250mV (-25 deg C) are assumed to be noise,
        // meaning there is no TMP36 connected to A2
        tmp36_present = false;
        found_stable_value = true;
        break;
      }
      ads1115_val_sum += ads1115_val;
    }
    if (ads1115_present && tmp36_present) {
      ads1115_val_avg = ads1115_val_sum / TEMP_POLLING_LOOPS;
      found_stable_value = true;
      ads1115_val_sum = 0;
      if (REQUIRE_STABLE_TEMP) {
        // If REQUIRE_STABLE_TEMP is true, loop again checking each
        // value for its deviation from the average. If any exceeds
        // MAX_STABLE_TEMP_ERR_PPM, the value is not stable, so retry
        // the whole thing.
        for (int ii = 0; ii < TEMP_POLLING_LOOPS; ii++) {
          ads1115_val = ads1115.readADC_SingleEnded(2);
          ppm_error_from_avg =
            (1000000 * abs(ads1115_val - ads1115_val_avg)) /
            abs(ads1115_val_avg);
          if (ppm_error_from_avg > MAX_STABLE_TEMP_ERR_PPM) {
            // If any value is more than MAX_STABLE_TEMP_ERR_PPM from
            // the average, we don't have a stable value
            if (!CSV_OUTPUT) {
              photodiode_temp = convert_ads1115_val_to_deg_c(ads1115_val_avg);
            }
            found_stable_value = false;
            break;
          }
        }
      }
    }
  }
  photodiode_temp = convert_ads1115_val_to_deg_c(ads1115_val_avg);

  // Irradiance (PDB-C139)
  if (ads1115_present) {
    ads1115.setGain(GAIN_EIGHT); // -512 mV to 512 mV
    ads1115_val_sum = 0;
    ads1115_val_avg = 0;
    found_stable_value = false;
    while (!found_stable_value) {
      for (int ii = 0; ii < IRRADIANCE_POLLING_LOOPS; ii++) {
        ads1115_val = ads1115.readADC_Differential_0_1();
        ads1115_val_sum += ads1115_val;
      }
      ads1115_val_avg = ads1115_val_sum / IRRADIANCE_POLLING_LOOPS;
      found_stable_value = true;
      ads1115_val_sum = 0;
      // If REQUIRE_STABLE_IRRAD is true, loop again checking each
      // value for its deviation from the average. If any exceeds
      // MAX_STABLE_IRRAD_ERR_PPM, the value is not stable, so retry
      // the whole thing.
      if (REQUIRE_STABLE_IRRAD) {
        for (int ii = 0; ii < IRRADIANCE_POLLING_LOOPS; ii++) {
          ads1115_val = ads1115.readADC_Differential_0_1();
          ppm_error_from_avg =
            (1000000 * abs(ads1115_val - ads1115_val_avg)) /
            abs(ads1115_val_avg);
          if (ppm_error_from_avg > MAX_STABLE_IRRAD_ERR_PPM) {
            // If any value is more than MAX_STABLE_IRRAD_ERR_PPM from
            // the average, we don't have a stable value
            found_stable_value = false;
            break;
          }
        }
      }
    }
 } 
  
  // Delay before next iteration
  delay(MS_DELAY_BETWEEN_LOOPS);

logfile.print(", ");    
logfile.print(convert_ads1115_val_to_irradiance(ads1115_val, true));
  
#if ECHO_TO_SERIAL
    Serial.print(", ");   
    Serial.print(convert_ads1115_val_to_irradiance(ads1115_val, true));
#endif //ECHO_TO_SERIAL

  logfile.println();
  #if ECHO_TO_SERIAL
    Serial.println();
  #endif // ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);

}

float convert_ads1115_val_to_deg_c(int16_t ads1115_val) {
  int max_millivolts;
  float tmp36_millivolts;
  float deg_c, temp_diff, multiplier;

  if (ads1115_val == 0) {
    return INVALID_TEMP; 
  }

  // First convert reading value to millivolts
  max_millivolts = (ADS1115_UNITY_GAIN_MAX_MILLIVOLTS /
                    ADS1115_PGA_GAIN_TMP36);
  tmp36_millivolts =
    (max_millivolts * (float)ads1115_val) / ADS1115_MIN_VALUE;

  // Then convert millivolts to degrees C
  deg_c = ((tmp36_millivolts - TMP36_OFFSET_MILLIVOLTS) /
           TMP36_MV_PER_DEG_C);

  // Calculate photodiode scaling factor
  temp_diff = deg_c - PHOTODIODE_NOMINAL_DEG_C;
  multiplier = PHOTODIODE_PCT_PER_DEG_C / 100;
  photodiode_temp_scaling_factor = temp_diff * multiplier + 1.0;

  // Return value
  return deg_c;
}

float convert_ads1115_val_to_irradiance(int16_t ads1115_val, bool temp_comp) {
  int max_millivolts;
  float photodiode_millivolts;
  float temp_scaling;
  float scaled_photodiode_millivolts;
  float adjusted_millivolts;
  float w_per_m_squared;

  // First convert reading value to millivolts
  photodiode_millivolts = convert_ads1115_val_to_millivolts(ads1115_val);

  // Conditionally apply temperature scaling
  if (temp_comp) {
    temp_scaling = photodiode_temp_scaling_factor;
  } else {
    temp_scaling = 1.0;
  }
  scaled_photodiode_millivolts =
    photodiode_millivolts * temp_scaling;

  // Then convert millivolts to irradiance
  //
  // Polynomial curve:
  //
  //   y = Ax^2 + Bx
  //
  //   x: scaled_photodiode_millivolts
  //   A: PYRANO_CAL_A
  //   B: PYRANO_CAL
  //   y: w_per_m_squared (irradiance)
  //
  // If A is 0, scaling is linear.  Intercept is always zero.
  //
  w_per_m_squared = ((PYRANO_CAL_A *
                      scaled_photodiode_millivolts *
                      scaled_photodiode_millivolts) +
                     (PYRANO_CAL *
                      scaled_photodiode_millivolts));

  return w_per_m_squared;
}

float convert_ads1115_val_to_millivolts(int16_t ads1115_val) {
  int max_millivolts;
  float photodiode_millivolts;

  max_millivolts = (ADS1115_UNITY_GAIN_MAX_MILLIVOLTS /
                    ADS1115_PGA_GAIN_PDB_C139);
  photodiode_millivolts =
    max_millivolts * (float(abs(ads1115_val)) / ADS1115_MIN_VALUE);

  return photodiode_millivolts;
}
