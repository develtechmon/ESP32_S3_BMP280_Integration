/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is" without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  5 February 2023 -  initial release
*/

#include <Wire.h>

// Define your custom I2C pins
#define CUSTOM_SDA_PIN 4
#define CUSTOM_SCL_PIN 5

// Circular buffer settings
#define BUFFER_SIZE 26  // Number of pressure readings to average

// Adaptive filter settings
#define DEADBAND_CM 2.0          // Ignore changes smaller than this (noise threshold)
#define SLOW_CHANGE_THRESHOLD 5.0   // cm/sec - below this = slow movement
#define FAST_CHANGE_THRESHOLD 15.0  // cm/sec - above this = fast movement

// Filter coefficients (alpha values)
#define ALPHA_STATIONARY 0.01    // Heavy filtering when not moving (1% new, 99% old)
#define ALPHA_SLOW 0.05          // Medium filtering for slow movements
#define ALPHA_FAST 0.3           // Light filtering for fast movements
#define ALPHA_VERY_FAST 0.7      // Minimal filtering for very fast movements

// BMP280 calibration coefficients
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9; 

// Pressure and altitude variables
double PressureRaw;           // Raw pressure from sensor (hPa)
double PressureSmooth;        // Smoothed pressure from circular buffer
float AltitudeRaw;            // Altitude calculated from raw pressure
float AltitudeSmooth;         // Altitude calculated from smoothed pressure  
float AltitudeFinal;          // Final filtered altitude
float AltitudeBarometerStartUp;

// Adaptive filtering variables
float previousAltitudeSmooth = 0;
unsigned long previousTime = 0;
float currentVelocity = 0;
float currentAlpha = ALPHA_STATIONARY;
bool firstRun = true;

// Circular buffer variables (for PRESSURE, not altitude)
double pressureBuffer[BUFFER_SIZE];  // Buffer stores pressure values
int bufferIndex = 0;
bool bufferFull = false;
double bufferSum = 0;

int RateCalibrationNumber;

void barometer_signals(void){
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76,6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
  var2 = (var2>>2)+(((signed long int )dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
  if (var1 == 0) { p=0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2;  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));
  
  // Store RAW pressure (this is what we'll filter)
  PressureRaw = (double)p / 100.0;  // Convert to hPa
}

// Convert pressure to altitude using barometric formula
float pressureToAltitude(double pressure) {
  return 44330 * (1 - pow(pressure / 1013.25, 1/5.255)) * 100; // Result in cm
}

// Add new PRESSURE reading to circular buffer
void updatePressureBuffer(double newPressure) {
  // Remove the old pressure from running sum (if buffer was full)
  if (bufferFull) {
    bufferSum -= pressureBuffer[bufferIndex];
  }
  
  // Add new pressure reading to current position
  pressureBuffer[bufferIndex] = newPressure;
  bufferSum += newPressure;
  
  // Move to next position (wrap around if at end)
  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFull = true;  // Buffer is now full
  }
  
  // Calculate average pressure
  int validReadings = bufferFull ? BUFFER_SIZE : bufferIndex;
  PressureSmooth = bufferSum / validReadings;
}

// Adaptive filter with deadband and rate-based switching
void updateAdaptiveFilter() {
  unsigned long currentTime = millis();
  
  if (firstRun) {
    AltitudeFinal = AltitudeSmooth;
    previousAltitudeSmooth = AltitudeSmooth;
    previousTime = currentTime;
    firstRun = false;
    return;
  }
  
  // Calculate time difference in seconds
  float deltaTime = (currentTime - previousTime) / 1000.0;
  if (deltaTime <= 0) return; // Avoid division by zero
  
  // Calculate velocity (rate of change)
  float altitudeChange = AltitudeSmooth - previousAltitudeSmooth;
  currentVelocity = abs(altitudeChange) / deltaTime; // cm/sec
  
  // Apply deadband - ignore small changes (likely noise)
  if (abs(altitudeChange) < DEADBAND_CM) {
    // Within deadband - use heavy filtering to reject noise
    currentAlpha = ALPHA_STATIONARY;
  } else {
    // Outside deadband - adjust filter based on velocity
    if (currentVelocity < SLOW_CHANGE_THRESHOLD) {
      currentAlpha = ALPHA_SLOW;           // Slow movement
    } else if (currentVelocity < FAST_CHANGE_THRESHOLD) {
      currentAlpha = ALPHA_FAST;           // Fast movement  
    } else {
      currentAlpha = ALPHA_VERY_FAST;      // Very fast movement
    }
  }
  
  // Apply adaptive low-pass filter
  AltitudeFinal = (currentAlpha * AltitudeSmooth) + ((1.0 - currentAlpha) * AltitudeFinal);
  
  // Update previous values
  previousAltitudeSmooth = AltitudeSmooth;
  previousTime = currentTime;
}

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  
  // Configure BMP280
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();   
  Wire.beginTransmission(0x76);
  Wire.write(0xF5); 
  Wire.write(0x14);
  Wire.endTransmission();   
  
  // Read calibration coefficients
  uint8_t data[24], i=0; 
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);      
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0]; 
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6]; 
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11]<< 8) | data[10];
  dig_P4 = (data[13]<< 8) | data[12];
  dig_P5 = (data[15]<< 8) | data[14];
  dig_P6 = (data[17]<< 8) | data[16];
  dig_P7 = (data[19]<< 8) | data[18];
  dig_P8 = (data[21]<< 8) | data[20];
  dig_P9 = (data[23]<< 8) | data[22];
  
  delay(250);
  
  // Calibrate startup altitude using RAW pressure readings
  double startupPressureSum = 0;
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    barometer_signals();
    startupPressureSum += PressureRaw;
    delay(1);
  }
  double startupPressure = startupPressureSum / 2000.0;
  AltitudeBarometerStartUp = pressureToAltitude(startupPressure);
  
  // Initialize buffer variables
  bufferSum = 0;
  bufferIndex = 0;
  bufferFull = false;
  firstRun = true;
  
  // Print headers for Serial Plotter
  Serial.println("Raw_Altitude,Smooth_Altitude,Final_Altitude,Velocity,Alpha");
}

void loop() {
  // Step 1: Get raw pressure reading
  barometer_signals();
  
  // Step 2: Calculate raw altitude (for comparison)
  AltitudeRaw = pressureToAltitude(PressureRaw) - AltitudeBarometerStartUp;
  
  // Step 3: Apply circular buffer to PRESSURE, then convert to altitude
  updatePressureBuffer(PressureRaw);
  AltitudeSmooth = pressureToAltitude(PressureSmooth) - AltitudeBarometerStartUp;
  
  // Step 4: Apply adaptive filter with deadband
  updateAdaptiveFilter();
  
  // Print values for Serial Plotter comparison
  // Serial.print(AltitudeRaw);           // Raw altitude (noisy)
  // Serial.print(",");
  // Serial.print(AltitudeSmooth);        // Circular buffer smoothed
  // Serial.print(",");
  // Serial.print(AltitudeFinal);         // Adaptive filtered (USE THIS for altitude hold)
  // Serial.print(",");
  // Serial.print(currentVelocity);       // Current velocity (cm/sec)
  // Serial.print(",");
  // Serial.println(currentAlpha * 100);  // Current alpha value (scaled for visibility)

  Serial.print("Current_Velocity:");
  Serial.print(currentVelocity); 
  Serial.print(",");
         
  Serial.print("Current_Alpha_Velocity:");
  Serial.println(currentAlpha * 100);  
  Serial.print(",");
      
  Serial.print("Raw_Altitude:");
  Serial.println(AltitudeRaw);   
  Serial.print(",");
       
  Serial.print("Smooth_Altitude:");
  Serial.println(AltitudeSmooth);  
  Serial.print(",");
     
  Serial.print("Final_Altitude:");
  Serial.println(AltitudeFinal); 
  
  delay(50);
}
