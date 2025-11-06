#include <Wire.h>

#define ADS1115_ADDRESS 0x48

//=================================================================================================================================
// VARIABLE DECLARATIONS
//=================================================================================================================================

// Define the pins of the Arduino ADC where the current sensor is measured
const int SensorPin = A1, RefPin = A0;

// Define the data from the current sensor
const int Rshunt = 33.3;                // Resistance of the transformer: Model 50 A: 20 ohms, Model 30 A: 33.3 ohms
double n_trafo = 1000.0;                  // Number of turns between primary and secondary

// Variables to calculate every millisecond
unsigned long time_now = 0;
unsigned long time_ant = 0, difTime = 0, act_time = 0, reading_time = 0, dif_reading_time = 0, timer1 = 0, timer2 = 0;
 
// Define variables to calculate the RMS of a power cycle
double quadratic_sum_v = 0.0;
double quadratic_sum_rms = 0.0;       // This variable accumulates the quadratic sum of instantaneous currents
const int sampleDuration = 20;          // Number of samples that determine how often the RMS is calculated
int quadratic_sum_counter = 0;       // Counter of how many times values have been accumulated in the quadratic sum
double freq = 50.0;                     // Define the frequency of the power cycle


// Define variables to calculate an average of the current
double accumulated_current = 0.0;       // Accumulator of RMS values for averaging
const int sampleAverage = 250;          // Number of samples that determine how often the RMS average is calculated
int accumulated_counter = 0;             // Counter of how many times RMS values have been accumulated
bool first_run = true;
double v_calib_acum = 0.0;
double v_calib = 0.0;
int i = 0;
byte writeBuf[3];
int Imessurement_counter = 0; 



//=================================================================================================================================
// Helper functions: Function created to partition the problem in smaller parts
//=================================================================================================================================
void config_i2c(){
  Wire.begin(); // begin I2C

  // ASD1115
  // set config register and start conversion
  // ANC1 and GND, 4.096v, 128s/

  writeBuf[0] = 1;    // config register is 1
  
  writeBuf[1] = 0b11010010; // 0xC2 single shot off <== ORIGINAL - single conversion/ AIN1 & GND/ 4.096V/ Continuous (0)
  
  // bit 15 flag bit for single shot
  // Bits 14-12 input selection:
  // 100 ANC0; 101 ANC1; 110 ANC2; 111 ANC3
  // Bits 11-9 Amp gain. Default to 010 here 001 P19
  // Bit 8 Operational mode of the ADS1115.
  // 0 : Continuous conversion mode
  // 1 : Power-down single-shot mode (default)

  writeBuf[2] = 0b11100101; // bits 7-0  0x85 //869 SPS 
  
  // Bits 7-5 data rate default to 100 for 128SPS
  // Bits 4-0  comparator functions see spec sheet.

  // setup ADS1115
  Wire.beginTransmission(ADS1115_ADDRESS);  // ADC 
  Wire.write(writeBuf[0]); 
  Wire.write(writeBuf[1]);
  Wire.write(writeBuf[2]);  
  Wire.endTransmission();  

  delay(500);
}

float read_voltage(){
  //unsigned long start = micros();
  // Read conversion register
  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(0x00); // Conversion register
  Wire.endTransmission();

  Wire.requestFrom(ADS1115_ADDRESS, 2);
  int16_t result = Wire.read() << 8 | Wire.read();  // Mount the 2 byte value
  Wire.endTransmission();
  
  //unsigned long end = micros();
  //Serial.print("ADC Read Time (us): ");
  //Serial.println(end - start);

  // Convert result to voltage
  float voltage = result * 4.096 / 32768.0;  // Raw adc * reference voltage configured / maximum adc value
  return voltage;  // Voltage in V
}

// function to get the voltage at the reference point (calibration)
float getReferenceVoltage(){
  float Vref = 0.0;
  // Take the average of 10 readings
  for(short i = 0; i < 10 ; i++){
    // Acumulate the voltage value converting from maximumvalue from analog read to maximum voltage value
    Vref += analogRead(RefPin)*3.3/4095.0;
    // Leave a 1s gap between readings
    delay(1000);
  }
  // Take the average
  Vref /= 10.0;
  // Multiply by empirical constant found contrasting readings with multimeter measurements
  Vref *= 1.122;
  Serial.print("New Voltage Reference: ");
  Serial.print(Vref);
  Serial.println("[V]");
  return Vref;
}


//=================================================================================================================================
// setup Function: Function that runs once on startup
//=================================================================================================================================
void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  Serial.println("Starting\n\n\n");
  // Initialize IIC communications
  config_i2c();
  // Get the calibrated voltage value
  pinMode(RefPin, INPUT);
  v_calib = getReferenceVoltage();
}

//=================================================================================================================================
// loop Function: Function that runs cyclically indefinitely
//=================================================================================================================================
void loop() {
  
  double Irms;
  // Read the time in microseconds since the Arduino started
  time_now = micros();
  // Calculate the time difference between the current time and the last time the instantaneous current was updated
  difTime = time_now - time_ant;

  // EVERY 1 MILLISECOND, READ ADC AND CALCULATE THE INSTANTANEOUS CURRENT TO CALCULATE THE RMS
  if (difTime >= 1000) {

    // Update the time record with the current time
    time_ant = time_now;
    
    // Read the voltage from the sensor, using the calibration voltage
    double v = read_voltage() - v_calib;

    // Convert voltage in shunt to current measurement
    double curr = v*30.0;

    // Accumulate cuadratic sum
    quadratic_sum_rms += curr*curr*difTime/1000000.0;
    quadratic_sum_counter++; 
    
    // Debuging
    //Serial.print("V = ");
    //Serial.println(v);
    //Serial.print("I = ");
    //Serial.println(curr);
    //delay(1000);
    
  }
  
  // EVERY POWER CYCLE (20 ACCUMULATED VALUES), CALCULATE RMS
  if (quadratic_sum_counter >= sampleDuration) { //20[ms] = 1/(50[Hz])

    // Take the square root to calculate the RMS of the last power cycle
    Irms = sqrt(quadratic_sum_rms*freq);
    // Debugging
    //Serial.print("Irms = ");
    //Serial.println(Irms);
    //delay(1000);
    // Reset accumulation values to calculate the RMS of the last power cycle
    quadratic_sum_counter = 0;
    quadratic_sum_rms = 0;

    // Filter base error
    // Empirically, the highest value of noise found was close to 0.17 [A]
    if (Irms <= 0.17){
      Irms = 0;
    }
    // Accumulate RMS current values to calculate the average RMS
    accumulated_current += Irms;
    accumulated_counter++;
    //Serial.println(Irms);
  }

  
  // EVERY 250 POWER CYCLES (approximately 5 seconds), CALCULATE THE AVERAGE RMS
  if (accumulated_counter >= 250) {

    // Calculate the average of the RMS current

    double Ifilt = accumulated_current/(double)accumulated_counter;
    
    // Reset accumulation values to calculate the average RMS
    accumulated_counter = 0;
    accumulated_current = 0.0;
    
    // Print the filtered current
    Serial.print("I = ");
    Serial.print(Ifilt);
    Serial.println(" [A]");
  }

  

}
