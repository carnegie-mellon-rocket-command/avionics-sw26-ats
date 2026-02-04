/*
Carnegie Schmellon Rocketry Club: Crewed Astrolab for Nutritional and Terrestrial Analytics in 
                                  Landing Operations for Unveiling Planetary Exploration ("CANTALOUPE") data logger

Made by the 2026 Avionics team :D (adapting on code from the 2025 Avionics team)
*/
//hi Sophia
// ***************** LIBRARIES *****************
#include <Servo.h>
#include <SD.h>
#include <Adafruit_BMP3XX.h> // version 1.1.2
#include <Adafruit_LSM6DSOX.h>
#include <BasicLinearAlgebra.h> // version 3.7
#include <Kalman.h>
#include <cassert>

// ***************** META *****************
// ⚠⚠⚠ VERY IMPORTANT ⚠⚠⚠
// true sets subscale altitude target, false sets fullscale altitude target
#define SUBSCALE true

// ⚠⚠⚠ IMPORTANT ⚠⚠⚠ 
// true will NOT actually gather data, only simulate it for testing purposes  
// false will gather data, FOR LAUNCH
#define SIMULATE false

// Simulation mode libraries
#if SIMULATE
    #include <Dictionary.h>
    Dictionary * m_simSensorValues = new Dictionary();
#endif

// Assertions (for debugging)
#if ENABLE_ASSERTS
    #define requires(condition) assert(condition)
    #define ensures(condition) assert(condition)
#else
    #define requires(condition) ((void)0)
    #define ensures(condition) ((void)0)
#endif

// ⚠⚠⚠ Do not create variables with same name as linalg library ⚠⚠⚠ 
using namespace BLA;

// ***************** UNITS (in IPS) *****************
#define SEA_LEVEL_PRESSURE_HPA 1018.0f
#define METERS_TO_FEET 3.28084f
#define ATMOSPHERE_FLUID_DENSITY 0.076474f // lbs/ft^3
#define GRAVITY 32.174f // ft/s^2

// ***************** CONSTANTS *****************
#define ROCKET_DRAG_COEFFICIENT 0.46f // Average value from OpenRocket
#define ROCKET_CROSS_SECTIONAL_AREA 0.0490873852f // The surface area (ft^2) of the rocket facing upwards
#if SUBSCALE
    #define ROCKET_MASS 11.28125f // lbs in dry mass (with engine housing but NOT propellant, assuming no ballast)
#else
    #define ROCKET_MASS 16.5f // lbs in dry mass (with engine housing but NOT propellant, assuming no ballast)
#endif
// #define ROCKET_MASS 19.5625f // lbs in dry mass (with engine housing but NOT propellant)
#define MAX_FLAP_SURFACE_AREA 0.0479010049f
#define ATS_MAX_SURFACE_AREA MAX_FLAP_SURFACE_AREA + ROCKET_CROSS_SECTIONAL_AREA // The maximum surface area (ft^2) of the rocket with flaps extended, including rocket's area

// Kalman filter parameters
#define NumStates 3
#define NumObservations 2
#define AltimeterNoise 1.0 // TODO: change
#define IMUNoise 1.0       // TODO: change

// Model covariance          (TODO: change)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8

// ***************** GLOBALS *****************
#define SKIP_ATS false // Whether the rocket is NOT running ATS, so don't try to mount servos, etc.
#define ENABLE_ASSERTS true

// ***************** FLIGHT PARAMETERS *****************
#define const bool DEBUG = false; // Whether to print debugging messages to the serial monitor (even if SIMULATE is off)
const int LOOP_TARGET_MS = 30; // How frequently data should be collected (in milliseconds)

// Target altitude in feet
#if SUBSCALE
    const float ALT_TARGET = 3750.0f; // ft
#else
    const float ALT_TARGET = 4500.0f; // ft above launch pad
#endif
const float ACCEL_THRESHOLD = 3 * GRAVITY; // Acceleration threshold for launch detection (ft/s^2)
const float VELOCITY_THRESHOLD = 0.1f;     // Velocity threshold for landing detection (ft/s)

// ***************** PIN DEFINITIONS *****************
const int ATS_PIN = 6;
const int LED_PIN = LED_BUILTIN;
#if SUBSCALE
    const int altimeter_chip_select = 1;  // BMP
#else
    const int altimeter_chip_select = 10; // BMP
#endif

// ***************** ATS SERVO PARAMETERS *****************
Servo m_atsServo;
float gATSPosition = 0.0f;
const int ATS_MIN = 180;
const int ATS_MAX = 13; // 254 constraint from flaps / 270 (servo max) * 180 (library function mapping)
const float ATS_IN = 0.0f;
const float ATS_OUT = 1.0f;

// ***************** SD CARD PARAMETERS *****************
#if SUBSCALE
  const int chip_select = 0;
#else
  const int chip_select = BUILTIN_SDCARD;
#endif
bool sd_active = false;
#if SUBSCALE
    String file_name = "subscl.txt"; // ⚠⚠⚠ FILE NAME MUST BE 8 CHARACTERS OR LESS OR ARDUINO CANNOT WRITE IT (WHY?!?!) ⚠⚠⚠
#else
    String file_name = "final.txt";
#endif

// ***************** SENSOR OBJECTS *****************
Adafruit_BMP3XX m_bmp;   // Altimeter
Adafruit_LSM6DSOX m_sox; // IMU
sensors_event_t accel, gyro, temp;

// ***************** MEASUREMENT VARIABLES *****************
String gBuffer; // Keeps track of data until it is written to the SD card
const int buffer_size = 50; // Number of measurements to take before writing to SD card
unsigned long gStartTime, gCurrTime, gTimer, gTimeDelta, gPrevLoopTime = 0; // Keeps track of time to make sure we are taking measurements at a consistent rate
float gAltFiltered, gVelocityFiltered, gAccelFiltered, gPredictedAltitude; // Filtered measurements shall be kept as global variables; raw data will be kept local to save memory

// Internal stuff for the Kalman Filter
float altitude_filtered_previous, acceleration_filtered_previous = 0.0f;
float gain_altitude, gain_acceleration, cov_altitude_current, cov_acceleration_current, cov_altitude_previous, cov_acceleration_previous = 0.0f;
float variance_altitude, variance_acceleration = 0.1f; // Might want to change these based on experiments or by calculating in flight
float previous_velocity_filtered = 0.0f; // We don't necessarily need this variable at this point, but it will be used when more advanced filtering techniques are implemented
bool gLaunched, gLanded; // Remembers if the rocket has launched and landed
unsigned long gLaunchTime;
unsigned long land_time = 0;
float absolute_alt_target = ALT_TARGET;

// Kalman filter stuff
BLA::Matrix<NumObservations> obs; // Observation vector
KALMAN<NumStates,NumObservations> KalmanFilter; // Kalman filter
BLA::Matrix<NumStates> measurement_state;


// ***************** ENTRY POINT TO THE PROGRAM *****************
/** @brief Initializes all devices, test devices and ATS */ 
void setup() {
    LEDSetup();

    // Setup serial terminal
    Serial.begin(115200); // Baud rate (bps)
    Serial.println("Initializing...");

    // Initalize Simulator
    #if SIMULATE
        startSimulation();
    #endif

    // Initialize SD card
    if (!initializeSDCard()) {
        Serial.println("SD card setup failed. Aborting.");
        LEDError();
    }

    // Initialize sensors
    if (!setupSensors()) {
        Serial.println("Sensor setup failed. Aborting.");
        LEDError();
    }
    
    // Test if all sensors on Altimeter works
    if (!testSensors()) {
        Serial.println("One or more altimeter sensors are not working");
        LEDError();
    }
    
    Serial.println("All sensors working!");
    testATS();

    // Initalize time evolution matrix
    KalmanFilter.F = {1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};
    // measurement matrix (first row: altimeter, second row: accelerometer)
    KalmanFilter.H = {1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0};
    // measurement covariance matrix
    KalmanFilter.R = {AltimeterNoise*AltimeterNoise, 0.0,
                      0.0,                           IMUNoise*IMUNoise};
    // model covariance matrix
    KalmanFilter.Q = {m_p*m_p, 0.0,     0.0,
                      0.0,     m_s*m_s, 0.0,
                      0.0,     0.0,     m_a*m_a};

    obs.Fill(0.0);
    measurement_state.Fill(0.0);

    Serial.println("Arduino is ready!");
    LEDSuccess();
    gStartTime = millis();
}

// Repeats indefinitely after setup() is finished
/** @brief Collect/log data and control ATS */
void loop() {
    gBuffer = "";
    for (int i = 0; i < buffer_size; i++) {
        runTimer(); // Ensures loop runs at a consistent rate

        // Get measurements from sensors and add to buffer
        gBuffer = gBuffer + getMeasurements() + "\n";

        // **** (PRE-FLIGHT) ****
        // Detect launch based on acceleration threshold
        if (gAccelFiltered > ACCEL_THRESHOLD && !gLaunched) {
            // Write CSV header to the file
            writeData("***************** START OF DATA ***************** TIME SINCE BOOT: " + String(millis()) + " ***************** TICK SPEED: " + String(LOOP_TARGET_MS) + "ms\n");
            writeData("time, pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, gAltFiltered (ft), gVelocityFiltered (ft/s), gAccelFiltered (ft/s^2), temperature (from IMU, degrees C), gATSPosition (servo degrees), gAltPredicted (ft)\n");

            #if (DEBUG) {Serial.println("Rocket has launched!");}
            gLaunched = true;
            gLaunchTime = millis();
            
            // Bring the ATS back online
            attachATS();
            setATSPosition(ATS_IN);

            // Set status LED
            LEDLogging();
        }

        // **** (DURING FLIGHT) ****
        // If the rocket has launched, adjust the ATS as necessary, and detect whether the rocket has landed
        if (gLaunched) {
            LEDFlying();
            adjustATS();
            if (detectLanding()) {
                gLanded = true;
            }
        }
        else {
            // If we are still on the pad, measure the altitude of the launch pad
            absolute_alt_target = ALT_TARGET + gAltFiltered;
        }
    }

    if (gLaunched) {
        writeData(gBuffer);
    }

    // **** (END OF FLIGHT) ****
    if (gLanded) {
        // End the program
        detachATS();
        #if (DEBUG) {Serial.println("Rocket has landed, ending program");}
        while (true);
    }
}


/** @brief Ensures data collected at same rate
  * Ensures each iteration in arduino main loop for loop runs at same rate */
void runTimer() {
    long tempTime = millis() - gPrevLoopTime;
    #if (DEBUG) {Serial.println("Current loop time (before comparison): " + String(tempTime));}
    if (tempTime < LOOP_TARGET_MS) {
        delayMicroseconds((LOOP_TARGET_MS - tempTime) * 1000);
    }
    else if (tempTime > LOOP_TARGET_MS + 1) {
        Serial.println("Board is unable to keep up with target loop time of " + String(LOOP_TARGET_MS) + " ms (execution took "+ String(tempTime) + " ms)");
    }
    gCurrTime = millis();
    gTimer = gCurrTime - gStartTime;
    gTimeDelta = gCurrTime - gPrevLoopTime;
    #if (DEBUG) {Serial.println("Current loop time (after comparison): " + String(loop_time));}
    gPrevLoopTime = gCurrTime;
}


// ***************** SD CARD METHODS *****************
/** @brief Initialize the SD card  
  * Right now, tries to connect 10 times before going into an error state; could change so it keeps trying indefinitely */
bool initializeSDCard() {
    #if SIMULATE
        // If simulation mode is active, don't try to connect to any hardware
        Serial.println("(Simulation) SD card initialized successfully!");
        return true;

        // Use this instead to simulate pain (pain is realistic)
        // return false;
    #endif

    #if (DEBUG) {Serial.println("Initializing SD card...");}

    for (int i = 0; i < 10; i++) {
        if (SD.begin(chip_select)) {
            break;
        }
        #if (DEBUG) {Serial.println("SD card initialization failed. Trying again...");}
        delay(1000);
        if (i == 9) {
            Serial.println("SD card initialization failed 10 times. Aborting.");
            return false;
        }
    }
    #if (DEBUG) {Serial.println("SD card initialized successfully!");}
    sd_active = true;
    return true;
}

/** @brief Write data to SD card in "datalog.txt" file */
void writeData(String text) {
    #if SIMULATE
        Serial.println("(Simulation) Data to SD card: " + text);
        return;
    #endif

    if (sd_active) {
        File data_file = SD.open("subscl_1.txt", FILE_WRITE);
        if (data_file) {
            #if (DEBUG) {Serial.println("Writing to SD card!");}
            data_file.print(text);
            data_file.close();
        }
        else {
            // Could leave this out so the program tries to keep logging data even if it fails once
            // sd_active = false;
            Serial.println("Error opening subscl_1.txt");
        }
    }
    else {
        // If the SD card has not connected successfully
        #if (DEBUG) {Serial.println("SD logging failed. Continuing without logging.");}
    }
}


// ***************** SENSOR SETUP METHODS *****************
/** @brief Initialize all sensors */
bool setupSensors() {
    #if SIMULATE
        Serial.println("(Simulation) Sensors connected successfully!");
        return true;
    #endif
    
    if (setupBMP3XX() && setupLSM6DSOX()) {
        #if (DEBUG) {Serial.println("Sensors initialized successfully!");}
        m_bmp.performReading();
        #if (DEBUG) {Serial.println("Setup reading good");}
        return true;
    }
    return false;
}

/** @brief Test Altimeter and IMU */
bool testSensors() {
    #if SIMULATE
        return true;
    #endif
    if (!m_bmp.performReading()) {
        Serial.println("One or more altimeter sensors are not working");
        return false;
    }
    if (!m_sox.getEvent(&accel, &gyro, &temp)) {
        Serial.println("One or more IMU sensors are not working");
        return false;
    }
    return true;
}

/** @brief Setup Altimeter */
bool setupBMP3XX() {
    if (!m_bmp.begin_SPI(altimeter_chip_select)) {
        Serial.println("Unable to connect to altimeter");
        return false;
    }
    m_bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    m_bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    m_bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    m_bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    #if (DEBUG) {Serial.println("Altimeter good");}
    m_bmp.performReading();
    Serial.println("Altitude: " + String(m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA)));
    return true;
}

/** @brief Setup IMU */
bool setupLSM6DSOX() {
    if (!m_sox.begin_I2C()) {
        Serial.println("Unable to connect to IMU");
        return false;
    }
    #if (DEBUG) {Serial.println("IMU good");}
    return true;
}


// ***************** DATA COLLECTION/LOGGING METHODS *****************
/** @brief Refresh sensors and update globals with latest measurements
  * Output string can be viewed when DEBUG is true
  * @returns a CSV string with format:
  * - time, pressure (hPa), altitude_raw (m), acceleration_raw_x (m/s^2), 
  * - acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, 
  * - gAltFiltered, gVelocityFiltered, gAccelFiltered, 
  * - temperature (from IMU, degrees C), gATSPosition (degrees) */
String getMeasurements() {
    #if SIMULATE
        // If simulation mode is ON, get simulated data from serial bus instead
        return getSimulatedMeasurements();
    #endif

    // Update sensors
    m_bmp.performReading();               // Update Altimeter
    m_sox.getEvent(&accel, &gyro, &temp); // Update IMU
    
    // Pass raw data to be filtered
    filterData(readAltimeter(), readIMU()); 

    // Print measurements
    String movementData = String(m_bmp.pressure/100.0) + ", " + 
                          String(m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA)) + ", " + 
                          String(accel.acceleration.x*METERS_TO_FEET) + ", " + 
                          String(accel.acceleration.y*METERS_TO_FEET) + ", " + 
                          String(accel.acceleration.z*METERS_TO_FEET) + ", " + 
                          String(gyro.gyro.x) + ", " + 
                          String(gyro.gyro.y) + ", " + 
                          String(gyro.gyro.z) + ", " + 
                          String(gAltFiltered) + ", " + 
                          String(gVelocityFiltered) + ", " + 
                          String(gAccelFiltered);
    String timeData = String(millis() - gStartTime);
    String sensorData = String(readThermometer());
    #if (DEBUG) {Serial.println("Time: " + timeData + ", " + "Movement: " + movementData + ", " + "Sensors: " + sensorData + ", " + "ATS position: " + String(gATSPosition));}
    return timeData + ", " + movementData + ", " + sensorData + ", " + String(gATSPosition) + ", " + String(gPredictedAltitude);
}

/** @brief Detect if rocket has landed 
  * If the rocket has been reasonably still for 5 seconds, it is considered landed */
bool detectLanding() {
    if (gLaunchTime != 0 && millis() - gLaunchTime > 300000) { 
        return true;
    } 
    return false;
}

/** @brief Filter raw data and updates globals
  * still need to implement (TO-DO)*/ 
void filterData(float alt, float acc) {
    float DT = ((float)gTimeDelta) / 1000;

    KalmanFilter.F = {1.0, DT,  DT*DT / 2,
                      0.0, 1.0, DT,
                      0.0, 0.0, 1.0};

    measurement_state(0) = (double)alt;
    measurement_state(1) = (double)0.0;
    measurement_state(2) = (double)acc;

    obs = KalmanFilter.H * measurement_state;
    KalmanFilter.update(obs);
    // BLA::Matrix<NumObservations, 3 > current_obs = {alt, 0.0, 0.0,
    //                                                 0.0, 0.0, acc};
    // KalmanFilter.update(current_obs);
    gAltFiltered = KalmanFilter.x(0);
    gVelocityFiltered = KalmanFilter.x(1);
    gAccelFiltered = KalmanFilter.x(2);
    // Serial << alt << "," << acc <<"," << gAltFiltered << "," << gVelocityFiltered << "," << gAccelFiltered << "\n";
    Serial << "Current accel: " << gAccelFiltered << "\n";
    // Serial.println("Filtered Altitude: " + String(gAltFiltered) + " Filtered Velocity: " + String(gVelocityFiltered) + " Filtered Acceleration: " + String(gAccelFiltered));
}

/** @brief  Read altitude from altimeter
  * IMPORTANT: update altimeter with `performreading()` beforehand
  * @returns raw altitude (ft) */
float readAltimeter() {
    float altimeter_data = m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA) * METERS_TO_FEET;
    #if (DEBUG) {Serial.println("Altimeter: " + String(altimeter_data));}
    return altimeter_data;
}

/** @brief Read acceleration from IMU 
  * IMPORTANT: update IMU with `getevent()` first for current reading
  * @returns raw acceleration (ft/s^2) */
float readIMU() {
    float imu_data = pow(pow(accel.acceleration.x,2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2), 0.5) * METERS_TO_FEET;
    #if (DEBUG) {Serial.println("IMU: " + String(imu_data));}
    return imu_data;
}

/** @brief Read temperature from IMU
  * IMPORTANT: update IMU with `getevent()` beforehand
  * @returns temperature (degrees C) */
float readThermometer() {
    float thermometer_data = temp.temperature;
    #if (DEBUG) {Serial.println("Thermometer: " + String(thermometer_data));}
    return thermometer_data;
}

/** @brief Runs PID
  * @arg error: acceleration error
  * @arg Kp: proportional gain
  * @arg Kd: derivative gain
  * - positive error means rocket is going too slow
  * - negative error means rocket is going too fast
  * - Kp: higher = stronger immediate response
  * - Kd: adjusts response based on how frequently error changes
  * @return the absolute position to adjust ATS to in degrees */
float PIDFactor(int error, float Kp, float Kd) {
    static int old_error = 0;
    float proportional = error * Kp;
    float derivative = (error - old_error) * Kd;
    old_error = error;
    return proportional + derivative; 
}


// ***************** ATS METHODS *****************
/** @brief Test the ATS: fully extend and retract it, then detach the servo */
void testATS() {
    #if (DEBUG) {Serial.println("Testing ATS...");}
    attachATS();
    setATSPosition(ATS_IN);
    delay(2000);
    setATSPosition(ATS_OUT); // Initial position
    delay(2000);
    setATSPosition(ATS_IN);  // Reset position
    delay(2000);
    detachATS();
    Serial.println("ATS Test Sucessful...");
}

/** @brief Turn on ATS servo */
void attachATS() {
    requires(!m_atsServo.attached());
    #if SKIP_ATS
        return;
    #endif
    m_atsServo.attach(ATS_PIN);
}

/** @brief Set ATS position  
  * @arg percent_rot: float between 0 (fully in) and 1 (fully out)
  * Only call this function when servo position changes */
void setATSPosition(float percent_rot) {
    #if SKIP_ATS
        return;
    #endif
    float pos = ATS_MIN - (ATS_MIN - ATS_MAX) * (1.0f - percent_rot);
    m_atsServo.write(int(pos));
    #if (DEBUG) {Serial.println("ATS position set to " + String(pos));}
}

/** @brief Adjust ATS with PID
  * Adjust the ATS based on the current altitude and desired apogee (stored in absolute_alt_target; the ALT_TARGET represents altitude above the launch level, while absolute_alt_target is the altitude above sea level) */
void adjustATS() {
    float targetAcceleration = abs(pow(gVelocityFiltered,2)/(2*(absolute_alt_target-gAltFiltered)));

    // Retract ATS fully after 18 seconds
    if (millis() - gLaunchTime > 18000) {
        setATSPosition(ATS_IN);
        gATSPosition = ATS_IN;
        delay(10);
        return;
    }
    
    // Fully deploy ATS if reached Altitude target
    if (gAltFiltered >= absolute_alt_target) {
        gATSPosition = ATS_OUT;
    }
    else {
        // Calculate desired surface-area to reach target altitude
        float target_area = (pow(gVelocityFiltered, 2)/(absolute_alt_target - gAltFiltered) - 2*GRAVITY)*ROCKET_MASS/(gVelocityFiltered*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT);
        #if (DEBUG) {Serial.println("Old ATS position: " + String(target_area/ATS_MAX_SURFACE_AREA));}

        // Calculate error in acceleration
        float error = gAccelFiltered - targetAcceleration; // Positive if drag + gravity >= target; means if <, we are going TOO FAST and need to slow down

        // Calculate adjustment
        float adjustment;
        if (error > 0) { // Too slow
            adjustment = 0;
        } 
        else { // Too fast
            adjustment = PIDFactor(abs(error), 0.03, 0); // Should normalize to 0 to 1
        }

        gATSPosition = adjustment;
        gPredictedAltitude = (0.5*ROCKET_MASS*pow(gVelocityFiltered, 2))/(ROCKET_MASS*GRAVITY + 0.5*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT*pow(gVelocityFiltered,2)*ATS_MAX_SURFACE_AREA*adjustment);
        #if (DEBUG) {Serial.println("Target acceleration: " + String(targetAcceleration));}
        #if (DEBUG) {Serial.println("Predicted altitude: " + String(gPredictedAltitude));}
    }

    // ATS window
    if (millis() - gLaunchTime > 4500) {
        if (millis() - gLaunchTime < 6000) {
            // Adjust ATS based on position
            setATSPosition(ATS_IN);
            delay(500);
            setATSPosition(ATS_OUT); // Initial position
            delay(500);
            setATSPosition(ATS_IN);  // Reset position
            delay(500);
            #if (DEBUG) {Serial.println("Cycling ATS position");}
        }
        else {
            // Adjust ATS based on position
            setATSPosition(gATSPosition);
            #if (DEBUG) {Serial.println("ATS position: " + String(gATSPosition));}
        }
    }
}

/** @brief Turn off ATS servo (for saving power) */
void detachATS() {
    requires(m_atsServo.attached());
    #if SKIP_ATS
        return;
    #endif
    m_atsServo.detach();
}


// ***************** STATUS LED METHODS *****************
void LEDSetup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}

void LEDError() {
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
}

void LEDSuccess() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_PIN, LOW);
        delay(250);
        digitalWrite(LED_PIN, HIGH);
        delay(250);
    }
}

void LEDLogging() {
    digitalWrite(LED_PIN, HIGH);
}

void LEDFlying() {
    digitalWrite(LED_PIN, HIGH);
}