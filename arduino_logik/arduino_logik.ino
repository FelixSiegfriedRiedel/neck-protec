/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <NeckProtec_inferencing.h>
#include <Arduino_LSM9DS1.h> //Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_lsm9ds1/
#include <ArduinoBLE.h>

// Festlegung von UUIDs
//Service
#define BLE_UUID_IMU_SERVICE      "0176b37a-1ede-4268-baae-ac5a3b1b2675"

//Rohdaten der Sensoren
#define BLE_UUID_ACC_CHAR         "19739973-4ea4-45fa-8d2d-408ffa6d73d3"
#define BLE_UUID_GYRO_CHAR        "705ae904-131f-4521-b0d8-267d17463e01"
#define BLE_UUID_MAG_CHAR         "619c0a15-3cd6-4043-af06-908f390f8779"

//Winkel des Accelerators
#define BLE_UUID_ACC_ANGLES      "d1fb867a-4653-4e62-950e-5f20c725e683"

//Klassifizierte Nackenposition
#define BLE_UUID_NECKPOS          "df6db541-5894-4eff-8384-087583af5bbf"


#define BLE_DEVICE_NAME           "NeckProtec"
#define BLE_LOCAL_NAME            "NeckProtec"

#define HISTORY_SIZE 12
float prediction_history[HISTORY_SIZE];
int prediction_index = 0;
int samples_collected = 0;
unsigned long lastInferenceTime = 0;
const unsigned long INFERENCE_INTERVAL_MS = 500;

unsigned long myTime;

BLEService IMUService(BLE_UUID_IMU_SERVICE);

BLEStringCharacteristic accCharacteristic(BLE_UUID_ACC_CHAR, BLERead | BLENotify, 50);
BLEStringCharacteristic gyroCharacteristic(BLE_UUID_GYRO_CHAR, BLERead | BLENotify, 50);
BLEStringCharacteristic magCharacteristic(BLE_UUID_MAG_CHAR, BLERead | BLENotify, 50);
BLEStringCharacteristic accAnglesCharacteristic(BLE_UUID_ACC_ANGLES, BLERead | BLENotify, 50);

BLEBoolCharacteristic neckPosCharacteristic(BLE_UUID_NECKPOS, BLERead | BLENotify);

String accStr, angleStr, gyrStr, magStr;  

enum sensor_status {
    NOT_USED = -1,
    NOT_INIT,
    INIT,
    SAMPLED
};

/** Struct to link sensor axis name to sensor value function */
typedef struct{
    const char *name;
    float *value;
    uint8_t (*poll_sensor)(void);
    bool (*init_sensor)(void);    
    sensor_status status;
} eiSensors;

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/**
 * When data is collected by the Edge Impulse Arduino Nano 33 BLE Sense
 * firmware, it is limited to a 2G range. If the model was created with a
 * different sample range, modify this constant to match the input values.
 * See https://github.com/edgeimpulse/firmware-arduino-nano-33-ble-sense/blob/master/src/sensors/ei_lsm9ds1.cpp
 * for more information.
 */
#define MAX_ACCEPTED_RANGE  2.0f

/** Number sensor axes used */
#define N_SENSORS     9

/* Forward declarations ------------------------------------------------------- */
float ei_get_sign(float number);

bool init_IMU(void);

uint8_t poll_acc(void);
uint8_t poll_gyr(void);
uint8_t poll_mag(void);

/* Private variables ------------------------------------------------------- */
static const bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

static float data[N_SENSORS];
static bool ei_connect_fusion_list(const char *input_list);

static int8_t fusion_sensors[N_SENSORS];
static int fusion_ix = 0;

/** Used sensors value function connected to label name */
eiSensors sensors[] =
{
    "accX", &data[0], &poll_acc, &init_IMU, NOT_USED,
    "accY", &data[1], &poll_acc, &init_IMU, NOT_USED,
    "accZ", &data[2], &poll_acc, &init_IMU, NOT_USED,
    "gyrX", &data[3], &poll_gyr, &init_IMU, NOT_USED,
    "gyrY", &data[4], &poll_gyr, &init_IMU, NOT_USED,
    "gyrZ", &data[5], &poll_gyr, &init_IMU, NOT_USED,
    "magX", &data[6], &poll_mag, &init_IMU, NOT_USED,
    "magY", &data[7], &poll_mag, &init_IMU, NOT_USED,
    "magZ", &data[8], &poll_mag, &init_IMU, NOT_USED,
};

/**
* @brief      Arduino setup function
*/
void setup()
{
    /* Init serial */
    Serial.begin(115200);

    // while (!Serial);
    Serial.println("Edge Impulse Sensor Fusion Inference\r\n");

    /* Connect used sensors */
    if(ei_connect_fusion_list(EI_CLASSIFIER_FUSION_AXES_STRING) == false) {
        ei_printf("ERR: Errors in sensor list detected\r\n");
        return;
    }

    /* Init & start sensors */

    for(int i = 0; i < fusion_ix; i++) {
        if (sensors[fusion_sensors[i]].status == NOT_INIT) {
            sensors[fusion_sensors[i]].status = (sensor_status)sensors[fusion_sensors[i]].init_sensor();
            if (!sensors[fusion_sensors[i]].status) {
              ei_printf("%s axis sensor initialization failed.\r\n", sensors[fusion_sensors[i]].name);             
            }
            else {
              ei_printf("%s axis sensor initialization successful.\r\n", sensors[fusion_sensors[i]].name);
            }
        }
    }

    if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(IMUService);

  IMUService.addCharacteristic(accCharacteristic);
  IMUService.addCharacteristic(gyroCharacteristic);
  IMUService.addCharacteristic(magCharacteristic);
  IMUService.addCharacteristic(accAnglesCharacteristic);
  IMUService.addCharacteristic(neckPosCharacteristic);

  BLE.addService(IMUService);
  BLE.advertise();

  Serial.println("BLE Ready, Advertising Started");


    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
}

/**
* @brief      Get data and run inferencing
*/

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      runModel();
      poll_acc();  // Fix: Remove (void)
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void runModel() {
    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != fusion_ix) {
        ei_printf("ERR: Sensors don't match the sensors required in the model\r\n");
        return;
    }

    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
        int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);
        for(int i = 0; i < fusion_ix; i++) {
            if (sensors[fusion_sensors[i]].status == INIT) {
                sensors[fusion_sensors[i]].poll_sensor();
                sensors[fusion_sensors[i]].status = SAMPLED;
            }
            if (sensors[fusion_sensors[i]].status == SAMPLED) {
                buffer[ix + i] = *sensors[fusion_sensors[i]].value;
                sensors[fusion_sensors[i]].status = INIT;
            }
        }
        int64_t wait_time = next_tick - (int64_t)micros();
        if(wait_time > 0) delayMicroseconds(wait_time);
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("ERR:(%d)\r\n", err);
        return;
    }

    ei_impulse_result_t result = { 0 };
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR:(%d)\r\n", err);
        return;
    }

    float badConfidence = result.classification[1].value; 
prediction_history[prediction_index++] = badConfidence;
if (prediction_index >= HISTORY_SIZE) prediction_index = 0;

samples_collected++;
if (samples_collected > HISTORY_SIZE) samples_collected = HISTORY_SIZE;  // Maintain full history

int badCount = 0;
int goodCount = 0; 
float badConfSum = 0.0; 
float goodConfSum = 0.0; 

for (int i = 0; i < samples_collected; i++) {  // Loop only through collected samples
    Serial.print(String(prediction_history[i]) + "  ");
    if (prediction_history[i] > 0.5) {  
        badCount++;
        badConfSum += (1.0 - prediction_history[i]);
    } else {  
        goodCount++; 
        goodConfSum += prediction_history[i];  
    }
}

bool poorPostureDetected = (badCount >= 6);
neckPosCharacteristic.writeValue(poorPostureDetected);

float avgConfidence = 0.0;
if (poorPostureDetected) {
    avgConfidence = (badCount > 0) ? (badConfSum / badCount) * 100.0 : 0.0; 
    Serial.print("Schlechte Haltung | Durchschnittliche Confidence: ");
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
} else {
    avgConfidence = (goodCount > 0) ? (goodConfSum / goodCount) * 100.0 : 0.0; 
    Serial.print("Gute Haltung | Durchschnittliche Confidence: ");
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);
}

Serial.print(avgConfidence, 1);
Serial.print("%");
Serial.println("");


    

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\r\n", result.anomaly);
#endif
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
#error "Invalid model for current sensor"
#endif


/**
 * @brief Go through sensor list to find matching axis name
 *
 * @param axis_name
 * @return int8_t index in sensor list, -1 if axis name is not found
 */
static int8_t ei_find_axis(char *axis_name)
{
    int ix;
    for(ix = 0; ix < N_SENSORS; ix++) {
        if(strstr(axis_name, sensors[ix].name)) {
            return ix;
        }
    }
    return -1;
}

/**
 * @brief Check if requested input list is valid sensor fusion, create sensor buffer
 *
 * @param[in]  input_list      Axes list to sample (ie. "accX + gyrY + magZ")
 * @retval  false if invalid sensor_list
 */
static bool ei_connect_fusion_list(const char *input_list)
{
    char *buff;
    bool is_fusion = false;

    /* Copy const string in heap mem */
    char *input_string = (char *)ei_malloc(strlen(input_list) + 1);
    if (input_string == NULL) {
        return false;
    }
    memset(input_string, 0, strlen(input_list) + 1);
    strncpy(input_string, input_list, strlen(input_list));

    /* Clear fusion sensor list */
    memset(fusion_sensors, 0, N_SENSORS);
    fusion_ix = 0;

    buff = strtok(input_string, "+");

    while (buff != NULL) { /* Run through buffer */
        int8_t found_axis = 0;

        is_fusion = false;
        found_axis = ei_find_axis(buff);

        if(found_axis >= 0) {
            if(fusion_ix < N_SENSORS) {
                fusion_sensors[fusion_ix++] = found_axis;
                sensors[found_axis].status = NOT_INIT;
            }
            is_fusion = true;
        }

        buff = strtok(NULL, "+ ");
    }

    ei_free(input_string);

    return is_fusion;
}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

bool init_IMU(void) {
  static bool init_status = false;
  if (!init_status) {
    init_status = IMU.begin();
  }
  return init_status;
}


uint8_t poll_acc(void) {
  
    if (IMU.accelerationAvailable()) {

    IMU.readAcceleration(data[0], data[1], data[2]);

    for (int i = 0; i < 3; i++) {
        if (fabs(data[i]) > MAX_ACCEPTED_RANGE) {
            data[i] = ei_get_sign(data[i]) * MAX_ACCEPTED_RANGE;
        }
    }

    float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
    calculateAngles(data[0], data[1], data[2], angleX, angleY, angleZ);
    angleStr = generateString(angleX, angleY, angleZ);
    accAnglesCharacteristic.writeValue(angleStr);

    data[0] *= CONVERT_G_TO_MS2;
    data[1] *= CONVERT_G_TO_MS2;
    data[2] *= CONVERT_G_TO_MS2;

    }

    return 0;
}

uint8_t poll_gyr(void) {
  
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(data[3], data[4], data[5]);
    }
    return 0;
}

uint8_t poll_mag(void) {
  
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(data[6], data[7], data[8]);
    }
    return 0;
}

void calculateAngles(float accX, float accY, float accZ, float &angleX, float &angleY, float &angleZ) {
  angleX = atan2(accY, accZ) * 180.0 / PI;
  angleY = atan2(accX, accZ) * 180.0 / PI;
  angleZ = atan2(accX, accY) * 180.0 / PI;
}

String generateString(float x, float y, float z) {
  return String(x)+"|"+String(y)+"|"+String(z);
}