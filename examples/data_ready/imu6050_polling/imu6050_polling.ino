/*****************************************************************************
 * MPU-6050 Data Ready INT Polling Example
 *
 * This example demonstrates how to use the INT line from the MPU6050 board
 * to indicate "data ready". It is set by the 6050 when a dataset is ready,
 * and cleared when any of the data or status registers are read.
 *
 * In this case, there is no need to poll the status registers across the I2C
 * buss. Just read the data when it is ready. It's faster to poll a GPIO line
 * than to request and read the status register byte, and you read one less
 * byte of data when there is data.
 *
 * There's a companion example that uses the data ready INT line as an
 * asynchronous interrupt, but this approach is simpler and good enough for
 * most.
 *
 * Hardware:
 *  * Product ID #3886 - Adafruit MPU-6050 6-DoF Accel and Gyro Sensor,
 *                       STEMMA QT Qwiic
 *  * MCU supporting I2C. Set your GPIO pin below.
 *  * (5) jumper wires - GND, Vcc, SDA, SCL (=STEMMA QT) and INT
 *****************************************************************************/

#include <Adafruit_BusIO_Register.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_MPU6050.h>

/* SET THIS for your MCU!
 * the GPIO pin that the 6050 INT pin is wired to */
#define IMU_DATA_READY    9
#define MCU_ACTIVE       10   // this pin outputs a timing mark for an o'scope

Adafruit_MPU6050 myMpu; // our MPU is an Adafruit_Sensor device
sensors_event_t  gyroEvent; // sensor event data objects
sensors_event_t  acclEvent;
sensors_event_t  tempEvent;

void setup() {
  pinMode(IMU_DATA_READY, INPUT);
  pinMode(MCU_ACTIVE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);             // not really 9600 bps; goes USB speed
  while ((not Serial)             // wait up to 3secs for Serial port setup
         and (millis() < 3e3)) delay(50);
  if (not myMpu.begin()) {        // uses the MPU-6050's default I2C
                                  //   address and sensor ID 0
    diagnose_mcu_fault();         // sort out the problem
  }
  configure_mcu_sampling();       // do all the sample timing prep
  configure_mcu_data_ready_out(); // do all the data ready signal prep
}

void loop() {
  check_the_mcu();  // check for new data
  delay(1);         // do something else for a bit
}

/*****************************************************************************
 * Process any new MCU data.
 *****************************************************************************/
void check_the_mcu (void) {
  if (digitalRead(IMU_DATA_READY) != LOW) return; // data not ready
  digitalWrite(MCU_ACTIVE, HIGH);   // output a signal for the o'scope
  // we've got data!
  /* this call reads 14 contiguous registers so we get
     (7) W16 values: XYZ x 2 + T */
  myMpu.getEvent(&acclEvent, &gyroEvent, &tempEvent);
  plotMcuData();
  digitalWrite(MCU_ACTIVE, LOW);  // drop the o'scope signal
}

/*****************************************************************************
 * Print the MPU/IMU data in the Arduino IDE's Serial Plotter format,
 * with offsets.
 *
 * Adjust offset(s) as needed.
 *****************************************************************************/
void plotMcuData (void) {
  const int offset = 15;
  bool graph_gyro = true; // pick your graph set
  if (graph_gyro) {
    Serial.print("gX=");        // display the value in the legend header
    Serial.print(gyroEvent.gyro.x);
    Serial.print(":");
    Serial.print(gyroEvent.gyro.x+offset); // add a graphing offset

    Serial.print(",gY=");
    Serial.print(gyroEvent.gyro.y);
    Serial.print(":");
    Serial.print(gyroEvent.gyro.y);

    Serial.print(",gZ=");
    Serial.print(gyroEvent.gyro.z);
    Serial.print(":");
    Serial.println(gyroEvent.gyro.z-offset);  // and send it
  } else {
    // plot acceleration data
    Serial.print("aX=");        // display the value in the legend header
    Serial.print(gyroEvent.acceleration.x);
    Serial.print(":");
    Serial.print(gyroEvent.acceleration.x+offset); // add a graphing offset

    Serial.print(",aY=");
    Serial.print(gyroEvent.acceleration.y);
    Serial.print(":");
    Serial.print(gyroEvent.acceleration.y);

    Serial.print(",aZ=");
    Serial.print(gyroEvent.acceleration.z);
    Serial.print(":");
    Serial.println(gyroEvent.acceleration.z-offset);  // and send it
  }
}

/*****************************************************************************
 * Configure the sampling rate of the MCU.
 *
 * Here we use a relatively slow rate to account for running a fusion algoithm
 * and printing plotting data on a modest MCU. We have slow movement so we opt
 * for minimum range/highest resolution. I haven't delved into filter
 * selection yet.
 *
 * Sampling frequency = 8.0kHz / (N + 1)
 *****************************************************************************/
void configure_mcu_sampling (void) {
  myMpu.setSampleRateDivisor(255);           // 31.25 Hz (31.23Hz measured)
  myMpu.setGyroRange(MPU6050_RANGE_250_DEG); // min value=250dps
  // your call here - this may affect sampling rate
  // myMpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
}

/*****************************************************************************
 * Configure the data ready output signal.
 *
 * We're wiring the INT signal to one of our GPIO pins so we can tell when the
 * MCU has fresh data for us. In this example we're not using the INT line in
 * conjuction with an interrupt service routine (ISR) - that's a separate
 * example. We could, but this is simpler and probably good enough for most
 * applications.
 *
 * You can choose which direction the signal goes when there is new data,
 * active high or low. It makes no difference for polling other than your test
 * for a HIGH or LOW value. it could matter for interrupt enabled hardware.
 *
 * Set the pin latching. The default is to strobe for a 50 microsecond pulse.
 * That's fine for latched interrupt pins, but terrible for a polling routine.
 * Set it to stay high until there's a status or data read.
 *
 * Set when the interrupt is cleared. With the interrupt set to latch, we get
 * to choose when the signal is cleared. The deflault is to clear when the
 * status register is read. Here, we select it to clear on any data read since
 * we just read the data registers and never read the status register (at least
 * for data status purposes).
 *
 * As for opimization, you trade all of those I2C status register read
 * exchanges for a quick, local GPIO read. Also, for other IMUs you sometimes
 * have to read an extra register (Status) to clear the interrupt. For this
 * MPU, you don't have to, so that's an I/O reduction right there.
 *****************************************************************************/
void configure_mcu_data_ready_out (void) {
  myMpu.setInterruptPinPolarity(true);// set active low so signal goes low on DR
  myMpu.setInterruptPinLatch(true);   // set signal to stay active until reads.
  myMpu.setClearIntrOnRead(true);     // no need to read status register to
                                      //   reset the data ready INTR signal
  myMpu.setDataReadyInterrupt(true);  // routes data ready signal to board's INT line
}

/* We got here because the mcu.begin() call failed.
   Find out why and report it. */
void diagnose_mcu_fault (void) {
  // replicate some of the mcu.begin() functionality
  // test I2C connectivity
  Adafruit_I2CDevice i2c_dev(MPU6050_I2CADDR_DEFAULT, &Wire);
  if (not i2c_dev.begin()) die("ERROR: I2C address/connectivity problem");
  // test the chip ID
  Adafruit_BusIO_Register who_reg(&i2c_dev, MPU6050_WHO_AM_I, 1);
  uint8_t id;
  if ((id=who_reg.read()) != MPU6050_DEVICE_ID) {
    Serial.print("Chip ID reported:0x");
    Serial.print(id, HEX);
    Serial.print(", expected:0x");
    Serial.println(MPU6050_DEVICE_ID, HEX);
    die("ERROR: mcu chip ID fault");
  }
  // something in _init() died
  die("ERROR: mcu._init() fault");
}

// print an error message and blink

void die (const char* error_string) {
  Serial.println(error_string);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}