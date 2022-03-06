#include <SoftwareSerial.h>
#include <Wire.h>
#include <VL53L1X.h>

#define DEBUG_MSGS 0

#define USE_POLOLU 1
#define USE_TFMINI 1

SoftwareSerial softSerial(A14, A15);

#if USE_POLOLU == 1
VL53L1X pololu0;
VL53L1X pololu1;
VL53L1X pololu2;
VL53L1X pololu3;
#endif

#if USE_TFMINI == 1
HardwareSerial *serialPorts[] = {&Serial, &Serial1, &Serial2, &Serial3};

class TFmini {
public:
  TFmini(HardwareSerial *serial_port) {
    serial = serial_port;    
  }

  void init() {
    serial->begin(115200);
  }

  struct TFmini_data{
    uint8_t distanceL;
    uint8_t distanceH;
    int distance;

    uint8_t strengthL;
    uint8_t strengthH;
    int strength;

    uint8_t tempL;
    uint8_t tempH;
    int temp;
  };

  void read() {
    uint8_t checksum = 0;
    uint8_t read_byte = 0;
  
    while (true) {
      checksum = 0;

      do {
        if (serial->available())        
          read_byte = serial->read();
      } while (read_byte != 0x59);
      frame[0] = read_byte;
      checksum += read_byte;
      
      while (!serial->available()) {}
      read_byte = serial->read();
      if (read_byte == 0x59) {
        frame[1] = read_byte;
        checksum += read_byte;
        break;
      }
    }
    
    for (uint8_t i = 2; i < 8; ++i) {
      while (!serial->available()) {}
      read_byte = serial->read();
      frame[i] = read_byte;
      checksum += read_byte;
    }

    while (!serial->available()) {}
    read_byte = serial->read();
    if (checksum == read_byte) {
      data.distanceL = frame[2];
      data.distanceH = frame[3];
      data.distance  = (frame[3] << 8) | frame[2];
      
      data.strengthL = frame[4];
      data.strengthH = frame[5];
      data.strength  = (frame[5] << 8) | frame[4];

      data.tempL     = frame[6];
      data.tempH     = frame[7];
      data.temp      = (frame[7] << 8) | frame[6];
    } else {
#if DEBUG_MSGS == 1
      softSerial.println("Wrong checksum!");
#endif
    }
  }

  struct TFmini_data get_data() {
    return data;
  }

  int get_distance_cm() {
    return data.distance;
  }  
  
private:  
  HardwareSerial *serial;
  struct TFmini_data data;
  uint8_t frame[9] = {0};
};

TFmini tfmini0(&Serial);
TFmini tfmini1(&Serial1);
TFmini tfmini2(&Serial2);
TFmini tfmini3(&Serial3);
#endif

void setup() {
  softSerial.begin(115200);

#if USE_POLOLU == 1
  Wire.begin();
  Wire.setClock(400000); // set I2C clock frequency to 400 kHz

  pinMode(4, OUTPUT); // pololu0 XSHUT
  pinMode(5, OUTPUT); // pololu1 XSHUT
  pinMode(6, OUTPUT); // pololu2 XSHUT
  pinMode(7, OUTPUT); // pololu3 XSHUT

  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  /* pololu0 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(4, INPUT);
  delay(150);
  pololu0.init();
  delay(100);
  pololu0.setAddress((uint8_t)0x2a);

  /* pololu1 */
  // allow the sensor board pololu1 to pull its XSHUT pin up to 2.8V
  pinMode(5, INPUT);
  delay(150);
  pololu1.init();
  delay(100);
  pololu1.setAddress((uint8_t)0x2b);

  /* pololu2 */
  // allow the sensor board pololu2 to pull its XSHUT pin up to 2.8V
  pinMode(6, INPUT);
  delay(150);
  pololu2.init();
  delay(100);
  pololu2.setAddress((uint8_t)0x2c);

  /* pololu3 */
  // allow the sensor board pololu3 to pull its XSHUT pin up to 2.8V
  pinMode(7, INPUT);
  delay(150);
  pololu3.init();
  delay(100);
  pololu3.setAddress((uint8_t)0x2d);

  /* pololu0*/
  pololu0.setDistanceMode(VL53L1X::Long);
  pololu0.setMeasurementTimingBudget(33000);
  pololu0.startContinuous(33);
  
  /* pololu1 */
  pololu1.setDistanceMode(VL53L1X::Long);
  pololu1.setMeasurementTimingBudget(33000);
  pololu1.startContinuous(33);
  
  /* pololu2 */
  pololu2.setDistanceMode(VL53L1X::Long);
  pololu2.setMeasurementTimingBudget(33000);
  pololu2.startContinuous(33);
  
  /* pololu3 */
  pololu3.setDistanceMode(VL53L1X::Long);
  pololu3.setMeasurementTimingBudget(33000);
  pololu3.startContinuous(33);
#endif

#if USE_TFMINI == 1
  tfmini0.init();
  tfmini1.init();
  tfmini2.init();
  tfmini3.init();
#endif
}

void loop() {
  // auto start = millis();

#if USE_POLOLU == 1
  pololu0.read();
  pololu1.read();
  pololu2.read();
  pololu3.read();
#endif

#if USE_TFMINI == 1
  tfmini0.read();
  tfmini1.read();
  tfmini2.read();
  tfmini3.read();
#endif

#if USE_POLOLU == 1
  softSerial.print(pololu0.ranging_data.range_mm);
  softSerial.print(',');
  softSerial.print(pololu1.ranging_data.range_mm);
  softSerial.print(',');
  softSerial.print(pololu2.ranging_data.range_mm);
  softSerial.print(',');
  softSerial.print(pololu3.ranging_data.range_mm);
  softSerial.print(',');
#else
  softSerial.print("0,0,0,0,");
#endif

#if USE_TFMINI == 1
  softSerial.print(tfmini0.get_distance_cm());
  softSerial.print(',');
  softSerial.print(tfmini1.get_distance_cm());
  softSerial.print(',');
  softSerial.print(tfmini2.get_distance_cm());
  softSerial.print(',');
  softSerial.print(tfmini3.get_distance_cm());
#else
  softSerial.print("0,0,0,0");
#endif

  softSerial.print("\r\n");

  // auto freq = 1000 / (millis() - start);
  // softSerial.print(freq);
  // softSerial.println(" Hz");
}
