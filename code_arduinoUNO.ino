#include <SoftwareSerial.h>
#include <DHT.h>

SoftwareSerial uno(2, 3);
DHT dht(4, DHT11);

#define         MQ_PIN                       (A2)
#define         RL_VALUE                     (22)
#define         RO_CLEAN_AIR_FACTOR          (9.83)
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
#define         CALIBRATION_SAMPLE_TIMES     (50)
#define         CALIBRATION_SAMPLE_INTERVAL  (500)
#define         READ_SAMPLE_INTERVAL         (50)
#define         READ_SAMPLE_TIMES            (5)

#define BUZZER_PIN 8
#define LED_RED 9
#define LED_YELLOW 10

float           LPGCurve[3]  =  {2.3, 0.21, -0.47};
float           COCurve[3]   =  {2.3, 0.72, -0.34};
float           SmokeCurve[3] = {2.3, 0.53, -0.44};
float Ro = 10;

void setup() {
  pinMode(A2, INPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(9600);
  uno.begin(9600);

  dht.begin();
  Serial.println("Calibrating MQ-2...");
  Ro = MQCalibration(MQ_PIN);
  Serial.print("Calibration is done. Ro = ");
  Serial.print(Ro);
  Serial.print(" kohm\n");
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  float lpg_ppm = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG);
  float co_ppm = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO);
  float smoke_ppm = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE);

  if ((lpg_ppm > 1000 && lpg_ppm < 1250) || (co_ppm > 30 && co_ppm < 200)) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    noTone(BUZZER_PIN);            
  } else if (lpg_ppm > 1250 || co_ppm > 200) {
    digitalWrite(LED_RED, HIGH);   
    digitalWrite(LED_YELLOW, LOW); 
    tone(BUZZER_PIN, 1000);        
  } else {
    digitalWrite(LED_RED, LOW);    
    digitalWrite(LED_YELLOW, LOW); 
    noTone(BUZZER_PIN);            
  }

  Serial.println("LPG         : " + String(lpg_ppm) + " ppm");
  Serial.println("CO          : " + String(co_ppm) + " ppm");
  Serial.println("Smoke       : " + String(smoke_ppm) + " ppm");
  Serial.println("Temperature : " + String(temperature) + " °C");
  Serial.println("Humidity    : " + String(humidity) + " %");

  uno.print(lpg_ppm);
  uno.print(",");
  uno.print(co_ppm);
  uno.print(",");
  uno.print(smoke_ppm);
  uno.print(",");
  uno.print(temperature);
  uno.print(",");
  uno.print(humidity);
  uno.println();

  Serial.println("Data sent to NodeMCU.");
  Serial.println();
  delay(5000);
}

float MQResistanceCalculation(int raw_adc) {
  return (((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

float MQCalibration(int mq_pin) {
  int i;
  float val = 0;

  for (i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) {
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBRATION_SAMPLE_TIMES;
  val = val / RO_CLEAN_AIR_FACTOR;

  return val;
}

float MQRead(int mq_pin) {
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if (gas_id == GAS_LPG) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if (gas_id == GAS_CO) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if (gas_id == GAS_SMOKE) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}

int MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10, (((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}