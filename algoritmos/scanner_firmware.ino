// =============================================================
// Firmware escaner 3D - ESP32-C3 Super Mini
// Puente IMU (ICM-20948) + control de 2 lasers para ROS 2,
// via USB serial.
//
// Placa Arduino IDE: "ESP32C3 Dev Module"
//                    + USB CDC On Boot: Enabled
// I2C: SDA -> GPIO 6, SCL -> GPIO 7, VIN -> 3.3V, GND -> GND
// Lasers CHT1230: GPIO 4 y GPIO 5 (via modulos MOSFET)
//
// -------------------------------------------------------------
// PROTOCOLO (texto, lineas terminadas en '\n')
//
//   ESP32 -> PC  (stream IMU, a la tasa configurada):
//     I,<t_us>,ax,ay,az,gx,gy,gz,mx,my,mz,temp
//       t_us  = micros() del ESP32
//       a*    = aceleracion en m/s^2
//       g*    = velocidad angular en rad/s
//       m*    = campo magnetico en uT
//       temp  = grados C
//
//   PC -> ESP32  (comandos):
//     L,<n>,<duty>   laser n (1|2) a duty 0..255  (0 = apagado)
//     RATE,<hz>      tasa del stream IMU (1..200)
//     STREAM,0|1     apaga / enciende el stream
//     PING           el ESP32 responde  PONG
//
//   Mensajes de estado: "READY" al arrancar, "ERR,..." si falla.
// =============================================================

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;

// ---- Pines ----
#define I2C_SDA 6
#define I2C_SCL 7

#define LASER1_PIN 4
#define LASER2_PIN 5
#define LASER_PWM_FREQ 5000   // 5 kHz
#define LASER_PWM_RES  8      // 8 bits -> duty 0..255

// ---- Estado del stream ----
uint32_t stream_period_us = 10000;   // 100 Hz por defecto
uint32_t last_imu_us = 0;
bool streaming = true;
bool icm_ok = false;

// ---- Buffer de comandos entrantes ----
char cmd_buf[64];
uint8_t cmd_len = 0;

void setLaser(int n, int duty) {
  duty = constrain(duty, 0, 255);
  if (n == 1) ledcWrite(LASER1_PIN, duty);
  else if (n == 2) ledcWrite(LASER2_PIN, duty);
}

void setup() {
  Serial.begin(115200);
  // Esperar al host, pero sin colgarse para siempre si corre solo.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);   // I2C rapido

  // PWM de los lasers -> ARRANCAN APAGADOS (seguridad)
  ledcAttach(LASER1_PIN, LASER_PWM_FREQ, LASER_PWM_RES);
  ledcAttach(LASER2_PIN, LASER_PWM_FREQ, LASER_PWM_RES);
  ledcWrite(LASER1_PIN, 0);
  ledcWrite(LASER2_PIN, 0);

  // Inicializar el ICM (prueba 0x69 y luego 0x68)
  if (icm.begin_I2C(0x69, &Wire) || icm.begin_I2C(0x68, &Wire)) {
    icm_ok = true;
    icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    // Muestreo interno por encima de la tasa de salida (mejor para fusion)
    icm.setAccelRateDivisor(4);  // ~225 Hz interno
    icm.setGyroRateDivisor(4);   // ~220 Hz interno
  } else {
    Serial.println("ERR,ICM20948 no encontrado");
  }

  Serial.println("READY");
}

void handleCommand(char *line) {
  char *tok = strtok(line, ",");
  if (!tok) return;

  if (strcmp(tok, "L") == 0) {
    char *a = strtok(NULL, ",");
    char *b = strtok(NULL, ",");
    if (a && b) setLaser(atoi(a), atoi(b));
  } else if (strcmp(tok, "RATE") == 0) {
    char *a = strtok(NULL, ",");
    if (a) {
      int hz = constrain(atoi(a), 1, 200);
      stream_period_us = 1000000UL / (uint32_t)hz;
    }
  } else if (strcmp(tok, "STREAM") == 0) {
    char *a = strtok(NULL, ",");
    if (a) streaming = (atoi(a) != 0);
  } else if (strcmp(tok, "PING") == 0) {
    Serial.println("PONG");
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_len > 0) {
        cmd_buf[cmd_len] = '\0';
        handleCommand(cmd_buf);
        cmd_len = 0;
      }
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    }
  }
}

void loop() {
  readSerial();

  uint32_t now = micros();
  if (streaming && icm_ok && (now - last_imu_us >= stream_period_us)) {
    last_imu_us = now;

    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    // Una sola escritura: mas rapido y atomico que muchos print()
    char out[160];
    int n = snprintf(out, sizeof(out),
      "I,%lu,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.2f\n",
      (unsigned long)now,
      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
      gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
      temp.temperature);
    Serial.write((uint8_t *)out, n);
  }
}
