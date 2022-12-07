#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define BLE_UUID_PADDLE_SERVICE "5101"
#define BLE_UUID_FORWARD_X "2101"
#define BLE_UUID_FORWARD_Y "2102"
#define BLE_UUID_FORWARD_Z "2103"

BLEService paddleService(BLE_UUID_PADDLE_SERVICE);
BLEFloatCharacteristic forwardX(BLE_UUID_FORWARD_X, BLERead | BLENotify);
BLEFloatCharacteristic forwardY(BLE_UUID_FORWARD_Y, BLERead | BLENotify);
BLEFloatCharacteristic forwardZ(BLE_UUID_FORWARD_Z, BLERead | BLENotify);

unsigned long last_micros = 0;
float movement;

// variables for finding average gyro offset
float g_xoff, g_yoff, g_zoff;
int g_av_count = 0;

const float sample_rate = 1.0f / 104.0f;
const float g_cutoff = 0.0001;
const float deg_to_rad = PI / 180.0f;
const float movement_decel = 0.995f;
const float movement_k = 8.0f;
const float max_radians_per_cycle = 6.0f * sample_rate;

const Matrix<3> DOWN = {0, 0, -1};
Matrix<3, 3> IMU_BASIS = {1, 0, 0, 0, 1, 0, 0, 0, 1};
Matrix<3, 3> ROT_MAT_X;
Matrix<3, 3> ROT_MAT_Y;
Matrix<3, 3> ROT_MAT_Z;

void putIdentity(Matrix<3, 3>& out) {
  out = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };
}

void putCrossProductMatrix(Matrix<3, 3>& out, const Matrix<3>& vec) {
  out = {
    0, -vec(2), vec(1),
    vec(2), 0, -vec(0),
    -vec(1), vec(0), 0
  };
}

float dot(const Matrix<3>& a, const Matrix<3>& b) {
  return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
}

void putRotateAroundV(Matrix<3, 3>& out, const Matrix<3>& v, float radians) {
  putRotateAroundVCosSin(out, v, cos(radians), sin(radians));
}

// Rodrigues' rotation formula
// returns a rotation matrix that rotates around v by radians
// v must be a unit vector
void putRotateAroundVCosSin(Matrix<3, 3>& out, const Matrix<3>& v, float cos_ang, float sin_ang) {
  // K will hold the cross product matrix of v
  Matrix<3, 3> K;
  putCrossProductMatrix(K, v);

  // clear output
  out *= 0;

  // second term of equation, sin(theta) * K
  out += K;
  out *= sin_ang;

  // third term of equation, (1 - cos(theta)) * K^2
  K *= K;
  K *= 1 - cos_ang;
  out += K;

  // first term of equation, I
  out(0, 0) += 1;
  out(1, 1) += 1;
  out(2, 2) += 1;
}

// returns a rotation matrix that rotates A towards B. If amount = 1, it rotates A on to B
// a and b must be unit vectors
// 0 <= amount <= 1
void putRotateATowardsB(Matrix<3, 3>& out, const Matrix<3>& a, const Matrix<3>& b, float max_amount) {
  float cos_ang = dot(a, b);
  cos_ang = constrain(cos_ang, -1.0f, 1.0f);
  float ang = acos(cos_ang);

  ang = min(ang, max_amount);
  cos_ang = cos(ang);
  float sin_ang = sin(ang);

  // If parallel, do nothing
  if (sin_ang < 0.001f) {
    putIdentity(out);

    // If anti-parallel, invert
    if (cos_ang < 0) {
      out *= -1;
    }
    return;
  }

  // u will hold vector normal to a and b
  putCrossProductMatrix(out, a);
  Matrix<3> u = out * b;
  u /= Norm(u);  

  putRotateAroundVCosSin(out, u, cos_ang, sin_ang);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.print(F("Accelerometer sample rate = "));
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(F("Hz"));
  Serial.print(F("Gyroscope sample rate = "));
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(F("Hz"));

  // initialize BLE
  if (!BLE.begin()) {
    Serial.println(F("starting Bluetooth® Low Energy failed!"));

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setDeviceName("Andrew Arduino");
  BLE.setLocalName("Andrew Arduino");
  BLE.setAdvertisedService(paddleService);

  // add the characteristic to the service
  paddleService.addCharacteristic(forwardX);
  paddleService.addCharacteristic(forwardY);
  paddleService.addCharacteristic(forwardZ);

  // add service
  BLE.addService(paddleService);

  // set the initial value for the characteristic:
  forwardX.writeValue(1);
  forwardY.writeValue(0);
  forwardZ.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println(F("BLE LED Peripheral"));
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float dt;
    if (last_micros == 0) {
      dt = sample_rate;
    } else {
      unsigned long delta_micros = micros() - last_micros;
      dt = ((float) delta_micros) / 1000000.0f;
    }
    last_micros = micros();

    gx *= dt * deg_to_rad;
    gy *= dt * deg_to_rad;
    gz *= dt * deg_to_rad;

    if (g_av_count < 500) {

      float r = ((float) g_av_count) / ((float) (g_av_count + 1));
      g_av_count++;

      g_xoff = g_xoff * r + gx / ((float) g_av_count);
      g_yoff = g_yoff * r + gy / ((float) g_av_count);
      g_zoff = g_zoff * r + gz / ((float) g_av_count);
      Serial.println("Averaging...");
      Serial.print(g_xoff * 1000);
      Serial.print(F("\t"));
      Serial.print(g_yoff * 1000);
      Serial.print(F("\t"));
      Serial.println(g_zoff * 1000);

      last_micros = micros();

      if (g_av_count == 500) {
        Serial.println(F("Done, advertising."));
      }

    } else {

      BLEDevice central = BLE.central();
      Matrix<3> A = {ax, ay, az};

      gx -= g_xoff;
      gx = (abs(gx) < g_cutoff) ? 0 : gx;
      gy -= g_yoff;
      gy = (abs(gy) < g_cutoff) ? 0 : gy;
      gz -= g_zoff;
      gz = (abs(gz) < g_cutoff) ? 0 : gz;

      putRotateAroundV(ROT_MAT_X, IMU_BASIS.Submatrix<3, 1>(0, 0), gx);
      putRotateAroundV(ROT_MAT_Y, IMU_BASIS.Submatrix<3, 1>(0, 1), gy);
      putRotateAroundV(ROT_MAT_Z, IMU_BASIS.Submatrix<3, 1>(0, 2), gz);
      IMU_BASIS = ROT_MAT_Z * (ROT_MAT_Y * (ROT_MAT_X * IMU_BASIS));

      // stillness = e ^ (-k * movement)
      float mag_A = sqrt(dot(A, A));
      float current_movement = abs(mag_A - 1);
      movement = max(current_movement, movement * movement_decel);
      float max_amount = exp(-movement_k * movement) * max_radians_per_cycle;

      // We would like to align what the IMU thinks is down w/ (0, 0, -1) when we are stationary
      // Down according to the IMU is the accelerometer's direction, converted to world space
      Matrix<3> world_accel = IMU_BASIS * A; // this is the guess for world down
      world_accel *= -1; // since gravity is pulling, accelerometer sees down as positive, so invert
      world_accel /= mag_A; // normalize
      putRotateATowardsB(ROT_MAT_X, world_accel, DOWN, max_amount);
      IMU_BASIS = ROT_MAT_X * IMU_BASIS;

      if (central) {
        Matrix<3> forward = IMU_BASIS.Submatrix<3, 1>(0, 0);
        
        forwardX.writeValue(forward(0));
        forwardY.writeValue(-forward(1));
        forwardZ.writeValue(-forward(2));
      }
    }
  }
}