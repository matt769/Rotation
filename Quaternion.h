// need to distinguish between
// creating quat with actual quat values
// and with vector

//createRotationQuat(angle, vectorx, vectory, vectorz){
//  a = cos(angle/2);
//  b = vectorx*sin(angle/2);
//  c = vectory*sin(angle/2);
//  d = vectorz*sin(angle/2);
//}

class Vector {
  public:
    float x;
    float y;
    float z;
    Vector(float xin, float yin, float zin);
};

Vector::Vector(float xin, float yin, float zin) {
  x = xin;
  y = yin;
  z = zin;
}

class Quaternion {
  public:
    float a;
    float b;
    float c;
    float d;
    Quaternion();
    Quaternion(const Quaternion& quat);
    Quaternion(const Vector& v);
    Quaternion(const Vector& v, float angle);
    Quaternion(const float scalar, const float i, const float j, const float k);
    Quaternion operator+(const Quaternion& q2);
    Quaternion operator-(const Quaternion& q2);
    Quaternion operator-();
    Quaternion operator*(const Quaternion& q2);
    Quaternion operator*(const float r);
    Quaternion operator+(const float r);
    Quaternion operator-(const float r);
    Quaternion operator/(const float scalar);
    float norm();
    Quaternion normalise();
    Quaternion conjugate();
    void print();
    float getRoll();
    float getPitch();
    float getYaw();
};

// rotation order is yaw then pitch then roll
class Euler {
  public:
    float roll;
    float pitch;
    float yaw;
    Euler();
    Euler(float rollIn, float pitchIn, float yawIn);
//    Euler(const& Quaternion q);
    void print();
    void printDegrees();
};

Euler::Euler() {
  roll = 0;
  pitch = 0;
  yaw = 0;
}

Euler::Euler(float rollIn, float pitchIn, float yawIn) {
  roll = rollIn;
  pitch = pitchIn;
  yaw = yawIn;
}

//Euler::Euler(const& Quaternion q) {
//  roll = atan2(2 * (q.a * q.b + q.c * q.d) / (q.a * q.a - q.b * q.b - q.c * q.c + q.d * q.d));
//  pitch = asin(2 * (q.b * q.d - q.a * q.c));
//  yaw = atan2(2 * (q.a * q.d + q.b * q.c) / (q.a * q.a + q.b * q.b - q.c * q.c - q.d * q.d));
//}

void Euler::print(){
  Serial.print(roll, 3); Serial.print('\t');
  Serial.print(pitch, 3); Serial.print('\t');
  Serial.print(yaw, 3); Serial.print('\n');
}

void Euler::printDegrees(){
  Serial.print(roll*180/PI, 3); Serial.print('\t');
  Serial.print(pitch*180/PI, 3); Serial.print('\t');
  Serial.print(yaw*180/PI, 3); Serial.print('\n');
}


Euler convertQuatToEuler(Quaternion q) {
  float roll = atan(2 * (q.a * q.b + q.c * q.d) / (q.a * q.a - q.b * q.b - q.c * q.c + q.d * q.d));
  float pitch = asin(2 * (q.b * q.d - q.a * q.c));
  float yaw = atan(2 * (q.a * q.d + q.b * q.c) / (q.a * q.a + q.b * q.b - q.c * q.c - q.d * q.d));
  return Euler(roll, pitch, yaw);
}

Quaternion::Quaternion() {
  a = 0.0f;
  b = 0.0f;
  c = 0.0f;
  d = 0.0f;
}

Quaternion::Quaternion(const Quaternion& quat) {
  a = quat.a;
  b = quat.b;
  c = quat.c;
  d = quat.d;
}

Quaternion::Quaternion(const Vector& v) {
  a = 0;
  b = v.x;
  c = v.y;
  d = v.z;
}

// Vector must (should?) be normalised
Quaternion::Quaternion(const Vector& v, float angle) {
  a = cos(angle / 2);
  b = v.x * sin(angle / 2);
  c = v.y * sin(angle / 2);
  d = v.z * sin(angle / 2);
}

Quaternion::Quaternion(const float scalar, const float i, const float j, const float k) {
  a = scalar;
  b = i;
  c = j;
  d = k;
}

Quaternion Quaternion::operator+(const Quaternion& q2) {
  return Quaternion(a + q2.a , b + q2.b, c + q2.c, d + q2.d);

}

Quaternion Quaternion::operator-(const Quaternion& q2) {
  return Quaternion(a - q2.a , b - q2.b, c - q2.c, d - q2.d);
}

Quaternion Quaternion::operator-() {
  return Quaternion(-a, -b, -c, -d);
}

Quaternion Quaternion::operator*(const Quaternion& q2) {
  Quaternion result;
  result.a = a * q2.a - b * q2.b - c * q2.c - d * q2.d;
  result.b = a * q2.b + b * q2.a + c * q2.d - d * q2.c;
  result.c = a * q2.c - b * q2.d + c * q2.a + d * q2.b;
  result.d = a * q2.d + b * q2.c - c * q2.b + d * q2.a;
  return result;

}

Quaternion Quaternion::conjugate() {
  return Quaternion(a, -b, -c, -d);
}

Quaternion Quaternion::operator+(const float r) {
  return Quaternion(a + r, b, c, d);
}

Quaternion Quaternion::operator-(const float r) {
  return Quaternion(a - r, b, c, d);
}

Quaternion Quaternion::operator*(const float r) {
  return Quaternion(a * r, b * r, c * r, d * r);
}

Quaternion Quaternion::operator/(const float scalar) {
  float a_new = a / scalar;
  float b_new = b / scalar;
  float c_new = c / scalar;
  float d_new = d / scalar;
  return Quaternion(a_new, b_new, c_new, d_new);
}

float Quaternion::norm() {
  return sqrt(a * a + b * b + c * c + d * d);
}

Quaternion Quaternion::normalise() {
  return Quaternion(*this / norm());
}

//float angleBetween(const Quaternion left, const Quaternion right){
//  return acos(dotproduct(left,right);
//  // not numerically stable?
//}

//void Quaternion::getRollPitchYaw(float roll, float pitch, float yaw){
// TO DO
//  roll =
//  pitch =
//  yaw =
//}

float Quaternion::getRoll() {
  return atan2(2 * (a * b + c * d), a * a - b * b - c * c + d * d);
}

float Quaternion::getPitch() {
  return -asin(2 * (b * d - a * c));
}

float Quaternion::getYaw() {
  return atan2(2 * (a * d + b * c), a * a + b * b - c * c - d * d);
}

void Quaternion::print() {
  Serial.print(a, 3); Serial.print('\t');
  Serial.print(b, 3); Serial.print('\t');
  Serial.print(c, 3); Serial.print('\t');
  Serial.print(d, 3); Serial.print('\n');
}

// NON-MEMBER
float dotProduct(const Quaternion left, const Quaternion right) {
  float dot = left.a * right.a + left.b * right.b + left.c * right.c + left.d * right.d;
  return dot;
}


