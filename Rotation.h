// add check if quat needs normalisation (some tolerance around 1)

const bool IN_DEGREES = false;
const bool IN_RADIANS = true;

class Quaternion;
class Euler;

class Vector {
  public:
    float x;
    float y;
    float z;
    Vector(float xIn, float yIn, float zIn);
    Vector(const Quaternion& q);
    Vector rotate(Quaternion& q);
};

class Quaternion {
  public:
    float a;
    float b;
    float c;
    float d;
    Quaternion();
    Quaternion(const Quaternion& quat);
    Quaternion(const Vector& v);
    Quaternion(float angle, const Vector& v);
    Quaternion(const float scalar, const float i, const float j, const float k);
    Quaternion(const Euler& e);
    Quaternion operator+(const Quaternion& q2);
    Quaternion operator-(const Quaternion& q2);
    Quaternion operator-();
    Quaternion operator*(const Quaternion& q2);
    Quaternion operator*(const float r);
    Quaternion operator+(const float r);
    Quaternion operator-(const float r);
    Quaternion operator/(const float scalar);
    float dotProduct(const Quaternion q2);
    float norm();
    Quaternion normalise();
    Quaternion conjugate();
    //    void print();
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
    Euler(float rollIn, float pitchIn, float yawIn, bool inRadians = true);
    Euler(const Quaternion& q);
};

