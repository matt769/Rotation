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
    Quaternion operator+(const Quaternion& q2) const;
    Quaternion operator-(const Quaternion& q2) const;
    Quaternion operator-() const;
    Quaternion operator*(const Quaternion& q2) const;
    Quaternion operator*(const float scalar) const;
    Quaternion operator+(const float scalar) const;
    Quaternion operator-(const float scalar) const;
    Quaternion operator/(const float scalar) const;
    float dotProduct(const Quaternion q2) const;
    float norm() const;
    Quaternion normalise() const;
    Quaternion conjugate() const;
    float angleBetween(const Quaternion& q2) const;
    Quaternion slerp(const Quaternion& q2, float ratio) const;
    //    void print();
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
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
    Euler(const Euler& e);
};


float dotProduct(const Quaternion& q1, const Quaternion q2);
float angleBetween(const Quaternion& q1, const Quaternion& q2);
Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float ratio);
