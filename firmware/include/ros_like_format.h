// ros_like_format.h
// Rewrite formats in this file

// ======= Common Vector3 Definition =======
struct Vector3 {
    float x;
    float y;
    float z;
};

using geometry_msgs__msg__Vector3 = Vector3;

// ======= Twist Struct (for completeness) =======
struct Twist {
    Vector3 linear;
    Vector3 angular;
};

// ======= IMU Struct with Multiple Covariances =======
struct Imu {
    Vector3 linear_acceleration;
    Vector3 angular_velocity;
    Vector3 orientation;

    double linear_acceleration_covariance[9] = {0};  // Default to 0s;
    double angular_velocity_covariance[9] = {0};  // Default to 0s;
    double orientation_covariance[9] = {0};  // Default to 0s;
};

using sensor_msgs__msg__Imu = Imu;

// ======= MagneticField Struct with Covariance =======
struct MagneticField {
    Vector3 magnetic_field;
    double magnetic_field_covariance[9] = {0};  // Default to 0s;
};

using sensor_msgs__msg__MagneticField = MagneticField;