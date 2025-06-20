#include <Arduino.h>
#include <pwm.h>
#include <encoder.h>
#include <motor.h>
#include <pid.h>
#include <kinematics.h>

// 部分物件重寫了資料格式於 ros_like_format.h ，擺脫對MicroROS的依賴
// Encoder object (Original)
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

// Motor object (Original)
Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

// Motor object (Original)
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Kinematics object (Original)
Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

// Timer variables
uint32_t last_ping_time;
uint32_t last_spd_time;
uint32_t last_ctrl_time;
uint32_t last_recv_time;

// Flag for check client online or not
bool nocs_online;

// Variables to store the speed received from NOCS
float control_spd_x = 0.0;
float control_spd_y = 0.0;
float control_spd_rz = 0.0;

// Step for receiving
int step = 0;

// Variables for receiving
char header[2];
int len_i = 0;
char payload[32];  // Max payload = 28 bytes
char chksum = 0;
char footer[2];

void setup()
{
    // 通訊介面（對 ROS2）
    Serial.begin(115200);
    // Set timeout prevent program stuck when serial data problem
    Serial.setTimeout(10);
    //   Serial1.begin(115200);  // 測試指令介面（安裝另一個UART Converter後可以擴充這部分）
    //   Serial1.println("MCU Ready!");
    // Init status LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);

    // Initalize the base's function
    initPwm();
    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();

    // Initalize the timer
    uint32_t now_time = millis();
    last_ping_time = now_time;
    last_spd_time = now_time;
    last_ctrl_time = now_time;
    last_recv_time = now_time;

    // Set the inital client status
    nocs_online = false;
}

// Blink LED function, it will stuck program about 200ms!
// Use carefully!
void blinkLED(int count)
{
    for (int i = 0; i < count; i++)
    {
        digitalWrite(LED_BUILTIN, 1);
        delay(100);
        digitalWrite(LED_BUILTIN, 0);
        delay(100);
    }
}

// XOR checksum calculate function
// Usage:
// length = length of payload to generate checksum
// payload = pointer for the payload's start point
// (If you need start from array's third element: &payload[2])
char chksum_cal(int length, char *payload)
{
    char chksum_calc = 0;
    for (int i = 0; i < length; i++)
    {
        chksum_calc ^= payload[i];
    }
    return chksum_calc;
}

// Handle the request packet(PING and Time request)
void handleRequestPacket(char *payload, int len)
{
    // Define the payload's pattern of these request
    const char PING[4] = {'P', 'i', 'n', 'g'};
    const char TIME_REQUEST[4] = {'T', 'i', 'm', 'e'};
    // Check the payload's pattern
    // PING request
    if (memcmp(payload, PING, 4) == 0)
    {
        // Hint connected signal
        if (!nocs_online)
            blinkLED(3);
        // Raise the client online flag
        nocs_online = true;
        // Reset client timeout
        last_ping_time = millis();
    }
    // TIME request
    else if (memcmp(payload, TIME_REQUEST, 4) == 0)
    {
        // Get current timer's value
        uint32_t now_time = millis();
        // Generate TIME packet to NOCS,
        // NOCS will calculate the difference between ROS wallclock and MCU's timer
        // Empty packet
        char packet[10] = {0};
        // Put header
        packet[0] = 'A';
        packet[1] = 't';
        // Put lenght
        packet[2] = 4;
        // Payload: now_time as 4 bytes (little endian assumed)
        memcpy(&packet[3], &now_time, 4);
        // Checksum
        packet[7] = chksum_cal(4, &packet[3]);
        // Footer
        packet[8] = 'p';
        packet[9] = 'k';
        // Send packet
        Serial.write(packet, 10);
    }
}

// Parse control speed payload
void handleControlSpeed(char *payload, int len)
{
    // Reset timer
    last_ctrl_time = millis();
    // Lit LED to hint user the control speed received
    digitalWrite(LED_BUILTIN, 1);
    // Copy 4 bytes for each float (assuming IEEE 754 float and little-endian)
    // Put to the global vars, and then sendRobotSpeed() will drive motors
    memcpy(&control_spd_x, payload, 4);       // bytes 0-3
    memcpy(&control_spd_y, payload + 4, 4);   // bytes 4-7
    memcpy(&control_spd_rz, payload + 20, 4); // bytes 20-23
}

// Drive motor and send RobotSpeed packet
void sendRobotSpeed()
{
    // These program keep the original logic to drive motor correctly
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        control_spd_x,
        control_spd_y,
        control_spd_rz);
    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();
    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));
    // Calculate the Kinematics of our robot
    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1,
        current_rpm2,
        current_rpm3,
        current_rpm4);

    // Ready to pack RobotSpeed packet
    // Get current timer as timestamp
    uint32_t timestamp = millis();
    // Get the current velocity from upper program
    float vx = current_vel.linear_x;
    float vy = current_vel.linear_y;
    float vrz = current_vel.angular_z;
    // If client online, send the RobotSpeed packet
    if (nocs_online)
    {
        // Empty packet
        char packet[34] = {0};
        // Header
        packet[0] = 'A';
        packet[1] = 'r';
        // Lenght
        packet[2] = 28;
        // Timestamp
        memcpy(&packet[3], &timestamp, 4);
        // Each value are float32 format, so 4 bytes each
        memcpy(&packet[7], &vx, 4);
        memcpy(&packet[11], &vy, 4);
        memcpy(&packet[27], &vrz, 4);
        // Checksum
        packet[31] = chksum_cal(28, &packet[3]);
        // Footer
        packet[32] = 'p';
        packet[33] = 'k';
        // Send packet
        Serial.write(packet, 34);
    }
}

void loop()
{
    // Grab current timer's value
    uint32_t now_time = millis();
    // 檢查距離上次PING是否超過5秒，超過視為斷線，立即剎車並應對
    if (now_time - last_ping_time > 5000)
    {
        // Hint disconnected
        if (nocs_online)
            blinkLED(5);
        // NOCS timeout, don't send speed packet!
        nocs_online = false;
        // Write the speeds to 0
        control_spd_x = 0.0;
        control_spd_y = 0.0;
        control_spd_rz = 0.0;
        // Brake for safety
        motor1_controller.brake();
        motor2_controller.brake();
        motor3_controller.brake();
        motor4_controller.brake();
    }
    // 檢查距離上次接收ControlSpeed是否超過0.5秒，超過了停車以策安全
    if (now_time - last_ctrl_time > 250)
    {
        // No command after 0.5s, auto write zero speed
        // Turn off LED to hint user
        digitalWrite(LED_BUILTIN, 0);
        control_spd_x = 0.0;
        control_spd_y = 0.0;
        control_spd_rz = 0.0;
    }
    // Handle communication messages at about 50 Hz
    if (now_time - last_spd_time > 20)
    {
        // Reset timer
        last_spd_time = now_time;
        // Drive motor and send RobotSpeed packet
        sendRobotSpeed();
    }
    // Receiving part
    switch (step)
    {
        // Wait data and header check
        case 0:
        {
            // 讀取 header(2)
            header[0] = Serial.read();
            // 第一字元不是A就直接跳過，直到A被接收到為止
            if (header[0] != 'A')
                return;
            header[1] = Serial.read();
            // 判斷 header 是否有效
            // Request = As
            // Control Speed = Ac
            bool validHeader = (header[1] == 's' ||
                                header[1] == 'c');
            // Not valid, skip and wait new packet
            if (!validHeader)
                return;
            else
            {
                // Vaild header, goto next step
                step = 1;
            }
            break;
        }
        // Length check
        case 1:
        {
            // 讀取長度 (1 byte)
            len_i = Serial.read();
            // 檢查緩衝區是否有足夠剩餘資料: payload(len_i) + chksum(1) + footer(2)
            if (Serial.available() < len_i + 3)
            {
                // 不夠資料，捨棄該包
                step = 0;
                return;
            }
            else
                step = 2;
            break;
        }
        // Slice data and footer check
        case 2:
        {
            char buffer[len_i + 3];
            // 讀取 payload + chksum + footer
            Serial.readBytes(buffer, len_i + 3);
            // 分離資料
            // Read payload into variable
            memcpy(payload, buffer, len_i);
            // Read checksum into variable
            chksum = buffer[len_i];
            // Read footer into variable
            memcpy(footer, &buffer[len_i + 1], 2);
            // 檢查 footer 是否為 "pk"
            if (footer[0] == 'p' && footer[1] == 'k')
                step = 3;
            else
            {
                step = 0;
                return;
            }
            break;
        }
        // Checksum vaildation
        case 3:
        {
            // Footer correct, try to calculate checksum to vaildate data
            char chksum_calc = chksum_cal(len_i, payload);
            if (chksum_calc == chksum)
            {
                step = 4;
            }
            else
            {
                step = 0;
                return;
            }
            break;
        }
        // Header check and run the handler
        case 4:
        {
            // 封包有效，依 header 呼叫對應函式
            if (header[1] == 's')
            {
                // As = request packet
                handleRequestPacket(payload, len_i);
            }
            else if (header[1] == 'c')
            {
                // Ac = control speed packet
                handleControlSpeed(payload, len_i);
            }
            step = 0;
            break;
        }
        default:
            break;
    }
}
