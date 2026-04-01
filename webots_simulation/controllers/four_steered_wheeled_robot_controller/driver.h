#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Pen.hpp>

#include <iostream>
#include <array>
#include <string>
#include <memory>
#include <cmath>


using namespace webots;


/**
 * PlanB four steered wheeled robot parameters:
 * The parameter list for all steering wheels is arranged in the order of "fl, fr, rl, rr"
 */


inline double RAD2DEG(double rad) {return rad * 180.0 / M_PI;}
inline double DEG2RAD(double deg) {return deg * M_PI / 180.0;}

template <typename T>
inline int sign(T x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 1;
}


class FourSteeredWheeledRobot {
public:
    FourSteeredWheeledRobot(Robot *robot);

    ~FourSteeredWheeledRobot();

    void initRobot();

    void updateRobotStates();

    void setWheelCommands(const std::array<double, 4>& target_steer, const std::array<double, 4>& target_drive);

    void stop();
    
    void spin(double angular_vel);

    void crabWalk(double direction, double linear_vel);

    void keyboardInit();
    
    void keyboardControl();

    bool isSteerAlign(const std::array<double, 4>& target_steer);

    void inverseKinematics(double Vx, double Vy, double Ometa, 
        std::array<double, 4>& targer_steer, std::array<double, 4>& targer_drive);  // calculate target_steer_angle and targer_drive_angular_vel by input target Vx,Vy,Ometa

    void ackermanWalk(double Vx, double Vy, double Ometa);

    static inline bool isArrayEqual(const std::array<double, 4>& a, const std::array<double, 4>& b, double tolerance = 0.01);

    static constexpr double CHASSIS_HALF_WIDTH = 0.175;
    static constexpr double WHEEL_RADIUS = 0.05;

private:

    std::array<std::string, 4> STEER_MOTORS = {
        "steer_fl", "steer_fr", "steer_rl", "steer_rr"
    };

    std::array<std::string, 4> STEER_SENSORS = {
        "steer_fl_sensor", "steer_fr_sensor", "steer_rl_sensor", "steer_rr_sensor"
    };

    std::array<std::string, 4> DRIVE_MOTORS = {
        "drive_fl", "drive_fr", "drive_rl", "drive_rr"
    };

    std::array<std::string, 4> DRIVE_SENSORS = {
        "drive_fl_sensor", "drive_fr_sensor", "drive_rl_sensor", "drive_rr_sensor"
    };


    Robot *robot_;
    int timestep_;
    std::array<Motor*, 4> steer_motors_;
    std::array<PositionSensor*, 4> steer_sensors_;
    std::array<Motor*, 4> drive_motors_;
    std::array<PositionSensor*, 4> drive_sensors_;
    InertialUnit* imu_;
    GPS* gps_;
    Keyboard* keyboard_;
    int key_state_[256] = {0};
    Pen* pen_;
    

    struct RobotState {
        double x = 0.0, y = 0.0;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        std::array<double, 4> steer_angles{};
        std::array<double, 4> drive_speeds{};
    } robot_states_;

    enum KeyboardMode {
        STOP,
        SPIN,
        CARBWALK,
        ACKERMANWALK
    };
    KeyboardMode cur_keyboard_mode = STOP;

    

};