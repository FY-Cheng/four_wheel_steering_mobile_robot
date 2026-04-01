#include <driver.h>
#include <cstring>


FourSteeredWheeledRobot::FourSteeredWheeledRobot(Robot *robot) : robot_(robot) {
    initRobot();
}


FourSteeredWheeledRobot::~FourSteeredWheeledRobot() {}


void FourSteeredWheeledRobot::initRobot() {
    timestep_ = (int)robot_->getBasicTimeStep();

    // init four steer and drive motors and position_sensors
    for (size_t i = 0; i < 4; ++i) {
        steer_motors_[i] = robot_->getMotor(STEER_MOTORS[i]);
        steer_motors_[i]->setPosition(INFINITY);
        
        steer_sensors_[i] = robot_->getPositionSensor(STEER_SENSORS[i]);
        steer_sensors_[i]->enable(timestep_);

        drive_motors_[i] = robot_->getMotor(DRIVE_MOTORS[i]);
        drive_motors_[i]->setPosition(INFINITY);
        drive_motors_[i]->setVelocity(0.0);

        drive_sensors_[i] = robot_->getPositionSensor(DRIVE_SENSORS[i]);
        drive_sensors_[i]->enable(timestep_);
    }

    imu_ = robot_->getInertialUnit("inertial unit");
    imu_->enable(timestep_);

    gps_ = robot_->getGPS("gps");
    gps_->enable(timestep_);

    keyboard_ = robot_->getKeyboard();
    keyboard_->enable(timestep_);

    pen_ = robot_->getPen("pen");

    stop();
    keyboardInit();
    pen_->write(true);

    std::cout<<"initialize robot done!";
}


void FourSteeredWheeledRobot::updateRobotStates() {
    // update GPS
    const auto value = gps_->getValues();
    robot_states_.x = value[0];
    robot_states_.y = value[1];

    // update imu
    const auto rpy = imu_->getRollPitchYaw();
    robot_states_.roll = rpy[2];
    robot_states_.pitch = rpy[2];
    robot_states_.yaw = rpy[2];
    
    //update steer and drive
    for (size_t i = 0; i < 4; ++i) {
        robot_states_.steer_angles[i] = steer_sensors_[i]->getValue();
        robot_states_.drive_speeds[i] = drive_motors_[i]->getVelocity();
    }
}


void FourSteeredWheeledRobot::setWheelCommands(const std::array<double, 4>& target_steer, const std::array<double, 4>& target_drive) {
    for (size_t i = 0; i < 4; ++i) {
        steer_motors_[i]->setPosition(target_steer[i]);
        drive_motors_[i]->setVelocity(target_drive[i]);
    }
}


void FourSteeredWheeledRobot::stop() {
    std::array<double, 4> target_drive;
    target_drive.fill(0);
    setWheelCommands(robot_states_.steer_angles, target_drive);
}


void FourSteeredWheeledRobot::spin(double body_angular_vel) {
    // constexpr std::array<double, 4> steer_angles = {
    //     -M_PI / 4.0,
    //     M_PI / 4.0,
    //     M_PI / 4.0,
    //     -M_PI / 4.0
    // };
    
    // double R = std::sqrt(2) * CHASSIS_HALF_WIDTH;
    // double wheel_linear_vel = body_angular_vel * R;
    // double wheel_angular_vel = wheel_linear_vel / WHEEL_RADIUS;

    // std::array<double, 4> drive_speeds = {
    //     wheel_angular_vel, -wheel_angular_vel,
    //     wheel_angular_vel, -wheel_angular_vel
    // };

    // setWheelCommands(steer_angles, drive_speeds);


    ackermanWalk(0, 0, body_angular_vel);
}


bool FourSteeredWheeledRobot::isSteerAlign(const std::array<double, 4>& target_steer) {
    for (int i = 0; i < 4; ++i) {
        if (fabs(target_steer[i] - robot_states_.steer_angles[i]) > DEG2RAD(5))
            return false;
    }
    return true;
}


void FourSteeredWheeledRobot::crabWalk(double direction, double linear_vel) {
    std::array<double, 4> steer_angles, drive_speeds;
    steer_angles.fill(direction);

    if (!isSteerAlign(steer_angles)) {
        drive_speeds.fill(0);
        setWheelCommands(steer_angles, drive_speeds);
        return;
    }

    const double wheel_vel = linear_vel / WHEEL_RADIUS;
    drive_speeds.fill(wheel_vel);
    setWheelCommands(steer_angles, drive_speeds);
}


void FourSteeredWheeledRobot::keyboardInit() {
    std::cout << "\n=========================================" << '\n';
    std::cout << "四舵轮键盘控制启动" << '\n';
    std::cout << "=========================================" << '\n';
    std::cout << "WASD         蟹行模式 (8方向移动)" << '\n';
    std::cout << "JL           原地旋转 (左/右转)" << '\n';
    std::cout << "IK + JL      阿克曼模式(弧形移动)" << '\n';
    std::cout << "空格          急停" << '\n';
    std::cout << "=========================================\n" << '\n';
}


void FourSteeredWheeledRobot::keyboardControl() {
    constexpr double VEL_LINEAR  = 1.0;
    constexpr double VEL_ANGULAR = 2.0;
    
    memset(key_state_, 0, sizeof(key_state_));
    int key;
    bool any_key = false;
    while ((key = keyboard_->getKey()) != -1) {
        if (key >= 0 && key < 256) {
            key_state_[key] = 1;
        }
        any_key = true;
    }

    if (!any_key) {
        stop();
        return;
    }

    bool w = key_state_['W'] || key_state_['w'];
    bool a = key_state_['A'] || key_state_['a'];
    bool s = key_state_['S'] || key_state_['s'];
    bool d = key_state_['D'] || key_state_['d'];
    bool i = key_state_['I'] || key_state_['i'];
    bool k = key_state_['K'] || key_state_['k'];
    bool j = key_state_['J'] || key_state_['j'];
    bool l = key_state_['L'] || key_state_['l'];


    // ------------------------------
    // 优先级1：WASD 蟹行8方向
    // ------------------------------
    if (w || a || s || d) {
        double dir = 0.0;

        if (w && !a && !d)      dir = 0.0;
        else if (w && a)        dir = M_PI_4;
        else if (a && !w && !s) dir = M_PI_2;
        else if (a && s)        dir = 3 * M_PI_4;
        else if (s && !a && !d) dir = M_PI;
        else if (s && d)        dir = -3 * M_PI_4;
        else if (d && !w && !s) dir = -M_PI_2;
        else if (w && d)        dir = -M_PI_4;

        crabWalk(dir, VEL_LINEAR);
        return;
    }

    // ------------------------------
    // 优先级2：阿克曼模式 (IK+JL 组合)
    // ------------------------------
    if (i || k) {
        const double Vx = i ? VEL_LINEAR : -VEL_LINEAR;
        double Omega = 0.0;

        if (j) Omega = 2;   // 左转向
        if (l) Omega = -2;  // 右转向

        // 阿克曼弧形移动
        ackermanWalk(Vx / 2, 0, Omega);
        return;
    }

    // ------------------------------
    // 优先级3：原地旋转 (JL)
    // ------------------------------
    if (j && !l) {
        spin(VEL_ANGULAR);
        return;
    }
    if (l && !j) {
        spin(-VEL_ANGULAR);
        return;
    }
}


inline bool FourSteeredWheeledRobot::isArrayEqual(const std::array<double, 4>& a, const std::array<double, 4>& b, double tolerance) {
    for (int i = 0; i < 4; ++i) {
        if (std::fabs(a[i] - b[i]) > tolerance) {
            return false;
        }
    }
    return true;
}


void FourSteeredWheeledRobot::inverseKinematics(double Vx, double Vy, double Omega, 
    std::array<double, 4>& target_steer, std::array<double, 4>& target_drive) {
    double h = CHASSIS_HALF_WIDTH;
    double hw = h * Omega;

    target_steer = {
        std::atan2((Vy + hw), (Vx - hw)),
        std::atan2((Vy + hw), (Vx + hw)),
        std::atan2((Vy - hw), (Vx - hw)),
        std::atan2((Vy - hw), (Vx + hw))
    };

    std::cout<<std::endl<<target_steer[0];

    target_drive = {
        std::sqrt(std::pow(Vx - hw, 2) + std::pow(hw + Vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(Vx + hw, 2) + std::pow(hw + Vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(Vx - hw, 2) + std::pow(hw - Vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(Vx + hw, 2) + std::pow(hw - Vy, 2)) / WHEEL_RADIUS
    };
}


void FourSteeredWheeledRobot::ackermanWalk(double Vx, double Vy, double Omega) {
    std::array<double, 4> target_steer, target_drive;
    inverseKinematics(Vx, Vy, Omega, target_steer, target_drive);
    setWheelCommands(target_steer, target_drive);
}