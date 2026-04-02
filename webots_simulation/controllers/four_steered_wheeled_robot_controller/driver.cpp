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
        steer_motors_[i]->setPosition(0.0);
        
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
    
    // update steer and drive
    for (size_t i = 0; i < 4; ++i) {
        robot_states_.steer_angles[i] = steer_sensors_[i]->getValue();
        robot_states_.drive_speeds[i] = drive_motors_[i]->getVelocity();
    }
}


bool FourSteeredWheeledRobot::isSteerAlign(const std::array<double, 4>& target_steer) {
    for (int i = 0; i < 4; ++i) {
        if (fabs(target_steer[i] - robot_states_.steer_angles[i]) > DEG2RAD(5))
            return false;
    }
    return true;
}


void FourSteeredWheeledRobot::setWheelCommands(std::array<double, 4> target_steer, std::array<double, 4> target_drive) {
    // 这里将舵轮角度限制到[-pi/2, pi/2]范围内，防止舵轮需要大角度旋转
    for (size_t i = 0; i < 4; ++i) {
        if (target_steer[i] > M_PI / 2.0) {
            target_steer[i] -= M_PI;
            target_drive[i] *= -1.0;
        } else if (target_steer[i] <= -M_PI / 2.0) {
            target_steer[i] += M_PI;
            target_drive[i] *= -1.0;
        }
    }

    bool steering_align_done = isSteerAlign(target_steer);

    for (size_t i = 0; i < 4; ++i) {
        steer_motors_[i]->setPosition(target_steer[i]);
        drive_motors_[i]->setVelocity(steering_align_done ? target_drive[i] : 0.0);
    }
}


void FourSteeredWheeledRobot::stop() {
    std::array<double, 4> target_drive;
    target_drive.fill(0);
    setWheelCommands(robot_states_.steer_angles, target_drive);
}


void FourSteeredWheeledRobot::spin(double body_angular_vel) {
    dualAckermanWalk(0, 0, body_angular_vel);
}


void FourSteeredWheeledRobot::crabWalk(double direction, double linear_vel) {
    std::array<double, 4> steer_angles, drive_speeds;
    steer_angles.fill(direction);
    drive_speeds.fill(linear_vel / WHEEL_RADIUS);
    setWheelCommands(steer_angles, drive_speeds);
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

    target_drive = {
        std::sqrt(std::pow(Vx - hw, 2) + std::pow(hw + Vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(Vx + hw, 2) + std::pow(hw + Vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(Vx - hw, 2) + std::pow(hw - Vy, 2)) / WHEEL_RADIUS,
        std::sqrt(std::pow(Vx + hw, 2) + std::pow(hw - Vy, 2)) / WHEEL_RADIUS
    };
}


void FourSteeredWheeledRobot::dualAckermanWalk(double Vx, double Vy, double Omega) {
    std::array<double, 4> target_steer, target_drive;
    inverseKinematics(Vx, Vy, Omega, target_steer, target_drive);
    setWheelCommands(target_steer, target_drive);
}


void FourSteeredWheeledRobot::ackermanWalk(double V, double Omega) {
    std::array<double, 4> target_steer, target_drive;
    double Vy = CHASSIS_HALF_WIDTH * Omega;
    double Vx = sign(V) * std::sqrt(std::pow(V, 2) - std::pow(Vy, 2));
    inverseKinematics(Vx, Vy, Omega, target_steer, target_drive);
    setWheelCommands(target_steer, target_drive);
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
        // dualAckermanWalk(Vx / 2, 0, Omega);
        ackermanWalk(Vx, Omega);
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

