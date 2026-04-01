#include <webots/Robot.hpp>
#include <driver.h>

using namespace webots;
int main(int argc, char **argv) {
    // create the Robot instance.
    Robot *robot = new Robot();
    
    auto my_4SWR = std::make_unique<FourSteeredWheeledRobot>(robot);

    while (robot->step((int)robot->getBasicTimeStep()) != -1) {
        my_4SWR->updateRobotStates();
        // std::array<double, 4> steer = {0.5, 0.5, 0.5, 0.5};
        // std::array<double, 4> drive = {2.0, 2.0, 2.0, 2.0};
        // my_4SWR->setWheelCommands(steer, drive);
        
        // my_4SWR->spin(0.5);

        // my_4SWR->crabWalk(2, 0.2);

        my_4SWR->keyboardControl();

    };
         
    delete robot;
    return 0;
}
