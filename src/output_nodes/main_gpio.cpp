#include "servo_control.hpp"
#include <iostream>

int main() {
	ServoMotor servo;
    if (!servo.setup(PWM_PIN)) {
        return 1;
    }

    int angle;
    std::cout << "Enter angle (0-180): ";
    std::cin >> angle;

    servo.move(10, angle);
    std::cout << "Servo moved to " << angle << " degrees" << std::endl;

    servo.destroy();
    return 0;
}