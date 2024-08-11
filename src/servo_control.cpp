
#include <GPIO_Control.hpp>

bool ServoMotor::setup() {
    chip = gpiod_chip_open_by_name(CHIP_NAME);
    if (!chip) {
        std::cerr << "Failed to open GPIO chip" << std::endl;
        return false;
    }

    line = gpiod_chip_get_line(chip, PWM_PIN);
    if (!line) {
        std::cerr << "Failed to get GPIO line" << std::endl;
        gpiod_chip_close(chip);
        return false;
    }

    if (gpiod_line_request_output(line, "servo_control", 0) < 0) {
        std::cerr << "Failed to request line as output" << std::endl;
        gpiod_chip_close(chip);
        return false;
    }

    return true;
}

void ServoMotor::move(int angle) {
    // Ensure the angle is within 0 to 180 degrees
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Convert angle to PWM duty cycle
    int duty_cycle = angle_to_duty_cycle(angle);

    // Send the PWM signal
    for (int i = 0; i < 50; ++i) {
        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(std::chrono::microseconds(duty_cycle));
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(20000 - duty_cycle));
    }
}

void ServoMotor::destroy() {
    if (line) {
        gpiod_line_release(line);
        line = nullptr;
    }
    if (chip) {
        gpiod_chip_close(chip);
        chip = nullptr;
    }
}

int ServoMotor::angle_to_duty_cycle(int angle) {
    // Map angle (0-180) to duty cycle (1000-2000 microseconds)
    return 1000 + ((angle * 1000) / 180);
}
