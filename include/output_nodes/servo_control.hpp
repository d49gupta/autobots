#include <gpiod.h>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std;

#define CHIP_NAME "gpiochip4"
#define PWM_PIN 27 // The GPIO pin number

class ServoMotor {
public:
    ServoMotor() : chip(nullptr), line(nullptr) {}
    bool setup(int pwm_pin);
    void move(int time, int angle);
    void setHigh();
    void setLow();
    void destroy();
private:
    gpiod_chip *chip;
    gpiod_line *line;
    int angle_to_duty_cycle(int angle);
};
