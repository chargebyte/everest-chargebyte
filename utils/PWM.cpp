#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include "PWM.hpp"

PWM::PWM(void) {
}

PWM::PWM(const std::filesystem::path& sysfs_path) : SysfsDevice(sysfs_path) {
    // read current state/values from kernel interface
    this->read_current_values();
}

PWM::~PWM(void) {
    // disable PWM if currently enabled to have a "clean" environment
    if (this->enabled)
        this->set_enabled(false);
}

std::chrono::nanoseconds PWM::get_period(void) const {
    return this->period;
}

void PWM::set_period(std::chrono::nanoseconds period) {
    std::chrono::nanoseconds p;

    p = std::max(std::chrono::nanoseconds(0), period);

    // fixup duty cycle property so that the current ratio is kept
    if (this->duty_cycle >= p)
        this->set_duty_cycle(p - std::chrono::nanoseconds(1));

    this->set_sys_attr("period", std::to_string(p.count()));
    this->period = p;
}

std::chrono::nanoseconds PWM::get_duty_cycle(void) const {
    return this->duty_cycle;
}

void PWM::set_duty_cycle(std::chrono::nanoseconds duty_cycle) {
    if (duty_cycle > this->period)
         throw std::invalid_argument("The duty cycle must be less than or equal to the period.");

    this->set_sys_attr("duty_cycle", std::to_string(duty_cycle.count()));
    this->duty_cycle = duty_cycle;
}

float PWM::get_ratio(void) const {
    return static_cast<float>(this->duty_cycle.count()) / static_cast<float>(this->period.count());
}

void PWM::set_ratio(float ratio) {
    ratio = std::fmax(0, std::fmin(1, ratio));

    this->set_duty_cycle(std::chrono::nanoseconds((unsigned long long)(this->period.count() * ratio)));
}

bool PWM::is_enabled(void) const {
    return this->enabled;
}

void PWM::set_enabled(bool enable) {
    this->set_sys_attr("enable", std::to_string(enable));
    this->enabled = enable;
}

bool PWM::is_inverted(void) const {
    return this->inverted;
}

void PWM::set_inverted(bool invert) {
    this->set_sys_attr("polarity", invert ? "inversed" : "normal");
    this->inverted = invert;
}

void PWM::set_frequency_and_ratio(unsigned long long frequency, float ratio) {
    this->set_period(std::chrono::nanoseconds(1000000000 / frequency));
    this->set_ratio(ratio);
}

void PWM::read_current_values(void) {
    std::string val;

    val = this->get_sys_attr("enable");
    this->enabled = std::stoi(val);

    val = this->get_sys_attr("polarity");
    this->inverted = val != "normal";

    val = this->get_sys_attr("period");
    this->period = std::chrono::nanoseconds(std::stoull(val));

    val = this->get_sys_attr("duty_cycle");
    this->duty_cycle = std::chrono::nanoseconds(std::stoull(val));
}
