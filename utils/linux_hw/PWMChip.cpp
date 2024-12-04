#include <filesystem>
#include <string>
#include <vector>
#include "SysfsDevice.hpp"
#include "PWMChip.hpp"

const std::filesystem::path PWMChip::default_sysfs_path("/sys/class/pwm");

PWMChip::PWMChip(const std::filesystem::path& sysfs_path) : SysfsDevice(sysfs_path) {
}

std::string PWMChip::get_name(void) const {
    std::filesystem::path device_path = this->root / "device";

    return std::filesystem::read_symlink(device_path).filename().string();
}

unsigned int PWMChip::get_pwm_count(void) const {
    unsigned int count = 0;

    if (this->has_sys_attr("npwm"))
        count = std::stoul(this->get_sys_attr("npwm"));

    return count;
}

PWM PWMChip::export_pwm(unsigned int number) const {
    std::string pwm_dirname = "pwm" + std::to_string(number);
    std::filesystem::path pwm_path = this->root / pwm_dirname;

    if (std::filesystem::exists(pwm_path)) {
        // seems to be already exported, so do nothing
    } else {
        this->set_sys_attr("export", std::to_string(number));
    }

    return PWM(pwm_path);
}

std::vector<PWMChip> PWMChip::get_system_pwmchips(const std::filesystem::path& sysfs_path) {
    std::vector<PWMChip> rv;

    for (const auto& entry : std::filesystem::directory_iterator(sysfs_path))
        rv.push_back(PWMChip(entry));

    return rv;
}

std::vector<PWMChip> PWMChip::get_system_pwmchips(void) {
    return PWMChip::get_system_pwmchips(PWMChip::default_sysfs_path);
}

PWMChip PWMChip::find_pwmchip(const std::string& name, const std::filesystem::path& sysfs_path) {
    std::vector<PWMChip> system_pwmchips;

    system_pwmchips = PWMChip::get_system_pwmchips(sysfs_path);

    for (const auto& entry : system_pwmchips) {
        if (entry.get_name() == name)
            return entry;
    }

    throw std::runtime_error("A PWMChip with name '" + name + "' was not found.");
}

PWMChip PWMChip::find_pwmchip(const std::string& name) {
    return PWMChip::find_pwmchip(name, PWMChip::default_sysfs_path);
}

PWM PWMChip::find_pwm(const std::string& name, unsigned int number) {
    return find_pwmchip(name).export_pwm(number);
}
