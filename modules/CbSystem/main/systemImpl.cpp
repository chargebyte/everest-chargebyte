// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "systemImpl.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

#include <unistd.h>

#include <utils/date.hpp>

#include <boost/process.hpp>

namespace module {
namespace main {

namespace fs = std::filesystem;

const std::string CONSTANTS = "constants.env";
const std::string DIAGNOSTICS_UPLOADER = "diagnostics_uploader.sh";
const std::string FIRMWARE_UPDATER = "firmware_updater.sh";
const std::string SIGNED_FIRMWARE_DOWNLOADER = "signed_firmware_downloader.sh";
const std::string SIGNED_FIRMWARE_INSTALLER = "signed_firmware_installer.sh";
const fs::path MARKER_FILE_PATH = "/var/lib/everest/.ocpp_fw_installed";

static const char* const diagnostic_files[] = {"/tmp/commands.log", /* must always be the first entry */
                                               "/etc/everest/config.yaml",
                                               "/etc/everest/ocpp-config.json",
                                               "/usr/share/secc/VERSION",
                                               "/etc/os-release",
                                               "/tmp/everest_ocpp_logs",
                                               "/var/lib/everest/ocpp16",
                                               nullptr};

static const char* const diagnostic_commands[] = {
    "df",
    "dmesg",
    "ifconfig",
    "ps ax",
    "lsblk",
    "lsusb",
    "cat /proc/meminfo",
    "uptime",
    "timedatectl status",
    "cat /sys/kernel/debug/gpio",
    "cat /sys/kernel/debug/pwm",
    "cat /sys/kernel/debug/eth1/info",
    "cat /sys/kernel/debug/eth2/info",
    "ethtool eth0",
    "ethtool -S eth1",
    "plctool -r -i eth1",
    "ethtool -S eth2",
    "plctool -r -i eth2 FF:FF:FF:FF:FF:FF",
    "plcstat -t -i eth2",
    "[ \"$(plcID -i eth2 -M)\" = '50:D3:E4:93:3F:85:5B:70:40:78:4D:F8:15:AA:8D:B7' ] && echo 'Warning: HomePlugAV "
    "default key set on mains interface!'",
    "fw_printenv",
    "fw_printenv -c /etc/baptism-data.config serial",
    "md5sum /boot/*.dtb",
    "ls -Rl1 /etc/everest/certs/",
    "ls -Rl1 /usr/libexec/everest/modules/",
    "rauc status --detailed",
    "cat /proc/tty/driver/IMX-uart",
    "grep \"\" /sys/bus/iio/devices/iio\\:device0/in_voltage?_raw",
    "mosquitto_sub -v -t '#' -W 1",
    "i2cdetect -y 3",
    "mmc extcsd read /dev/mmcblk0",
    "journalctl --no-pager -u everest",
    nullptr};

// FIXME (aw): this function needs to be refactored into some kind of utility library
fs::path create_temp_file(const fs::path& dir, const std::string& prefix) {
    const std::string fn_template = (dir / prefix).string() + "XXXXXX" + std::string(1, '\0');
    std::vector<char> fn_template_buffer {fn_template.begin(), fn_template.end()};

    // mkstemp needs to have at least 6 XXXXXX at the end and it will replace these
    // with a valid file name
    auto fd = mkstemp(fn_template_buffer.data());

    if (fd == -1) {
        EVLOG_AND_THROW(Everest::EverestBaseRuntimeError("Failed to create temporary file at: " + fn_template));
    }

    // close the file descriptor
    close(fd);

    return fn_template_buffer.data();
}

void systemImpl::init() {
    this->scripts_path = mod->info.paths.libexec;
    this->log_upload_running = false;
    this->firmware_download_running = false;
    this->firmware_installation_running = false;
    this->standard_firmware_update_running = false;

    if (fs::exists(MARKER_FILE_PATH)) {
        this->boot_reason = types::system::BootReason::FirmwareUpdate;
    }
}

enum class PartitionType {
    Active,
    Inactive
};

static std::string get_partition(PartitionType part_type) {
    std::string rauc_status_search;
    switch (part_type) {
    case PartitionType::Active:
        rauc_status_search = "booted";
        break;
    case PartitionType::Inactive:
        rauc_status_search = "inactive";
        break;
    }

    const std::string shell_cmd = R"(rauc status | sed 's/\x1b\[[0-9;]*m//g' | awk '/rootfs\./ && /)" +
                                  rauc_status_search + R"(/ {gsub(/\[|\]/, "", $2); print $2}')";
    boost::process::ipstream stream;
    boost::process::child cmd(boost::process::search_path("sh"), std::vector<std::string> {"-c", shell_cmd},
                              boost::process::std_out > stream);

    std::string output;
    // this is generic multi-line handling
    // while (cmd.running() && std::getline(stream, line) && !line.empty()) {
    //     output += line + "\n";
    // }
    // this is single-line handling
    if (cmd.running()) {
        std::getline(stream, output);
    }

    cmd.wait();

    return output;
}

void systemImpl::ready() {

    // In case the firmware-update marker file exists, we need to check if the firmware update was successful
    // and publish the status.
    if (fs::exists(MARKER_FILE_PATH)) {
        std::ifstream marker_file(MARKER_FILE_PATH);
        std::string partition;
        std::string request_id;

        std::string line1, line2;
        if (std::getline(marker_file, line1) && std::getline(marker_file, line2)) {
            partition = line1;
            request_id = line2;
        }

        if (partition == get_partition(PartitionType::Active)) {
            this->publish_firmware_update_status(
                {types::system::FirmwareUpdateStatusEnum::Installed, std::stoi(request_id)});
            EVLOG_info << "Firmware update was successful";
        } else {
            this->publish_firmware_update_status(
                {types::system::FirmwareUpdateStatusEnum::InstallationFailed, std::stoi(request_id)});
            EVLOG_error << "Firmware update was not successful: expected partition: " << partition
                        << " actual partition: " << get_partition(PartitionType::Active);
        }
        fs::remove(MARKER_FILE_PATH);
    }
}

void systemImpl::standard_firmware_update(const types::system::FirmwareUpdateRequest& firmware_update_request) {

    this->standard_firmware_update_running = true;
    EVLOG_info << "Starting firmware update";
    // create temporary file
    const auto date_time = Everest::Date::to_rfc3339(date::utc_clock::now());

    // Note: we don't download to /tmp as in vanilla everest since our download sizes
    // might be larger than available RAM;
    // we use a fixed path for now so that we can ensure a cleanup...
    const auto firmware_file_path = fs::path("/srv/firmware-update.image");
    const auto constants = this->scripts_path / CONSTANTS;

    // ...here by deleting a possibly existing, older file
    if (fs::exists(firmware_file_path)) {
        fs::remove(firmware_file_path);
    }

    this->update_firmware_thread = std::thread([this, firmware_update_request, firmware_file_path, constants]() {
        const auto firmware_updater = this->scripts_path / FIRMWARE_UPDATER;

        const std::vector<std::string> args = {constants.string(), firmware_update_request.location,
                                               firmware_file_path.string()};
        int32_t retries = 0;
        const auto total_retries = firmware_update_request.retries.value_or(this->mod->config.DefaultRetries);
        const auto retry_interval =
            firmware_update_request.retry_interval_s.value_or(this->mod->config.DefaultRetryInterval);

        auto firmware_status_enum = types::system::FirmwareUpdateStatusEnum::DownloadFailed;
        types::system::FirmwareUpdateStatus firmware_status;
        firmware_status.request_id = firmware_update_request.request_id;
        firmware_status.firmware_update_status = firmware_status_enum;

        const std::string partition = get_partition(PartitionType::Inactive);
        // write a flag/marker file which gets copied to the new installation, which indicates what partition we
        // expect to be on after update
        {
            std::ofstream marker_file(MARKER_FILE_PATH, std::ios::trunc);
            marker_file << partition << "\n" << firmware_status.request_id << std::endl;
        }

        while (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::DownloadFailed &&
               retries <= total_retries) {
            boost::process::ipstream stream;
            boost::process::child cmd(firmware_updater.string(), boost::process::args(args),
                                      boost::process::std_out > stream);
            std::string temp;
            retries += 1;
            while (std::getline(stream, temp)) {
                EVLOG_info << "Firmware update status: " << temp;
                firmware_status.firmware_update_status = types::system::string_to_firmware_update_status_enum(temp);
                this->publish_firmware_update_status(firmware_status);
            }
            if (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::DownloadFailed &&
                retries <= total_retries) {
                std::this_thread::sleep_for(std::chrono::seconds(retry_interval));
            }
            cmd.wait();
        }
        this->standard_firmware_update_running = false;
        if (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::InstallRebooting) {
            EVLOG_warning << "Firmware update finished successfully. Initiating reboot in 5 seconds";
            std::this_thread::sleep_for(std::chrono::seconds(5));
            EVLOG_info << "Rebooting...";
            boost::process::system("reboot");
        } else if ((firmware_status.firmware_update_status ==
                    types::system::FirmwareUpdateStatusEnum::DownloadFailed) ||
                   (firmware_status.firmware_update_status ==
                    types::system::FirmwareUpdateStatusEnum::InstallationFailed)) {
            fs::remove(MARKER_FILE_PATH);
        }
    });
    this->update_firmware_thread.detach();
}

types::system::UpdateFirmwareResponse
systemImpl::handle_standard_firmware_update(const types::system::FirmwareUpdateRequest& firmware_update_request) {

    if (!this->standard_firmware_update_running) {
        if (firmware_update_request.retrieve_timestamp.has_value() &&
            Everest::Date::from_rfc3339(firmware_update_request.retrieve_timestamp.value()) > date::utc_clock::now()) {
            const auto retrieve_timestamp =
                Everest::Date::from_rfc3339(firmware_update_request.retrieve_timestamp.value());
            this->standard_update_firmware_timer.at(
                [this, retrieve_timestamp, firmware_update_request]() {
                    this->standard_firmware_update(firmware_update_request);
                },
                retrieve_timestamp);
            EVLOG_info << "Download for firmware scheduled for: " << Everest::Date::to_rfc3339(retrieve_timestamp);
        } else {
            // start download immediately
            this->update_firmware_thread = std::thread(
                [this, firmware_update_request]() { this->standard_firmware_update(firmware_update_request); });
            this->update_firmware_thread.detach();
        }
        return types::system::UpdateFirmwareResponse::Accepted;
    } else {
        EVLOG_info << "Not starting firmware update because firmware update process already running";
        return types::system::UpdateFirmwareResponse::Rejected;
    }
}

types::system::UpdateFirmwareResponse
systemImpl::handle_signed_fimware_update(const types::system::FirmwareUpdateRequest& firmware_update_request) {

    if (!firmware_update_request.signing_certificate.has_value()) {
        EVLOG_warning << "Signing certificate is missing in FirmwareUpdateRequest";
        return types::system::UpdateFirmwareResponse::Rejected;
    }
    if (!firmware_update_request.signature.has_value()) {
        EVLOG_warning << "Signature is missing in FirmwareUpdateRequest";
        return types::system::UpdateFirmwareResponse::Rejected;
    }

    EVLOG_info << "Executing signed firmware update download callback";

    if (firmware_update_request.retrieve_timestamp.has_value() &&
        Everest::Date::from_rfc3339(firmware_update_request.retrieve_timestamp.value()) > date::utc_clock::now()) {
        const auto retrieve_timestamp = Everest::Date::from_rfc3339(firmware_update_request.retrieve_timestamp.value());
        this->signed_firmware_update_download_timer.at(
            [this, retrieve_timestamp, firmware_update_request]() {
                this->download_signed_firmware(firmware_update_request);
            },
            retrieve_timestamp);
        EVLOG_info << "Download for firmware scheduled for: " << Everest::Date::to_rfc3339(retrieve_timestamp);
        types::system::FirmwareUpdateStatus firmware_update_status;
        firmware_update_status.request_id = firmware_update_request.request_id;
        firmware_update_status.firmware_update_status = types::system::FirmwareUpdateStatusEnum::DownloadScheduled;
        this->publish_firmware_update_status(firmware_update_status);
    } else {
        // start download immediately
        this->update_firmware_thread =
            std::thread([this, firmware_update_request]() { this->download_signed_firmware(firmware_update_request); });
        this->update_firmware_thread.detach();
    }

    if (this->firmware_download_running) {
        return types::system::UpdateFirmwareResponse::AcceptedCancelled;
    } else if (this->firmware_installation_running) {
        return types::system::UpdateFirmwareResponse::Rejected;
    } else {
        return types::system::UpdateFirmwareResponse::Accepted;
    }
}

void systemImpl::download_signed_firmware(const types::system::FirmwareUpdateRequest& firmware_update_request) {

    if (!firmware_update_request.signing_certificate.has_value()) {
        EVLOG_warning << "Signing certificate is missing in FirmwareUpdateRequest";
        this->publish_firmware_update_status(
            {types::system::FirmwareUpdateStatusEnum::DownloadFailed, firmware_update_request.request_id});
        return;
    }
    if (!firmware_update_request.signature.has_value()) {
        EVLOG_warning << "Signature is missing in FirmwareUpdateRequest";
        this->publish_firmware_update_status(
            {types::system::FirmwareUpdateStatusEnum::DownloadFailed, firmware_update_request.request_id});
        return;
    }

    if (this->firmware_download_running) {
        EVLOG_info
            << "Received Firmware update request and firmware update already running - cancelling firmware update";
        this->interrupt_firmware_download.exchange(true);
        EVLOG_info << "Waiting for other firmware download to finish...";
        std::unique_lock<std::mutex> lk(this->firmware_update_mutex);
        this->firmware_update_cv.wait(lk, [this]() { return !this->firmware_download_running; });
        EVLOG_info << "Previous Firmware download finished!";
    }

    std::lock_guard<std::mutex> lg(this->firmware_update_mutex);
    EVLOG_info << "Starting Firmware update";
    this->interrupt_firmware_download.exchange(false);
    this->firmware_download_running = true;

    // // create temporary file
    const auto date_time = Everest::Date::to_rfc3339(date::utc_clock::now());
    const auto firmware_file_path = fs::path("/srv/firmware-update.image");

    // ...here by deleting a possibly existing, older file
    if (fs::exists(firmware_file_path)) {
        fs::remove(firmware_file_path);
    }

    const auto firmware_downloader = this->scripts_path / SIGNED_FIRMWARE_DOWNLOADER;
    const auto constants = this->scripts_path / CONSTANTS;

    const std::vector<std::string> download_args = {
        constants.string(), firmware_update_request.location, firmware_file_path.string(),
        firmware_update_request.signature.value(), firmware_update_request.signing_certificate.value()};
    int32_t retries = 0;
    const auto total_retries = firmware_update_request.retries.value_or(this->mod->config.DefaultRetries);
    const auto retry_interval =
        firmware_update_request.retry_interval_s.value_or(this->mod->config.DefaultRetryInterval);

    auto firmware_status_enum = types::system::FirmwareUpdateStatusEnum::DownloadFailed;
    types::system::FirmwareUpdateStatus firmware_status;
    firmware_status.request_id = firmware_update_request.request_id;
    firmware_status.firmware_update_status = firmware_status_enum;

    while (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::DownloadFailed &&
           retries <= total_retries && !this->interrupt_firmware_download) {
        boost::process::ipstream download_stream;
        boost::process::child download_cmd(firmware_downloader.string(), boost::process::args(download_args),
                                           boost::process::std_out > download_stream);
        std::string temp;
        retries += 1;
        while (std::getline(download_stream, temp) && !this->interrupt_firmware_download) {
            firmware_status.firmware_update_status = types::system::string_to_firmware_update_status_enum(temp);
            this->publish_firmware_update_status(firmware_status);
        }
        if (this->interrupt_firmware_download) {
            EVLOG_info << "Updating firmware was interrupted, terminating firmware update script, requestId: "
                       << firmware_status.request_id;
            download_cmd.terminate();
        }
        if (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::DownloadFailed &&
            retries <= total_retries) {
            std::this_thread::sleep_for(std::chrono::seconds(retry_interval));
        }
    }
    if (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::SignatureVerified) {
        this->initialize_firmware_installation(firmware_update_request, firmware_file_path);
    }

    this->firmware_download_running = false;
    this->firmware_update_cv.notify_one();
    EVLOG_info << "Firmware update thread finished";
}

void systemImpl::initialize_firmware_installation(const types::system::FirmwareUpdateRequest& firmware_update_request,
                                                  const fs::path& firmware_file_path) {
    if (firmware_update_request.install_timestamp.has_value() &&
        Everest::Date::from_rfc3339(firmware_update_request.install_timestamp.value()) > date::utc_clock::now()) {
        const auto install_timestamp = Everest::Date::from_rfc3339(firmware_update_request.install_timestamp.value());
        this->signed_firmware_update_install_timer.at(
            [this, firmware_update_request, firmware_file_path]() {
                this->install_signed_firmware(firmware_update_request, firmware_file_path);
            },
            install_timestamp);
        EVLOG_info << "Installation for firmware scheduled for: " << Everest::Date::to_rfc3339(install_timestamp);
        types::system::FirmwareUpdateStatus firmware_update_status;
        firmware_update_status.request_id = firmware_update_request.request_id;
        firmware_update_status.firmware_update_status = types::system::FirmwareUpdateStatusEnum::InstallScheduled;
        this->publish_firmware_update_status(firmware_update_status);
    } else {
        // start installation immediately
        this->update_firmware_thread = std::thread([this, firmware_update_request, firmware_file_path]() {
            this->install_signed_firmware(firmware_update_request, firmware_file_path);
        });
        this->update_firmware_thread.detach();
    }
}

void systemImpl::install_signed_firmware(const types::system::FirmwareUpdateRequest& firmware_update_request,
                                         const fs::path& firmware_file_path) {
    auto firmware_status_enum = types::system::FirmwareUpdateStatusEnum::Installing;
    types::system::FirmwareUpdateStatus firmware_status;
    firmware_status.request_id = firmware_update_request.request_id;
    firmware_status.firmware_update_status = firmware_status_enum;

    const std::string partition = get_partition(PartitionType::Inactive);
    // write a flag/marker file which gets copied to the new installation, which indicates what partition we
    // expect to be on after update
    {
        std::ofstream marker_file(MARKER_FILE_PATH, std::ios::trunc);
        marker_file << partition << "\n" << firmware_status.request_id << std::endl;
    }

    if (!this->firmware_installation_running) {
        this->firmware_installation_running = true;
        boost::process::ipstream install_stream;
        const auto firmware_installer = this->scripts_path / SIGNED_FIRMWARE_INSTALLER;
        const auto constants = this->scripts_path / CONSTANTS;
        const std::vector<std::string> install_args = {constants.string(), firmware_file_path.string()};
        boost::process::child install_cmd(firmware_installer.string(), boost::process::args(install_args),
                                          boost::process::std_out > install_stream);
        std::string temp;
        while (std::getline(install_stream, temp)) {
            firmware_status.firmware_update_status = types::system::string_to_firmware_update_status_enum(temp);
            this->publish_firmware_update_status(firmware_status);
        }
    } else {
        firmware_status.firmware_update_status = types::system::FirmwareUpdateStatusEnum::InstallationFailed;
        this->publish_firmware_update_status(firmware_status);
    }

    this->firmware_installation_running = false;
    if (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::InstallRebooting) {
        EVLOG_warning << "Secure firmware update finished successfully. Initiating reboot in 5 seconds";
        std::this_thread::sleep_for(std::chrono::seconds(5));
        EVLOG_info << "Rebooting...";
        boost::process::system("reboot");
    } else if (firmware_status.firmware_update_status == types::system::FirmwareUpdateStatusEnum::InstallationFailed) {
        fs::remove(MARKER_FILE_PATH);
    }
}

types::system::UpdateFirmwareResponse
systemImpl::handle_update_firmware(types::system::FirmwareUpdateRequest& firmware_update_request) {
    if (firmware_update_request.request_id == -1) {
        return this->handle_standard_firmware_update(firmware_update_request);
    } else {
        return this->handle_signed_fimware_update(firmware_update_request);
    }
};

void systemImpl::handle_allow_firmware_installation() {
    // TODO: implement me
}

static std::string escape_single_quotes(const std::string& str) {
    std::stringstream ss;

    for (const char& ch : str) {
        if (ch == '\'') {
            ss << R"('"'"')";
        } else {
            ss << ch;
        }
    }

    return ss.str();
}

static bool build_upload_logs(const std::string& targetFile) {
    std::stringstream ss;
    std::string command;
    int i, rv;

    // Start new file
    ss << "echo \"\" > " << diagnostic_files[0];

    rv = std::system(ss.str().c_str());
    if (rv) {
        EVLOG_error << "echo (starting a new file) returned: rv = " << rv;
    }

    for (i = 0; diagnostic_commands[i]; i++) {

        // Generate header with command description
        ss.str("");
        ss << "echo '" << escape_single_quotes(std::string(diagnostic_commands[i])) << "' >> " << diagnostic_files[0];

        rv = std::system(ss.str().c_str());
        if (rv) {
            EVLOG_error << "echo (generating command description) returned: rv = " << rv;
        }

        // Append command output
        ss.str("");
        ss << diagnostic_commands[i] << " >> " << diagnostic_files[0] << " 2>&1";
        command = ss.str();

        rv = std::system(command.c_str());
        if (rv) {
            EVLOG_error << "command \"" << diagnostic_commands[i] << "\" returned: rv = " << rv;
        }

        // Append empty line
        ss.str("");
        ss << "echo \"\" >> " << diagnostic_files[0];

        rv = std::system(ss.str().c_str());
        if (rv) {
            EVLOG_error << "echo (appending a new line) returned: rv = " << rv;
        }
    }

    // This doesn't work with busybox tar
    ss.str("");
    ss << "tar -c -h --ignore-failed-read -z";
    ss << " -f " << targetFile << " --";

    for (i = 0; diagnostic_files[i]; i++) {
        ss << " " << diagnostic_files[i];
    }

    rv = std::system(ss.str().c_str());
    if (rv) {
        EVLOG_error << "tar returned: rv = " << rv;
    }

    fs::remove(diagnostic_files[0]);

    return rv == 0;
}

types::system::UploadLogsResponse
systemImpl::handle_upload_logs(types::system::UploadLogsRequest& upload_logs_request) {

    types::system::UploadLogsResponse response;

    if (this->log_upload_running) {
        response.upload_logs_status = types::system::UploadLogsStatus::AcceptedCancelled;
    } else {
        response.upload_logs_status = types::system::UploadLogsStatus::Accepted;
    }

    const auto date_time = Everest::Date::to_rfc3339(date::utc_clock::now());
    // TODO(piet): consider start time and end time

    const auto diagnostics_file_name = "diagnostics" + date_time + ".tar.gz";
    const auto diagnostics_file_path = fs::temp_directory_path() / diagnostics_file_name;
    EVLOG_info << "Diagnostics file: " << diagnostics_file_path;

    response.file_name = diagnostics_file_name;

    if (!build_upload_logs(diagnostics_file_path.string())) {
        response.upload_logs_status = types::system::UploadLogsStatus::Rejected;
        return response;
    }

    this->upload_logs_thread = std::thread([this, upload_logs_request, diagnostics_file_name, diagnostics_file_path]() {
        if (this->log_upload_running) {
            EVLOG_info << "Received Log upload request and log upload already running - cancelling current upload";
            this->interrupt_log_upload = true;
            EVLOG_info << "Waiting for other log upload to finish...";
            std::unique_lock<std::mutex> lk(this->log_upload_mutex);
            this->log_upload_cv.wait(lk, [this]() { return !this->log_upload_running; });
            EVLOG_info << "Previous Log upload finished!";
        }

        std::lock_guard<std::mutex> lg(this->log_upload_mutex);
        EVLOG_info << "Starting upload of log file";
        this->interrupt_log_upload = false;
        this->log_upload_running = true;
        const auto diagnostics_uploader = this->scripts_path / DIAGNOSTICS_UPLOADER;
        const auto constants = this->scripts_path / CONSTANTS;

        std::vector<std::string> args = {constants.string(), upload_logs_request.location, diagnostics_file_name,
                                         diagnostics_file_path.string()};
        bool uploaded = false;
        int32_t retries = 0;
        const auto total_retries = upload_logs_request.retries.value_or(this->mod->config.DefaultRetries);
        const auto retry_interval =
            upload_logs_request.retry_interval_s.value_or(this->mod->config.DefaultRetryInterval);

        types::system::LogStatus log_status;
        while (!uploaded && retries <= total_retries && !this->interrupt_log_upload) {

            boost::process::ipstream stream;
            boost::process::child cmd(diagnostics_uploader.string(), boost::process::args(args),
                                      boost::process::std_out > stream);
            std::string temp;
            retries += 1;
            log_status.request_id = upload_logs_request.request_id.value_or(-1);
            while (std::getline(stream, temp) && !this->interrupt_log_upload) {
                if (temp == "Uploaded") {
                    log_status.log_status = types::system::string_to_log_status_enum(temp);
                } else if (temp == "UploadFailure" || temp == "PermissionDenied" || temp == "BadMessage" ||
                           temp == "NotSupportedOperation") {
                    log_status.log_status = types::system::LogStatusEnum::UploadFailure;
                } else {
                    log_status.log_status = types::system::LogStatusEnum::Uploading;
                }
                EVLOG_info << "Log upload status: " << temp;
                this->publish_log_status(log_status);
            }
            if (this->interrupt_log_upload) {
                EVLOG_info << "Uploading Logs was interrupted, terminating upload script, requestId: "
                           << log_status.request_id;
                // N01.FR.20
                // FIXME: This enum is not yet implemented upstream. When it is, activate this code:
                // log_status.log_status = types::system::LogStatusEnum::AcceptedCanceled;
                // this->publish_log_status(log_status);
                cmd.terminate();
            } else if (log_status.log_status != types::system::LogStatusEnum::Uploaded && retries <= total_retries) {
                // command finished, but neither interrupted nor uploaded
                std::this_thread::sleep_for(std::chrono::seconds(retry_interval));
            } else {
                uploaded = true;
            }
            cmd.wait();
        }
        this->log_upload_running = false;
        this->log_upload_cv.notify_one();
        EVLOG_info << "Log upload thread finished";
        fs::remove(diagnostics_file_path);
    });
    this->upload_logs_thread.detach();

    return response;
};

bool systemImpl::handle_is_reset_allowed(types::system::ResetType& type) {
    // Right now we dont want to reject a reset ever
    (void)type;
    return true;
}

void systemImpl::handle_reset(types::system::ResetType& type, bool& scheduled) {
    // let the actual work be done by a worker thread, which can also delay it
    // a little bit (if configured) to allow e.g. clean shutdown of communication
    // channels in parallel when this call returns
    std::thread([this, type, scheduled] {
        EVLOG_info << "Reset request received: " << type << ", " << (scheduled ? "" : "not ") << "scheduled";

        std::this_thread::sleep_for(std::chrono::seconds(this->mod->config.ResetDelay));

        if (type == types::system::ResetType::Soft) {
            EVLOG_info << "Performing soft reset now.";
            kill(getpid(), SIGINT);
        } else {
            EVLOG_info << "Performing hard reset. Rebooting...";
            boost::process::system("reboot");
        }
    }).detach();
}

bool systemImpl::handle_set_system_time(std::string& timestamp) {
    int exit_code = 1;

    // FIXME: Time is set on every Heartbeat due to milliseconds differences. This needs a proper fix
    static bool time_is_set = false;

    if (!time_is_set && (timestamp != Everest::Date::to_rfc3339(date::utc_clock::now()))) {
        EVLOG_debug << "Setting system time to: " << timestamp;
        exit_code = boost::process::system("/bin/date", "--set", timestamp);
        time_is_set = true;
    }

    return (exit_code == 0);
};

types::system::BootReason systemImpl::handle_get_boot_reason() {
    return this->boot_reason;
}

} // namespace main
} // namespace module
