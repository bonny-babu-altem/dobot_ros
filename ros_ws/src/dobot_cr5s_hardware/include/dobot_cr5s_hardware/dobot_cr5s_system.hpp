#ifndef DOBOT_CR5S_SYSTEM_HPP_
#define DOBOT_CR5S_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_component_info.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

namespace dobot_cr5s_hardware
{

class DobotCR5SSystem : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DobotCR5SSystem)

    // Lifecycle callbacks
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

    // Read / Write
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Export interfaces
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("dobot_cr5s_hardware");

    // Internal state
    std::array<double, 6> pos_;
    std::array<double, 6> vel_;
    std::array<double, 6> cmd_;
    std::array<double, 6> last_sent_;
    std::vector<std::string> joint_names_;

    std::atomic<bool> fb_run_{false};
    std::thread fb_thread_;
    int dash_fd_ = -1;
    int fb_fd_ = -1;

    std::string robot_ip_;
    int dashboard_port_ = 29999;
    int feedback_port_ = 30005;
    double a_ratio_ = 1.0;
    double v_ratio_ = 1.0;
    double send_threshold_rad_ = 0.01;

    // Private methods
    void feedback_loop_();
    bool connect_dashboard_();
    bool connect_feedback_();
    void close_all_();
    bool send_dashboard_cmd_(const std::string &cmd, std::string *resp = nullptr);
};

} // namespace dobot_cr5s_hardware

#endif // DOBOT_CR5S_SYSTEM_HPP_
