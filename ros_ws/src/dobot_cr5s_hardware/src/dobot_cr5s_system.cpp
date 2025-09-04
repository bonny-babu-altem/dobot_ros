#include "dobot_cr5s_hardware/dobot_cr5s_system.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <cmath>

#define HW_IF_POSITION "position"
#define HW_IF_VELOCITY "velocity"

using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

namespace dobot_cr5s_hardware
{

    static bool set_socket_timeout(int fd, int ms)
    {
        timeval tv{};
        tv.tv_sec = ms / 1000;
        tv.tv_usec = (ms % 1000) * 1000;
        return setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0 &&
               setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) == 0;
    }

    CallbackReturn DobotCR5SSystem::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        auto getp = [&](const std::string &key, const std::string &def = "")
        {
            auto it = info.hardware_parameters.find(key);
            return (it == info.hardware_parameters.end()) ? def : it->second;
        };

        robot_ip_ = getp("robot_ip", robot_ip_);
        if (auto s = getp("dashboard_port"); !s.empty())
            dashboard_port_ = std::stoi(s);
        if (auto s = getp("feedback_port"); !s.empty())
            feedback_port_ = std::stoi(s);
        if (auto s = getp("a_ratio"); !s.empty())
            a_ratio_ = std::stod(s);
        if (auto s = getp("v_ratio"); !s.empty())
            v_ratio_ = std::stod(s);
        if (auto s = getp("send_threshold_deg"); !s.empty())
            send_threshold_rad_ = std::stod(s) * M_PI / 180.0;

        joint_names_.clear();
        for (const auto &j : info.joints)
        {
            if (j.name == "unconfigured")
                continue;
            joint_names_.push_back(j.name);
        }
        if (joint_names_.size() != 6)
        {
            RCLCPP_ERROR(logger_, "Expected 6 joints, got %zu", joint_names_.size());
            return CallbackReturn::ERROR;
        }

        pos_.fill(0.0);
        vel_.fill(0.0);
        cmd_.fill(0.0);
        last_sent_.fill(1e9);

        last_pos_.fill(0.0);

        return CallbackReturn::SUCCESS;
    }

    std::vector<StateInterface> DobotCR5SSystem::export_state_interfaces()
    {
        std::vector<StateInterface> st;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            st.emplace_back(joint_names_[i], HW_IF_POSITION, &pos_[i]);
            st.emplace_back(joint_names_[i], HW_IF_VELOCITY, &vel_[i]);
        }
        return st;
    }

    std::vector<CommandInterface> DobotCR5SSystem::export_command_interfaces()
    {
        std::vector<CommandInterface> cmd;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            cmd.emplace_back(joint_names_[i], HW_IF_POSITION, &cmd_[i]);
        }
        return cmd;
    }

    bool DobotCR5SSystem::connect_dashboard_()
    {
        dash_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (dash_fd_ < 0)
            return false;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(dashboard_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &addr.sin_addr);
        if (connect(dash_fd_, (sockaddr *)&addr, sizeof(addr)) != 0)
            return false;
        set_socket_timeout(dash_fd_, 1000);
        return true;
    }

    bool DobotCR5SSystem::connect_feedback_()
    {
        fb_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (fb_fd_ < 0)
            return false;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(feedback_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &addr.sin_addr);
        if (connect(fb_fd_, (sockaddr *)&addr, sizeof(addr)) != 0)
            return false;
        set_socket_timeout(fb_fd_, 1000);
        return true;
    }

    void DobotCR5SSystem::close_all_()
    {
        fb_run_.store(false);
        if (fb_thread_.joinable())
            fb_thread_.join();
        if (dash_fd_ >= 0)
        {
            close(dash_fd_);
            dash_fd_ = -1;
        }
        if (fb_fd_ >= 0)
        {
            close(fb_fd_);
            fb_fd_ = -1;
        }
    }

    bool DobotCR5SSystem::send_dashboard_cmd_(const std::string &cmd, std::string *resp)
    {
        std::string out = cmd;
        if (out.empty() || out.back() != ';')
            out.push_back(';');
        ssize_t n = ::send(dash_fd_, out.c_str(), out.size(), 0);
        if (n != (ssize_t)out.size())
            return false;

        std::string line;
        line.reserve(128);
        char ch;
        while (true)
        {
            ssize_t r = recv(dash_fd_, &ch, 1, 0);
            if (r <= 0)
                break;
            if (ch == '\n')
                break;
            if (ch != '\r')
                line.push_back(ch);
        }
        if (resp)
            *resp = line;
        return !line.empty();
    }

    CallbackReturn DobotCR5SSystem::on_activate(const rclcpp_lifecycle::State &)
    {
        if (!connect_dashboard_())
        {
            RCLCPP_ERROR(logger_, "Failed to connect dashboard at %s:%d", robot_ip_.c_str(), dashboard_port_);
            return CallbackReturn::ERROR;
        }
        if (!connect_feedback_())
        {
            RCLCPP_ERROR(logger_, "Failed to connect feedback at %s:%d", robot_ip_.c_str(), feedback_port_);
            close_all_();
            return CallbackReturn::ERROR;
        }

        std::string resp;
        if (!send_dashboard_cmd_("RequestControl()", &resp))
        {
            RCLCPP_ERROR(logger_, "RequestControl() failed");
            close_all_();
            return CallbackReturn::ERROR;
        }

        send_dashboard_cmd_("PowerOn()", &resp);

        auto poll_mode = [&]() -> int
        {
            std::string r;
            if (!send_dashboard_cmd_("RobotMode()", &r))
                return -1;
            auto l = r.find('{');
            auto m = r.find('}');
            if (l == std::string::npos || m == std::string::npos || m <= l + 1)
                return -1;
            return std::stoi(r.substr(l + 1, m - l - 1));
        };

        for (int i = 0; i < 150; ++i)
        {
            int mode = poll_mode();
            if (mode == 4 || mode == 5 || mode == 7)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        send_dashboard_cmd_("EnableRobot()", &resp);

        fb_run_.store(true);
        fb_thread_ = std::thread(&DobotCR5SSystem::feedback_loop_, this);

        RCLCPP_INFO(logger_, "Dobot hardware activated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DobotCR5SSystem::on_deactivate(const rclcpp_lifecycle::State &)
    {
        fb_run_.store(false);
        if (fb_thread_.joinable())
            fb_thread_.join();
        close_all_();
        return CallbackReturn::SUCCESS;
    }

    void DobotCR5SSystem::feedback_loop_()
    {
        constexpr size_t PKT_SIZE = 1440;
        constexpr size_t QACTUAL_OFFSET = 432;
        constexpr size_t QACTUAL_LEN = 6 * sizeof(double);

        std::vector<uint8_t> buf(PKT_SIZE);
        while (fb_run_.load())
        {
            size_t read_total = 0;
            while (read_total < PKT_SIZE && fb_run_.load())
            {
                ssize_t r = recv(fb_fd_, buf.data() + read_total, PKT_SIZE - read_total, 0);
                if (r <= 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }
                read_total += static_cast<size_t>(r);
            }
            if (!fb_run_.load())
                break;
            if (read_total != PKT_SIZE)
                continue;

            double qact[6];
            std::memcpy(qact, buf.data() + QACTUAL_OFFSET, QACTUAL_LEN);

            std::lock_guard<std::mutex> lock(fb_mutex_);
            for (int i = 0; i < 6; ++i)
            {
                pos_[i] = deg2rad(qact[i]);
            }
        }
    }

    return_type DobotCR5SSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // period in seconds
        double dt = period.seconds();
        if (dt <= 0.0)
            dt = 0.001; // prevent division by zero

        // Compute velocities using numerical differentiation
        std::lock_guard<std::mutex> lock(fb_mutex_);
        for (size_t i = 0; i < 6; ++i)
        {
            // vel = (current_pos - last_pos) / dt
            vel_[i] = (pos_[i] - last_pos_[i]) / dt;

            // save current position for next cycle
        }
        last_pos_ = pos_;

        return return_type::OK;
    }

    return_type DobotCR5SSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        double max_diff = 0.0;
        for (int i = 0; i < 6; ++i)
        {
            double d = std::fabs(cmd_[i] - last_sent_[i]);
            if (d > max_diff)
                max_diff = d;
        }
        if (max_diff < send_threshold_rad_)
            return return_type::OK;

        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(3);
        ss << "MovJ(joint={";
        for (int i = 0; i < 6; ++i)
        {
            ss << rad2deg(cmd_[i]);
            if (i < 5)
                ss << ",";
        }
        ss << "},a=" << 100 << ",v=" << 5 << ")";
        std::string resp;
        if (!send_dashboard_cmd_(ss.str(), &resp))
        {
            RCLCPP_WARN(logger_, "MovJ send failed");
        }
        else
        {
            last_sent_ = cmd_;
        }
        return return_type::OK;
    }

} // namespace dobot_cr5s_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dobot_cr5s_hardware::DobotCR5SSystem, hardware_interface::SystemInterface)
