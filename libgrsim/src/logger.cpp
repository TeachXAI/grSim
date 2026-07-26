#include "grsim/logger.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace grsim {

namespace {
bool mkdirs(const std::string& path) {
    // create single or nested directory (simple)
    std::string cur;
    for (size_t i = 0; i < path.size(); i++) {
        cur.push_back(path[i]);
        if (path[i] == '/' || i + 1 == path.size()) {
            if (cur.back() == '/') {
                std::string d = cur.substr(0, cur.size() - 1);
                if (!d.empty()) mkdir(d.c_str(), 0755);
            } else {
                mkdir(cur.c_str(), 0755);
            }
        }
    }
    return true;
}

std::string makeRunId(const std::string& prefix) {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << prefix << "_"
        << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}
}  // namespace

SimLogger::~SimLogger() { close(); }

void SimLogger::open(const LoggingSettings& settings, const std::string& run_id) {
    close();
    if (!settings.enabled) return;
    log_vision_ = settings.log_vision;
    log_commands_ = settings.log_commands;
    run_id_ = run_id.empty() ? makeRunId(settings.prefix) : run_id;
    mkdirs(settings.directory);
    vision_path_ = settings.directory + "/" + run_id_ + "_vision.csv";
    command_path_ = settings.directory + "/" + run_id_ + "_commands.csv";

    if (log_vision_) {
        vision_out_.open(vision_path_);
        vision_out_ << "frame,t,ball_x,ball_y,ball_z,team,id,x,y,orientation,vx,vy,omega\n";
    }
    if (log_commands_) {
        cmd_out_.open(command_path_);
        cmd_out_ << "frame,t,team,id,vel_tangent,vel_normal,vel_angular,kick_x,kick_z,spinner\n";
    }
}

void SimLogger::close() {
    if (vision_out_.is_open()) vision_out_.close();
    if (cmd_out_.is_open()) cmd_out_.close();
}

void SimLogger::logVision(const VisionFrame& frame) {
    if (!log_vision_ || !vision_out_.is_open()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    auto write_robot = [&](const RobotState& r) {
        vision_out_ << frame.frame_number << ","
                    << frame.t_capture << ","
                    << frame.ball.x << "," << frame.ball.y << "," << frame.ball.z << ","
                    << (r.team == Team::Blue ? "blue" : "yellow") << ","
                    << r.id << ","
                    << r.x << "," << r.y << ","
                    << r.orientation << ","
                    << r.vx << "," << r.vy << "," << r.omega << "\n";
    };
    if (frame.robots_blue.empty() && frame.robots_yellow.empty()) {
        // still log ball with dummy robot -1
        vision_out_ << frame.frame_number << ","
                    << frame.t_capture << ","
                    << frame.ball.x << "," << frame.ball.y << "," << frame.ball.z << ","
                    << "none,-1,0,0,0,0,0,0\n";
    }
    for (const auto& r : frame.robots_blue) write_robot(r);
    for (const auto& r : frame.robots_yellow) write_robot(r);
}

void SimLogger::logCommands(double t, uint64_t frame, const std::vector<RobotCommand>& cmds) {
    if (!log_commands_ || !cmd_out_.is_open()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& c : cmds) {
        cmd_out_ << frame << "," << t << ","
                 << (c.team == Team::Blue ? "blue" : "yellow") << ","
                 << c.id << ","
                 << c.vel_tangent << "," << c.vel_normal << "," << c.vel_angular << ","
                 << c.kick_speed_x << "," << c.kick_speed_z << ","
                 << (c.spinner ? 1 : 0) << "\n";
    }
}

}  // namespace grsim
