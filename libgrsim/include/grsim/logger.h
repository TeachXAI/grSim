#pragma once

#include "grsim/types.h"
#include "grsim/config.h"
#include <fstream>
#include <string>
#include <mutex>

namespace grsim {

// Writes vision and command logs as CSV for post-run visualization
class SimLogger {
public:
    SimLogger() = default;
    ~SimLogger();

    void open(const LoggingSettings& settings, const std::string& run_id = "");
    void close();
    bool isOpen() const { return vision_out_.is_open() || cmd_out_.is_open(); }

    void logVision(const VisionFrame& frame);
    void logCommands(double t, uint64_t frame, const std::vector<RobotCommand>& cmds);

    std::string visionPath() const { return vision_path_; }
    std::string commandPath() const { return command_path_; }
    std::string runId() const { return run_id_; }

private:
    std::ofstream vision_out_;
    std::ofstream cmd_out_;
    std::string vision_path_;
    std::string command_path_;
    std::string run_id_;
    std::mutex mutex_;
    bool log_vision_ = true;
    bool log_commands_ = true;
};

}  // namespace grsim
