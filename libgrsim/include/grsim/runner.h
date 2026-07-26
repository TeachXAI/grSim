#pragma once

#include "grsim/world.h"
#include "grsim/client.h"
#include "grsim/logger.h"
#include "grsim/config.h"
#include <atomic>
#include <thread>
#include <memory>
#include <chrono>
#include <string>

namespace grsim {

struct RunResult {
    uint64_t frames = 0;
    double sim_time = 0;
    double wall_time_sec = 0;
    std::string vision_log;
    std::string command_log;
    std::string run_id;
};

class SimulationRunner {
public:
    SimulationRunner(SimConfig config, std::unique_ptr<ClientController> client);
    ~SimulationRunner();

    // Run for duration_sec of simulated time
    // sync: client called every control_period on the sim thread
    // async: client runs on a separate thread at control_period wall-clock-ish,
    //        commands applied at each sim step via queue
    RunResult run(double duration_sec, RunMode mode = RunMode::Sync);

    // Single-step helpers for ML training loops
    void reset();
    VisionFrame stepOnce();  // one physics step + optional client tick
    void setClient(std::unique_ptr<ClientController> client);

    SimWorld& world() { return *world_; }
    const SimWorld& world() const { return *world_; }
    SimLogger& logger() { return logger_; }

private:
    RunResult runSync(double duration_sec);
    RunResult runAsync(double duration_sec);
    void maybeClientTick(double t, double& last_client_t, double client_dt);

    SimConfig cfg_;
    std::unique_ptr<SimWorld> world_;
    std::unique_ptr<ClientController> client_;
    SimLogger logger_;
    std::atomic<bool> async_running_{false};
    std::thread async_thread_;
    double last_client_t_ = -1e9;
};

}  // namespace grsim
