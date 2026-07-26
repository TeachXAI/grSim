#include "grsim/runner.h"
#include "grsim/behaviors.h"
#include <iostream>

namespace grsim {

SimulationRunner::SimulationRunner(SimConfig config, std::unique_ptr<ClientController> client)
    : cfg_(std::move(config)), client_(std::move(client)) {
    world_ = std::make_unique<SimWorld>(cfg_);
    if (!client_) client_ = createBehavior(cfg_);
}

SimulationRunner::~SimulationRunner() {
    async_running_ = false;
    if (async_thread_.joinable()) async_thread_.join();
}

void SimulationRunner::setClient(std::unique_ptr<ClientController> client) {
    client_ = std::move(client);
}

void SimulationRunner::reset() {
    world_ = std::make_unique<SimWorld>(cfg_);
    if (client_) client_->reset();
    last_client_t_ = -1e9;
}

void SimulationRunner::maybeClientTick(double t, double& last_client_t, double client_dt) {
    if (t - last_client_t + 1e-12 >= client_dt) {
        auto vision = world_->captureVision();
        auto cmds = client_->compute(vision, t);
        world_->applyCommands(cmds);
        logger_.logCommands(t, world_->frameNumber(), cmds);
        last_client_t = t;
    }
}

VisionFrame SimulationRunner::stepOnce() {
    double client_dt = cfg_.client.control_period_ms / 1000.0;
    maybeClientTick(world_->simTime(), last_client_t_, client_dt);
    world_->step(cfg_.DeltaTime());
    auto vision = world_->captureVision();
    logger_.logVision(vision);
    return vision;
}

RunResult SimulationRunner::run(double duration_sec, RunMode mode) {
    if (cfg_.logging.enabled) {
        logger_.open(cfg_.logging);
    }
    RunResult result;
    auto t0 = std::chrono::steady_clock::now();
    if (mode == RunMode::Async) {
        result = runAsync(duration_sec);
    } else {
        result = runSync(duration_sec);
    }
    auto t1 = std::chrono::steady_clock::now();
    result.wall_time_sec = std::chrono::duration<double>(t1 - t0).count();
    result.vision_log = logger_.visionPath();
    result.command_log = logger_.commandPath();
    result.run_id = logger_.runId();
    logger_.close();
    return result;
}

RunResult SimulationRunner::runSync(double duration_sec) {
    RunResult result;
    double client_dt = cfg_.client.control_period_ms / 1000.0;
    double last_client_t = -1e9;
    double dt = cfg_.DeltaTime();

    // Initial vision log
    logger_.logVision(world_->captureVision());

    while (world_->simTime() < duration_sec) {
        maybeClientTick(world_->simTime(), last_client_t, client_dt);
        world_->step(dt);
        logger_.logVision(world_->captureVision());
    }
    result.frames = world_->frameNumber();
    result.sim_time = world_->simTime();
    return result;
}

RunResult SimulationRunner::runAsync(double duration_sec) {
    RunResult result;
    double client_dt = cfg_.client.control_period_ms / 1000.0;
    async_running_ = true;

    // Client thread: triggered on a consistent loop paced by *simulation time*
    // so that when the physics runs faster than real-time (ML training), the
    // controller still fires every control_period_ms of sim time. Commands are
    // enqueued and applied on the sim thread (no network).
    async_thread_ = std::thread([this, client_dt]() {
        double last_client_sim_t = -1e9;
        while (async_running_) {
            double t = world_->simTime();
            if (t - last_client_sim_t + 1e-12 >= client_dt) {
                auto vision = world_->captureVision();
                auto cmds = client_->compute(vision, vision.t_capture);
                world_->enqueueCommands(cmds);
                logger_.logCommands(vision.t_capture, vision.frame_number, cmds);
                last_client_sim_t = t;
            } else {
                // Avoid busy-spinning when sim is paused / slower than client
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    });

    double dt = cfg_.DeltaTime();
    logger_.logVision(world_->captureVision());
    while (world_->simTime() < duration_sec) {
        world_->step(dt);  // drains command queue
        logger_.logVision(world_->captureVision());
    }

    async_running_ = false;
    if (async_thread_.joinable()) async_thread_.join();

    result.frames = world_->frameNumber();
    result.sim_time = world_->simTime();
    return result;
}

}  // namespace grsim
