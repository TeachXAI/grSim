#include "grsim/config.h"
#include "grsim/runner.h"
#include "grsim/behaviors.h"
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace grsim;

static void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  --config PATH       YAML config (default: config/default.yaml)\n"
        << "  --duration SEC      Simulated duration seconds (default: 10)\n"
        << "  --mode sync|async   Run mode (default from config)\n"
        << "  --behavior circle|square|idle\n"
        << "  --robots N          Robots per team\n"
        << "  --log-dir DIR       Log output directory\n"
        << "  --no-log            Disable logging\n"
        << "  --help              Show this help\n";
}

int main(int argc, char** argv) {
    std::string config_path = "config/default.yaml";
    double duration = 10.0;
    bool mode_set = false;
    RunMode mode = RunMode::Sync;
    bool behavior_set = false;
    BehaviorType behavior = BehaviorType::Circle;
    int robots = -1;
    std::string log_dir;
    bool no_log = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (a == "--config") {
            config_path = need("--config");
        } else if (a == "--duration") {
            duration = std::stod(need("--duration"));
        } else if (a == "--mode") {
            mode_set = true;
            std::string m = need("--mode");
            mode = (m == "async") ? RunMode::Async : RunMode::Sync;
        } else if (a == "--behavior") {
            behavior_set = true;
            std::string b = need("--behavior");
            if (b == "square") behavior = BehaviorType::Square;
            else if (b == "idle") behavior = BehaviorType::Idle;
            else behavior = BehaviorType::Circle;
        } else if (a == "--robots") {
            robots = std::stoi(need("--robots"));
        } else if (a == "--log-dir") {
            log_dir = need("--log-dir");
        } else if (a == "--no-log") {
            no_log = true;
        } else {
            std::cerr << "Unknown option: " << a << "\n";
            printUsage(argv[0]);
            return 2;
        }
    }

    SimConfig cfg;
    try {
        cfg = SimConfig::loadFromFile(config_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config '" << config_path << "': " << e.what()
                  << "\nTrying defaults...\n";
        cfg = SimConfig::defaults();
        // try alternate paths
        for (const char* p : {"../config/default.yaml", "../../config/default.yaml",
                              "config/default.yaml"}) {
            try {
                cfg = SimConfig::loadFromFile(p);
                config_path = p;
                std::cerr << "Loaded " << p << "\n";
                break;
            } catch (...) {}
        }
    }

    if (mode_set) cfg.client.mode = mode;
    if (behavior_set) cfg.client.behavior = behavior;
    if (robots > 0) cfg.robots_count = robots;
    if (!log_dir.empty()) cfg.logging.directory = log_dir;
    if (no_log) cfg.logging.enabled = false;

    std::cout << "grSim headless library runner\n"
              << "  config:    " << config_path << "\n"
              << "  mode:      " << (cfg.client.mode == RunMode::Async ? "async" : "sync") << "\n"
              << "  behavior:  "
              << (cfg.client.behavior == BehaviorType::Square ? "square"
                  : cfg.client.behavior == BehaviorType::Idle ? "idle" : "circle")
              << "\n"
              << "  robots:    " << cfg.robots_count << " per team\n"
              << "  duration:  " << duration << " s\n"
              << "  dt:        " << cfg.DeltaTime() << " s\n"
              << "  control:   every " << cfg.client.control_period_ms << " ms\n";

    auto client = createBehavior(cfg);
    SimulationRunner runner(cfg, std::move(client));

    // Seed robots near the behaviour path so demos show the shape quickly
    // instead of a long approach from the sideline formation.
    {
        const double R = cfg.client.circle_radius;
        const double half = cfg.client.square_size / 2.0;
        for (int i = 0; i < cfg.robots_count; i++) {
            double ang = i * (2.0 * M_PI / std::max(1, cfg.robots_count));
            if (cfg.client.behavior == BehaviorType::Circle) {
                runner.world().setRobotPose(i, Team::Blue,
                    -1.5 + R * std::cos(ang), R * std::sin(ang), ang * 180.0 / M_PI + 90.0);
                runner.world().setRobotPose(i, Team::Yellow,
                    1.5 + R * std::cos(-ang), R * std::sin(-ang), -ang * 180.0 / M_PI + 90.0);
            } else if (cfg.client.behavior == BehaviorType::Square) {
                double p = i * (cfg.client.square_size * 2.0);
                double px, py, txx, tyy;
                // reuse square perimeter helper via simple side placement
                px = -half + (i % 2) * half;
                py = -half + (i / 2) * half;
                runner.world().setRobotPose(i, Team::Blue, px - 0.5, py, 0);
                runner.world().setRobotPose(i, Team::Yellow, px + 0.5, py, 180);
            }
        }
        runner.world().setBallPose(0, 0);
    }

    auto result = runner.run(duration, cfg.client.mode);

    std::cout << "Run complete:\n"
              << "  frames:     " << result.frames << "\n"
              << "  sim_time:   " << result.sim_time << " s\n"
              << "  wall_time:  " << result.wall_time_sec << " s\n"
              << "  realtime x: " << (result.wall_time_sec > 0
                                        ? result.sim_time / result.wall_time_sec
                                        : 0)
              << "\n";
    if (!result.vision_log.empty()) {
        std::cout << "  vision log:  " << result.vision_log << "\n"
                  << "  command log: " << result.command_log << "\n"
                  << "  run_id:      " << result.run_id << "\n";
    }
    // Write a small meta file for the visualizer
    if (!result.run_id.empty()) {
        std::string meta = cfg.logging.directory + "/" + result.run_id + "_meta.txt";
        FILE* f = fopen(meta.c_str(), "w");
        if (f) {
            fprintf(f, "run_id=%s\n", result.run_id.c_str());
            fprintf(f, "vision=%s\n", result.vision_log.c_str());
            fprintf(f, "commands=%s\n", result.command_log.c_str());
            fprintf(f, "field_length=%.3f\n", cfg.Field_Length());
            fprintf(f, "field_width=%.3f\n", cfg.Field_Width());
            fprintf(f, "mode=%s\n", cfg.client.mode == RunMode::Async ? "async" : "sync");
            fprintf(f, "behavior=%s\n",
                    cfg.client.behavior == BehaviorType::Square ? "square"
                    : cfg.client.behavior == BehaviorType::Idle ? "idle" : "circle");
            fprintf(f, "robots_count=%d\n", cfg.robots_count);
            fprintf(f, "control_period_ms=%d\n", cfg.client.control_period_ms);
            fprintf(f, "delta_time=%.8f\n", cfg.DeltaTime());
            fprintf(f, "circle_radius=%.3f\n", cfg.client.circle_radius);
            fprintf(f, "square_size=%.3f\n", cfg.client.square_size);
            fprintf(f, "speed=%.3f\n", cfg.client.speed);
            fprintf(f, "frames=%llu\n", (unsigned long long)result.frames);
            fprintf(f, "sim_time=%.4f\n", result.sim_time);
            fprintf(f, "wall_time=%.4f\n", result.wall_time_sec);
            fclose(f);
            std::cout << "  meta:        " << meta << "\n";
        }
    }
    return 0;
}
