#pragma once

namespace grsim {

constexpr int kMaxRobotCount = 16;
constexpr int kTeamCount = 2;
constexpr int kWheelCount = 4;
constexpr int kWallCount = 10;

enum class Team { Blue = 0, Yellow = 1 };

enum class KickStatus {
    NoKick = 0,
    FlatKick = 1,
    ChipKick = 2
};

enum class RunMode {
    Sync,
    Async
};

enum class BehaviorType {
    Idle,
    Circle,
    Square
};

}  // namespace grsim
