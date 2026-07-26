#pragma once

#include "grsim/types.h"
#include "grsim/world.h"
#include "grsim/config.h"
#include <vector>
#include <memory>
#include <functional>

namespace grsim {

// Interface for in-process clients (no network)
class ClientController {
public:
    virtual ~ClientController() = default;
    // Called each control tick with current vision; returns commands to apply
    virtual std::vector<RobotCommand> compute(const VisionFrame& vision, double t) = 0;
    virtual void reset() {}
};

using ClientFactory = std::function<std::unique_ptr<ClientController>(const SimConfig&)>;

}  // namespace grsim
