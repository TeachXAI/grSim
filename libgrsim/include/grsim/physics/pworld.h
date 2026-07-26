#pragma once

#include "grsim/physics/pobject.h"
#include <vector>
#include <functional>

namespace grsim {

class PSurface;
class PWorld;

using PSurfaceCallback = std::function<bool(dGeomID o1, dGeomID o2, PSurface* s, int robot_count)>;

class PSurface {
public:
    PSurface();
    bool isIt(dGeomID i1, dGeomID i2) const;
    dSurfaceParameters surface{};
    dGeomID id1 = nullptr;
    dGeomID id2 = nullptr;
    bool usefdir1 = false;
    dVector3 fdir1{};
    dVector3 contactPos{};
    dVector3 contactNormal{};
    PSurfaceCallback callback;
};

class PWorld {
public:
    PWorld(dReal dt, dReal gravity, int robot_count);
    ~PWorld();
    void setGravity(dReal gravity);
    void addObject(PObject* o);
    void initAllObjects();
    PSurface* createSurface(PObject* o1, PObject* o2);
    PSurface* findSurface(PObject* o1, PObject* o2);
    void step(dReal dt = -1);
    void handleCollisions(dGeomID o1, dGeomID o2);

    dWorldID world = nullptr;
    dSpaceID space = nullptr;
    int robot_count = 0;
private:
    dJointGroupID contactgroup = nullptr;
    std::vector<PObject*> objects;
    std::vector<PSurface*> surfaces;
    dReal delta_time = 0.016;
    int** sur_matrix = nullptr;
    int objects_count = 0;
};

}  // namespace grsim
