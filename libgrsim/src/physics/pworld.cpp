#include "grsim/physics/pworld.h"
#include <atomic>

namespace grsim {

namespace {
std::atomic<int> g_ode_refcount{0};
void odeAcquire() {
    if (g_ode_refcount.fetch_add(1) == 0) {
        dInitODE();
    }
}
void odeRelease() {
    if (g_ode_refcount.fetch_sub(1) == 1) {
        dCloseODE();
    }
}
}  // namespace

PSurface::PSurface() {
    usefdir1 = false;
    surface.mode = dContactApprox1;
    surface.mu = 0.5;
}

bool PSurface::isIt(dGeomID i1, dGeomID i2) const {
    return ((i1 == id1) && (i2 == id2)) || ((i1 == id2) && (i2 == id1));
}

static void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    static_cast<PWorld*>(data)->handleCollisions(o1, o2);
}

PWorld::PWorld(dReal dt, dReal gravity, int _robot_count) {
    robot_count = _robot_count;
    odeAcquire();
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -gravity);
    objects_count = 0;
    sur_matrix = nullptr;
    delta_time = dt;
}

PWorld::~PWorld() {
    for (auto* s : surfaces) delete s;
    surfaces.clear();
    if (sur_matrix) {
        for (int i = 0; i < objects_count; i++) delete[] sur_matrix[i];
        delete[] sur_matrix;
        sur_matrix = nullptr;
    }
    // Destroying the world/space frees all bodies and geoms.
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    world = nullptr;
    space = nullptr;
    // Clear wrapper pointers so ~PObject does not touch freed IDs
    for (auto* o : objects) {
        if (o) {
            o->body = nullptr;
            o->geom = nullptr;
            o->world = nullptr;
            o->space = nullptr;
        }
    }
    objects.clear();
    odeRelease();
}

void PWorld::setGravity(dReal gravity) {
    dWorldSetGravity(world, 0, 0, -gravity);
}

void PWorld::handleCollisions(dGeomID o1, dGeomID o2) {
    int j = sur_matrix[*static_cast<int*>(dGeomGetData(o1))]
                      [*static_cast<int*>(dGeomGetData(o2))];
    if (j == -1) return;

    const int N = 10;
    dContact contact[N];
    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (n <= 0) return;

    PSurface* sur = surfaces[j];
    sur->contactPos[0] = contact[0].geom.pos[0];
    sur->contactPos[1] = contact[0].geom.pos[1];
    sur->contactPos[2] = contact[0].geom.pos[2];
    sur->contactNormal[0] = contact[0].geom.normal[0];
    sur->contactNormal[1] = contact[0].geom.normal[1];
    sur->contactNormal[2] = contact[0].geom.normal[2];

    bool flag = true;
    if (sur->callback) flag = sur->callback(o1, o2, sur, robot_count);
    if (!flag) return;

    for (int i = 0; i < n; i++) {
        contact[i].surface = sur->surface;
        if (sur->usefdir1) {
            contact[i].fdir1[0] = sur->fdir1[0];
            contact[i].fdir1[1] = sur->fdir1[1];
            contact[i].fdir1[2] = sur->fdir1[2];
            contact[i].fdir1[3] = sur->fdir1[3];
        }
        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
    }
}

void PWorld::addObject(PObject* o) {
    int oid = static_cast<int>(objects.size());
    o->id = oid;
    if (!o->world) o->world = world;
    if (!o->space) o->space = space;
    o->init();
    dGeomSetData(o->geom, static_cast<void*>(&o->id));
    objects.push_back(o);
}

void PWorld::initAllObjects() {
    objects_count = static_cast<int>(objects.size());
    int c = objects_count;
    bool flag = (sur_matrix != nullptr);
    if (sur_matrix) {
        for (int i = 0; i < c; i++) delete[] sur_matrix[i];
        // Note: previous count may differ; use objects_count after rebuild carefully.
        // We delete based on previous allocation size stored - reallocate cleanly.
        delete[] sur_matrix;
    }
    sur_matrix = new int*[c];
    for (int i = 0; i < c; i++) {
        sur_matrix[i] = new int[c];
        for (int j = 0; j < c; j++) sur_matrix[i][j] = -1;
    }
    if (flag) {
        for (size_t i = 0; i < surfaces.size(); i++) {
            int a = *static_cast<int*>(dGeomGetData(surfaces[i]->id1));
            int b = *static_cast<int*>(dGeomGetData(surfaces[i]->id2));
            sur_matrix[a][b] = sur_matrix[b][a] = static_cast<int>(i);
        }
    }
}

PSurface* PWorld::createSurface(PObject* o1, PObject* o2) {
    auto* s = new PSurface();
    s->id1 = o1->geom;
    s->id2 = o2->geom;
    surfaces.push_back(s);
    sur_matrix[o1->id][o2->id] = sur_matrix[o2->id][o1->id] = static_cast<int>(surfaces.size()) - 1;
    return s;
}

PSurface* PWorld::findSurface(PObject* o1, PObject* o2) {
    for (auto* s : surfaces) {
        if (s->isIt(o1->geom, o2->geom)) return s;
    }
    return nullptr;
}

void PWorld::step(dReal dt) {
    dSpaceCollide(space, this, &nearCallback);
    dWorldStep(world, (dt < 0) ? delta_time : dt);
    dJointGroupEmpty(contactgroup);
}

}  // namespace grsim
