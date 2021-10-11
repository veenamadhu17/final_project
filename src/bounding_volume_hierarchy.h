#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <span>

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Implement these two functions for the Visual Debug.
    // The first function should return how many levels there are in the tree that you have constructed.
    // The second function should draw the bounding boxes of the nodes at the selected level.
    int numLevels() const;
    void debugDraw(int level);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

private:
    Scene* m_pScene;
};
