#pragma once


#include "RTTriangle.hpp"
#include "RaycastResult.hpp"
#include "rtlib.hpp"
#include "Bvh.hpp"

#include "base/String.hpp"

#include <vector>
#include <atomic>

namespace FW
{

// Given a vector n, forms an orthogonal matrix with n as the last column, i.e.,
// a coordinate system aligned such that n is its local z axis.
// You'll have to fill in the implementation for this.
Mat3f formBasis(const Vec3f& n);

Vec2f getTexelCoords(Vec2f uv, const Vec2i size);


// Main class for tracing rays using BVHs.
class RayTracer {
public:
                        RayTracer				(void);
                        ~RayTracer				(void);

	void				constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode);

    void				saveHierarchy			(const char* filename, const std::vector<RTTriangle>& triangles);
    void				loadHierarchy			(const char* filename, std::vector<RTTriangle>& triangles);

    RaycastResult		raycast					(const Vec3f& orig, const Vec3f& dir) /*const*/ ;     // TODO check the const!

    void RayTracer::partitionPrimitives(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, uint32_t start, uint32_t end, uint32_t& mid, AABB bb);
    void RayTracer::constructBvh(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, BvhNode& node, uint32_t start, uint32_t end);
    RaycastResult RayTracer::traverseBvh(const Vec3f& orig, const Vec3f& dir, const BvhNode& node);
    bool RayTracer::rayBBIntersect(const Vec3f& orig, const Vec3f& dir, BvhNode& node, float& t_start);

    // This function computes an MD5 checksum of the input scene data,
    // WITH the assumption that all vertices are allocated in one big chunk.
    static FW::String	computeMD5				(const std::vector<Vec3f>& vertices);

    std::vector<RTTriangle>* m_triangles;
    int m_triangleCount;
    //void setTriangleCount(int count) { m_triangleCount = count; }

	void resetRayCounter() { m_rayCount = 0; }
	int getRayCount() { return m_rayCount; }
    bool hasBeenInitalized() { return m_bvhInitalized; }
    void initalizeBvh() { m_bvhInitalized = true; }

private:
	mutable std::atomic<int> m_rayCount;
	Bvh m_bvh;
    bool m_bvhInitalized = false;
};


} // namespace FW