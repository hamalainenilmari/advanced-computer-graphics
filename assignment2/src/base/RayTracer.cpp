#define _CRT_SECURE_NO_WARNINGS

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "RayTracer.hpp"
#include <stdio.h>
#include "rtIntersect.inl"
#include <fstream>

#include "rtlib.hpp"


// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer( void* buffer, size_t bufLen, unsigned int* pDigest );


namespace FW
{


Vec2f getTexelCoords(Vec2f uv, const Vec2i size)
{
    float mappedX;
    float mappedY;
    if (uv.x < 0) {
        mappedX = -1.0f * uv.x - std::floor(-1.0f * uv.x); // e.g. uv.x = -4.5, mapped will be 0.5
    }
    else {
        mappedX = uv.x - std::floor(uv.x);
    }
    if (uv.y < 0) {

        mappedY = -1.0f * uv.y - std::floor(-1.0f * uv.y);
    }
    else {
        mappedY = uv.y - std::floor(uv.y);
    }
    return Vec2f(mappedX * size.x, mappedY * size.y);
    //return Vec2f();
	// Convert texture coordinate to pixel index as you did in assignment 1.
}

Mat3f formBasis(const Vec3f& n) {
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
    Mat3f R = Mat3f(.0f);
    Vec3f Q;
    // set the smallest abs element of Q/N to 1
    float x = std::abs(n.x);
    float y = std::abs(n.y);
    float z = std::abs(n.z);

    if (x < y && x < z) { Q = Vec3f(1.0, 0.0, 0.0); }
    else if (y < z) { Q = Vec3f(0.0, 1.0, 0.0); }
    else { Q = Vec3f(0.0, 0.0, 1.0); }

    // perpendicular to each other
    Vec3f T = cross(Q, n).normalized();
    Vec3f B = cross(n, T).normalized();

    R.m00 = T.x;
    R.m10 = T.y;
    R.m20 = T.z;

    R.m01 = B.x;
    R.m11 = B.y;
    R.m21 = B.z;

    R.m02 = n.x;
    R.m12 = n.y;
    R.m22 = n.z;

    return R;
    // return rtlib::formBasis(n);
}


String RayTracer::computeMD5( const std::vector<Vec3f>& vertices )
{
    unsigned char digest[16];
    MD5Buffer( (void*)&vertices[0], sizeof(Vec3f)*vertices.size(), (unsigned int*)digest );

    // turn into string
    char ad[33];
    for ( int i = 0; i < 16; ++i )
        ::sprintf( ad+i*2, "%02x", digest[i] );
    ad[32] = 0;

    return FW::String( ad );
}


// --------------------------------------------------------------------------


RayTracer::RayTracer()
{
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
    // After that you can remove the following line.
    //m_rt.reset(new rtlib::RayTracer);
}

RayTracer::~RayTracer()
{
}

// create the axis-aligned bounding box for the group of objects
AABB computeBB(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, uint32_t start, uint32_t end) {
    // initalize min point and max point of bb by first triangle
    Vec3f min = triangles[indiceList[start]].min();
    Vec3f max = triangles[indiceList[start]].max();

    for (uint32_t i = start; i < end; ++i) {
        // update the min/max point coordinates according to the current min/max
        RTTriangle tri = triangles[indiceList[i]];
        min.x = std::min(min.x, tri.min().x);
        min.y = std::min(min.y, tri.min().y);
        min.z = std::min(min.z, tri.min().z);

        max.x = std::max(max.x, tri.max().x);
        max.y = std::max(max.y, tri.max().y);
        max.z = std::max(max.z, tri.max().z);
    }

    return AABB(min, max);
}

void RayTracer::partitionPrimitives(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, uint32_t start, uint32_t end, uint32_t& mid, AABB bb) {
    // AABB of centroids to create actual split

    // initalize min point and max point of bb by first triangle centroid
    Vec3f centroidMin = triangles[indiceList[start]].centroid();
    Vec3f centroidMax = centroidMin;

    // get bb min, max points
    for (uint32_t i = start; i < end; ++i) {
        Vec3f ct = triangles[indiceList[i]].centroid();
        centroidMin.x = std::min(centroidMin.x, ct.x);
        centroidMin.y = std::min(centroidMin.y, ct.y);
        centroidMin.z = std::min(centroidMin.z, ct.z);

        centroidMax.x = std::max(centroidMax.x, ct.x);
        centroidMax.y = std::max(centroidMax.y, ct.y);
        centroidMax.z = std::max(centroidMax.z, ct.z);
    }

    // get longest axis
    float x = centroidMax.x - centroidMin.x;
    float y = centroidMax.y - centroidMin.y;
    float z = centroidMax.z - centroidMin.z;
    float longestAxis = std::max({ x, y, z });

    Vec3f midPlane = (centroidMin + centroidMax) * 0.5f;

    auto leftSide = start; // beginning of this nodes partition
    auto rightSide = end - 1; // end

    // partition indexes of triangles based on the centroids
    while (leftSide <= rightSide) {
        const Vec3f& centroid = triangles[indiceList[leftSide]].centroid();

        if (
            (longestAxis == x && centroid.x < midPlane.x) ||
            (longestAxis == y && centroid.y < midPlane.y) ||
            (longestAxis == z && centroid.z < midPlane.z)
            ) {
            leftSide++; // triangle belongs in the left side partition, increase left side size
        }
        else {
            std::swap(indiceList[leftSide], indiceList[rightSide]); //  triangle belongs to the right, swap with one from the right side
            rightSide--; // reduce right side partition size 
        }
    }
    // leftSide contains the index of first element in right side partition = midpoint
    mid = leftSide;

    // in case of infinite loop by bad split
    if (mid == start || mid == end) {
        mid = start + (end - start) / 2;
    }
}

void RayTracer::constructBvh(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, BvhNode& node, uint32_t start, uint32_t end) {

    if (node.bb.area() == 0.0f) node.bb = computeBB(triangles, indiceList, start, end); // bounding box of the node
    node.left = nullptr;
    node.right = nullptr;
    uint32_t triCount = end - start;
    if (triCount > 6) { // this leaf node triangle size proved to be ok
        uint32_t mid;
        // in this case only split by spacial median
        partitionPrimitives(triangles, indiceList, start, end, mid, node.bb);

        node.left = std::make_unique<BvhNode>();
        node.right = std::make_unique<BvhNode>();

        constructBvh(triangles, indiceList, *node.left, start, mid);
        constructBvh(triangles, indiceList, *node.right, mid, end);
    }
    else {
        node.startPrim = start;
        node.endPrim = end;
    }
      

}

void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles)
{
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
	//m_rt->loadHierarchy(filename, triangles);
	//m_triangles = &triangles;

    std::ifstream ifs(filename, std::ios::binary);
    m_bvh = Bvh(ifs);
    m_triangles = &triangles;
}

void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
    //m_rt->saveHierarchy(filename, triangles);
    (void)triangles; // Not used.

    std::ofstream ofs(filename, std::ios::binary);
    m_bvh.save(ofs);
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
	//m_rt->constructHierarchy(triangles, splitMode);
	//m_triangles = &triangles;
 
    // check if bvh has been initalized in this scene
    m_triangles = &triangles;
    uint32_t start = 0;
    uint32_t end = m_triangles->size(); // index of "last" triangle in the scene
    m_bvh = Bvh(splitMode, start, end);

    // then call the actual recursive builder with the root node
    constructBvh(triangles, m_bvh.getIndices(), m_bvh.root(), start, end);
}

// check if there is intersection between ray and bounding box
bool RayTracer::rayBBIntersect(const Vec3f& orig, const Vec3f& dir, BvhNode& node, float& t_start) {

    // bounding box points
    Vec3f minP = node.bb.min;
    Vec3f maxP = node.bb.max;

    // parallel check
    if (dir.x == 0.0f) {
        if (orig.x < minP.x || orig.x > maxP.x) { return false; }
    }

    if (dir.y == 0.0f) {
        if (orig.y < minP.y || orig.y > maxP.y) { return false; }
    }

    if (dir.z == 0.0f) {
        if (orig.z < minP.z || orig.z > maxP.z) { return false; }
    }

    Vec3f precompute = Vec3f(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);

    // x interval
    float t1_x = (minP.x - orig.x) * precompute.x; // t min x
    float t2_x = (maxP.x - orig.x) * precompute.x;
    if (t1_x > t2_x) std::swap(t1_x, t2_x);

    // y interval
    float t1_y = (minP.y - orig.y) * precompute.y;
    float t2_y = (maxP.y - orig.y) * precompute.y;
    if (t1_y > t2_y) std::swap(t1_y, t2_y);

    // z interval
    float t1_z = (minP.z - orig.z) * precompute.z;
    float t2_z = (maxP.z - orig.z) * precompute.z;
    if (t1_z > t2_z) std::swap(t1_z, t2_z);

    // final intersect interval: max of starts, min of ends
    t_start = std::max(std::max(t1_x, t1_y), t1_z);
    float t_end = std::min(std::min(t2_x, t2_y), t2_z);

    // box is missed?
    if (t_start > t_end) {
        return false;
    }
    // box is behind?
    if (t_end < 0) {
        return false;
    }

    return true;
}

RaycastResult RayTracer::traverseBvh(const Vec3f& orig, const Vec3f& dir, const BvhNode& node) {
    RaycastResult castresult;

    // check if bb has child bbs
    if (node.hasChildren()) {
        RaycastResult resultLeft, resultRight;
        float tStartLeft, tStartRight;

        // check children bb ray intersect, return intersect starting points
        bool intersectLeft = rayBBIntersect(orig, dir, *node.left, tStartLeft);
        bool intersectRight = rayBBIntersect(orig, dir, *node.right, tStartRight);

        if (intersectLeft && intersectRight) {  // both child node bbs hit
            if (tStartLeft < tStartRight) { // left closer, traverse it first
                resultLeft = traverseBvh(orig, dir, *node.left);
                if (resultLeft.tri != nullptr && resultLeft.t < tStartRight) { // if left child hit triangle is closer than right child bb -> skip right child, return result
                    return resultLeft;
                }
                resultRight = traverseBvh(orig, dir, *node.right);
            }
            else { // traverse right
                resultRight = traverseBvh(orig, dir, *node.right);
                if (resultRight.tri != nullptr && resultRight.t < tStartLeft) {
                    return resultRight;
                }
                resultLeft = traverseBvh(orig, dir, *node.left);
            }
        }
        else if (intersectLeft) { // intersect only left child
            resultLeft = traverseBvh(orig, dir, *node.left);
        }
        else if (intersectRight) {
            resultRight = traverseBvh(orig, dir, *node.right);
        }
        else { // no intersect for children
            return castresult;
        }

        if ((resultLeft.tri == nullptr) && (resultRight.tri == nullptr)) { // no hit
            return castresult;
        }

        if (resultLeft.tri == nullptr) return resultRight; // hit from only right, return it
        if (resultRight.tri == nullptr) return resultLeft; // only left

        // both hit, return closer
        return (resultLeft.t < resultRight.t) ? resultLeft : resultRight;
    }
    else {
        // leaf node, check intersection between triangles in this node
        float closest_t = 1.0f, closest_u = 0.0f, closest_v = 0.0f;
        int closest_i = -1;

        // loop through the triangles in the node
        const std::vector<uint32_t>& indices = m_bvh.getIndices();
        for (uint32_t i = node.startPrim; i < node.endPrim; ++i) {
            float t, u, v;
            const RTTriangle& tri = (*m_triangles)[indices[i]];
            if (tri.intersect_woop(orig, dir, t, u, v))
            {
                if (t > 0.0f && t < closest_t)
                {
                    closest_i = indices[i];
                    closest_t = t;
                    closest_u = u;
                    closest_v = v;
                }
            }
        }
        if (closest_i != -1) {
            castresult = RaycastResult(&(*m_triangles)[closest_i], closest_t, closest_u, closest_v, orig + closest_t * dir, orig, dir);
        }
        return castresult;
    }
}


RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) {
	++m_rayCount;

    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
    //return m_rt->raycast(orig, dir);

    RaycastResult castresult;
    castresult = traverseBvh(orig, dir, m_bvh.root());

    return castresult;
}


} // namespace FW