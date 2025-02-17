#define _CRT_SECURE_NO_WARNINGS

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "RayTracer.hpp"
#include <stdio.h>
#include "rtIntersect.inl"
#include <fstream>

#include "rtlib.hpp"

#include "BvhNode.hpp"
#include "rtutil.hpp"
#include <algorithm>
#include <cmath>

// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer( void* buffer, size_t bufLen, unsigned int* pDigest );


namespace FW
{


Vec2f getTexelCoords(Vec2f uv, const Vec2i size)
{

	// YOUR CODE HERE (R3):
	// Get texel indices of texel nearest to the uv vector. Used in texturing.
	// UV coordinates range from negative to positive infinity. First map them
	// to a range between 0 and 1 in order to support tiling textures, then
	// scale the coordinates by image resolution and find the nearest pixel.

    // sigmoid function
    float mappedX;
    float mappedY;
    //std::cout << "before: x: " << uv.x << ", y: " << uv.y << std::endl;

    if (uv.x < 0) {
        mappedX = -1.0f * uv.x - std::floor(-1.0f*uv.x);
    }
    else {
        mappedX = uv.x - std::floor(uv.x);
    }
    if (uv.y < 0) {

        mappedY = -1.0f * uv.y - std::floor(-1.0f*uv.y);
    }
    else {
        mappedY = uv.y - std::floor(uv.y);
    }
    //std::cout << "x: " << mappedX << ", y: " << mappedY << std::endl;

	return Vec2f(mappedX * size.x, mappedY * size.y);
}

Mat3f formBasis(const Vec3f& n) {
    // YOUR CODE HERE (R4):
    return Mat3f();
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
}

RayTracer::~RayTracer()
{
}


void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles)
{
	std::ifstream ifs(filename, std::ios::binary);
    m_bvh = Bvh(ifs);
    m_triangles = &triangles;
}

void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
	(void)triangles; // Not used.

	std::ofstream ofs(filename, std::ios::binary);
	m_bvh.save(ofs);
}

// create the axis-aligned bounding box for the group of objects
AABB computeBB(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, uint32_t start, uint32_t end) {

    if (start >= end || start >= indiceList.size() || end > indiceList.size() || indiceList.empty()) {
        return AABB(Vec3f(0, 0, 0), Vec3f(0, 0, 0));
    }

    // initalize min point and max point of bb by first triangle
    if (indiceList[start] >= triangles.size()) {
        return AABB(Vec3f(0, 0, 0), Vec3f(0, 0, 0));
    }
    
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
    if (start >= end || start >= indiceList.size() || end > indiceList.size() || indiceList.empty()) {
        // Return an empty AABB or throw an exception
        return;
    }
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
            std::swap(indiceList[leftSide], indiceList[rightSide]); //  triangle belons to the right, swap with one from the right side
            rightSide--; // reduce right side partition size 
        }
    }
    // leftSide contains the index of first element in right side partition, midpoint
    mid = leftSide;

    // in case of infinite loop by bad split
    if (mid == start || mid == end) {
        mid = start + (end - start) / 2; 
    }
}


void RayTracer:: constructBvh(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, BvhNode& node, uint32_t start, uint32_t end) {
    node.bb = computeBB(triangles, indiceList, start, end); // bounding box of the node
    node.left = nullptr;
    node.right = nullptr;
    uint32_t triCount = end - start;
    if (triCount > 3) { // TODO check the minimum size later

        // Instead of sorting the triangle list when building your hierarchy,
        // you should sort the index list, through which your Bvh traversal code should access the triangles.

        uint32_t mid;
        partitionPrimitives(triangles, indiceList, start,end, mid, node.bb);

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


void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
    // YOUR CODE HERE (R1):
    // This is where you should construct your BVH.
    // check if bvh has been initalized in raytracer
    
    if (!m_bvhInitalized) {
        // if not, create initial bvh
        m_triangles = &triangles;
        uint32_t start = 0;
        uint32_t end = m_triangles->size(); // index of "last" triangle in the scene TODO -1 ???
        m_bvh = Bvh(splitMode, start, end);
        m_bvhInitalized = true;

        // then call the actual recursive builder with the root node
        constructBvh(triangles, m_bvh.getIndices(), m_bvh.root(), start, end);
    }
}

// traverse through the BVH and recursively check for intersections 
RaycastResult RayTracer::traverseBvh(const Vec3f& orig, const Vec3f& dir, BvhNode& node) {
    RaycastResult castresult;

    // first check if ray intersects bb
    if (!rayBBIntersect(orig, dir, node)) {
        return castresult;
    }

    if (node.hasChildren()) {   
        // intersect the bb, traverse children
        RaycastResult resultLeft = traverseBvh(orig, dir, *node.left);
        RaycastResult resultRight = traverseBvh(orig, dir, *node.right);

        // return closer
        return (resultLeft.t < resultRight.t) ? resultLeft : resultRight;
    }
    else {
        // leaf node, check intersection between triangles in this node
        float closest_t = 1.0f, closest_u = 0.0f, closest_v = 0.0f;
        int closest_i = -1;
        uint32_t i = 0;

        // loop through the triangles in the child node
        const std::vector<uint32_t>& indices = m_bvh.getIndices();
        for (uint32_t i = node.startPrim; i < node.endPrim; ++i) { // TODO check if faster to call start, end once in the beginning
            // TODO check if intersect between the bb?  Ax+By+Cz+D
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

// check if there is intersection between ray and bounding box
bool RayTracer::rayBBIntersect(const Vec3f& orig, const Vec3f& dir, BvhNode& node) {
    
    // bounding box points
    Vec3f minP = node.bb.min;
    Vec3f maxP = node.bb.max;

    // parallel check
    if (dir.x == 0.0f) {
        if (orig.x < minP.x || orig.x > maxP.x) {return false;}
    }

    if (dir.y == 0.0f) {
        if (orig.y < minP.y || orig.y > maxP.y) {return false;}
    }

    if (dir.z == 0.0f) {
        if (orig.z < minP.z || orig.z > maxP.z) {return false;}
    }

    // x interval
    float t1_x = (minP.x - orig.x) / dir.x;
    float t2_x = (maxP.x - orig.x) / dir.x;
    if (t1_x > t2_x) std::swap(t1_x, t2_x);
    
    // y interval
    float t1_y = (minP.y - orig.y) / dir.y;
    float t2_y = (maxP.y - orig.y) / dir.y;
    if (t1_y > t2_y) std::swap(t1_y, t2_y);

    // z interval
    float t1_z = (minP.z - orig.z) / dir.z;
    float t2_z = (maxP.z - orig.z) / dir.z;
    if (t1_z > t2_z) std::swap(t1_z, t2_z);

    // final intersect interval: max of starts, min of ends
    float t_start = std::max(std::max(t1_x, t1_y), t1_z);
    float t_end = std::min(std::min(t2_x, t2_y), t2_z);
    
    // TODO check if we are behind box

    // box is missed?
    if (t_start > t_end) {
        return false;
    }

    return true;
}

RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) /*const*/ {  // TODO check the const!
	++m_rayCount;

    // YOUR CODE HERE (R1):
    // This is where you traverse the tree you built! It's probably easiest
    // to introduce another function above that does the actual traversal, and
    // use this function only to begin the recursion by calling the traversal
    // function with the given ray and your root node. You can also use this
    // function to do one-off things per ray like finding the elementwise
    // reciprocal of the ray direction.

    RaycastResult castresult;
    castresult =  traverseBvh(orig, dir, m_bvh.root());

    return castresult;
}
} // namespace FW

    // You can use this existing code for leaf nodes of the BVH (you do want to
    // change the range of the loop to match the elements the leaf covers.)

    /*
    float closest_t = 1.0f, closest_u = 0.0f, closest_v = 0.0f;
    int closest_i = -1;



    RaycastResult castresult;

    // Naive loop over all triangles; this will give you the correct results,
    // but is terribly slow when ran for all triangles for each ray. Try it.
    for ( int i = 0; i < m_triangles->size(); ++i )
    {
        float t, u, v;
        if ( (*m_triangles)[i].intersect_woop( orig, dir, t, u, v ) )
        {
            if ( t > 0.0f && t < closest_t)
            {
                closest_i = i;
                closest_t = t;
                closest_u = u;
                closest_v = v;
            }
        }
    }

    if (closest_i != -1)
        castresult = RaycastResult(&(*m_triangles)[closest_i], closest_t, closest_u, closest_v, orig + closest_t *dir, orig, dir);

    */