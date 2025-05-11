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
    //return rtlib::formBasis(n);
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


// Partition with Spatial median
void RayTracer::partitionSpatialMedian(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, uint32_t start, uint32_t end, uint32_t& mid, AABB bb) {
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


// Partition with Surface Area Heurestic
void RayTracer::partitionSAH(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, uint32_t start, uint32_t end, uint32_t& mid, AABB bb, AABB& lBB, AABB& rBB) {
    if (end - start <= 1) {
        mid = start;
        lBB = rBB = bb;
        return;
    }

    // initialize values
    std::vector<float> splitRatios = { 0.10f, 0.20f, 0.30f, 0.40f, 0.5f, 0.60f, 0.70f, 0.80f, 0.90f };
    float minimalSah = std::numeric_limits<float>::max();
    uint32_t sahMid = start;
    int sahAxis = 0;

    // initialize best child BBs and best sorted indice list
    AABB bestLeftBB, bestRightBB;
    std::vector<uint32_t> bestIndiceList = indiceList;

    for (int axis = 0; axis < 3; ++axis) {
        for (float ratio : splitRatios) {
            // create split plane
            Vec3f splitPlane = bb.min + ((bb.max - bb.min) * ratio);
            float splitValue = (axis == 0) ? splitPlane.x : ((axis == 1) ? splitPlane.y : splitPlane.z);

            uint32_t left = start;
            uint32_t right = end - 1;

            // partition list based on primitive centroid and split plane
            while (left <= right) {
                const Vec3f& centroid = triangles[indiceList[left]].centroid();
                if (centroid[axis] < splitValue) {
                    left++;
                }
                else {
                    std::swap(indiceList[left], indiceList[right]);
                    if (right > 0) {
                        right--;
                    }
                    else {
                        break;
                    }
                }
            }

            // mid index of this partition
            uint32_t currentMid = left;

            // if all primitives in one side, skip
            if (currentMid == start || currentMid == end) {
                continue;
            }

            // inititalize left, right BBs
            AABB leftBB, rightBB;
            leftBB.min = rightBB.min = Vec3f(std::numeric_limits<float>::max());
            leftBB.max = rightBB.max = Vec3f(std::numeric_limits<float>::lowest());

            // create tight left child BB
            for (uint32_t i = start; i < currentMid; ++i) {
                const RTTriangle& tri = triangles[indiceList[i]];
                for (int v = 0; v < 3; ++v) {
                    leftBB.min = min(leftBB.min, tri.m_vertices[v].p);
                    leftBB.max = max(leftBB.max, tri.m_vertices[v].p);
                }
            }

            // create tight right child BB
            for (uint32_t i = currentMid; i < end; ++i) {
                const RTTriangle& tri = triangles[indiceList[i]];
                for (int v = 0; v < 3; ++v) {
                    rightBB.min = min(rightBB.min, tri.m_vertices[v].p);
                    rightBB.max = max(rightBB.max, tri.m_vertices[v].p);
                }
            }

            // calculate sah = A_left * n_left + A_right * n_right
            int nLeft = currentMid - start;
            int nRight = end - currentMid;
            float sah = leftBB.area() * nLeft + rightBB.area() * nRight;

            // keep if lower cost than current
            if (sah < minimalSah) {
                minimalSah = sah;
                sahMid = currentMid;
                sahAxis = axis;
                bestLeftBB = leftBB;
                bestRightBB = rightBB;
                bestIndiceList = indiceList;
            }
        }
    }

    // if only bad splits, make this a leaf node
    if (sahMid == start || sahMid == end) {
        mid = start;
        lBB = rBB = bb;
        return;
    }

    // return final best values
    indiceList = bestIndiceList;
    mid = sahMid;
    lBB = bestLeftBB;
    rBB = bestRightBB;
}


void RayTracer::constructBvh(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, BvhNode& node, uint32_t start, uint32_t end) {

    if (node.bb.area() == 0.0f) node.bb = computeBB(triangles, indiceList, start, end); // bounding box of the node
    node.left = nullptr;
    node.right = nullptr;
    uint32_t triCount = end - start;

    switch (m_bvh.splitMode()) {
    case SplitMode_SpatialMedian:
        if (triCount > 100) { // this leaf node triangle size proved to be ok
            //node.bb = computeBB(triangles, indiceList, start, end); // bounding box of the node
            uint32_t mid;
            // in this case only split by spacial median
            partitionSpatialMedian(triangles, indiceList, start, end, mid, node.bb);

            node.left = std::make_unique<BvhNode>();
            node.right = std::make_unique<BvhNode>();

            constructBvh(triangles, indiceList, *node.left, start, mid);
            constructBvh(triangles, indiceList, *node.right, mid, end);
        }
        else {
            node.startPrim = start;
            node.endPrim = end;
        }
        break;
    case SplitMode_Sah:
        if (triCount > 1000) {

            uint32_t mid;
            AABB lBB, rBB;
            // SAH partition

            //std::cout << "before: start: " << start << ", end: " << end;
            partitionSAH(triangles, indiceList, start, end, mid, node.bb, lBB, rBB);
            //std::cout << "after: start: " << start << ", mid: " << mid;

            // if bad split, stop recursion
            if (mid == start) {
                node.startPrim = start;
                node.endPrim = end;
            }
            else {
                node.left = std::make_unique<BvhNode>();
                node.right = std::make_unique<BvhNode>();

                node.left->bb = lBB;
                node.right->bb = rBB;

                constructBvh(triangles, indiceList, *node.left, start, mid);
                constructBvh(triangles, indiceList, *node.right, mid, end);
            }
        }
        else {
            node.startPrim = start;
            node.endPrim = end;
        }
        break;
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

    std::ofstream ofs(filename, std::ios::binary);
    m_bvh.save(ofs);
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
    // YOUR CODE HERE (R1):
    // Integrate your implementation here.
	//m_rt->constructHierarchy(triangles, splitMode);
	//m_triangles = &triangles;

    m_triangles = &triangles;
    uint32_t start = 0;
    uint32_t end = m_triangles->size(); // index of "last" triangle in the scene
    m_bvh = Bvh(splitMode, start, end);
    //m_bvh = Bvh(SplitMode_SpatialMedian, start, end);

    // then call the actual recursive builder with the root node
    constructBvh(triangles, m_bvh.getIndices(), m_bvh.root(), start, end);
}


// check if there is intersection between ray and bounding box
bool RayTracer::rayBBIntersect(const Vec3f& orig, const Vec3f& inverseDir, BvhNode& node, float& t_start) {

    // bounding box points
    Vec3f minP = node.bb.min;
    Vec3f maxP = node.bb.max;

    float t1_x = (minP.x - orig.x) * inverseDir.x;
    float t2_x = (maxP.x - orig.x) * inverseDir.x;
    float t_min_x = std::min(t1_x, t2_x);
    float t_max_x = std::max(t1_x, t2_x);

    float t1_y = (minP.y - orig.y) * inverseDir.y;
    float t2_y = (maxP.y - orig.y) * inverseDir.y;
    float t_min_y = std::min(t1_y, t2_y);
    float t_max_y = std::max(t1_y, t2_y);

    float t1_z = (minP.z - orig.z) * inverseDir.z;
    float t2_z = (maxP.z - orig.z) * inverseDir.z;
    float t_min_z = std::min(t1_z, t2_z);
    float t_max_z = std::max(t1_z, t2_z);

    t_start = std::max({ t_min_x, t_min_y, t_min_z });
    float t_end = std::min({ t_max_x, t_max_y, t_max_z });

    return !(t_start > t_end || t_end < 0);
}


RaycastResult RayTracer::traverseBvh(const Vec3f& orig, const Vec3f& dir, const Vec3f& inverseDir, const BvhNode& node) {
    RaycastResult castresult;

    // check if bb has child bbs
    if (node.hasChildren()) {
        RaycastResult resultLeft, resultRight;
        float tStartLeft, tStartRight;

        // check children bb ray intersect, return intersect starting points
        bool intersectLeft = rayBBIntersect(orig, inverseDir, *node.left, tStartLeft);
        bool intersectRight = rayBBIntersect(orig, inverseDir, *node.right, tStartRight);

        if (intersectLeft && intersectRight) {  // both child node bbs hit
            if (tStartLeft < tStartRight) { // left closer, traverse it first
                resultLeft = traverseBvh(orig, dir, inverseDir, *node.left);
                if (resultLeft.tri != nullptr && resultLeft.t < tStartRight) { // if left child hit triangle is closer than right child bb -> skip right child, return result
                    return resultLeft;
                }
                resultRight = traverseBvh(orig, dir, inverseDir, *node.right);
            }
            else { // traverse right
                resultRight = traverseBvh(orig, dir, inverseDir, *node.right);
                if (resultRight.tri != nullptr && resultRight.t < tStartLeft) {
                    return resultRight;
                }
                resultLeft = traverseBvh(orig, dir, inverseDir, *node.left);
            }
        }
        else if (intersectLeft) { // intersect only left child
            resultLeft = traverseBvh(orig, dir, inverseDir, *node.left);
        }
        else if (intersectRight) {
            resultRight = traverseBvh(orig, dir, inverseDir, *node.right);
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
        float closest_t = std::numeric_limits<float>::max(), closest_u = 0.0f, closest_v = 0.0f;
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

    // precompute dir before recursive BVH traversing
    Vec3f inverseDir = Vec3f(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);

    RaycastResult castresult;
    castresult = traverseBvh(orig, dir, inverseDir, m_bvh.root());

    return castresult;
}


} // namespace FW