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

// Surface Area Heurestic
void partitionSAH(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, uint32_t start, uint32_t end, uint32_t& mid, AABB bb, AABB& lBB, AABB& rBB) {

    // pick planes at uniform intervals from the extent of the parent AABB along all three axis
    // this will give 3 different splits

    Vec3f midPlane1 = bb.min + ((bb.max - bb.min) * 0.15f);
    Vec3f midPlane2 = bb.min + ((bb.max - bb.min) * 0.30f);
    Vec3f midPlane3 = bb.min + ((bb.max - bb.min) * 0.45f);
    Vec3f midPlane4 = bb.min + ((bb.max - bb.min) * 0.60f);
    Vec3f midPlane5 = bb.min + ((bb.max - bb.min) * 0.75f);
    Vec3f midPlane6 = bb.min + ((bb.max - bb.min) * 0.90f);

    // array of each split
    std::vector<Vec3f> splitPlanes = { midPlane1, midPlane2, midPlane3, midPlane4, midPlane5, midPlane6 };
    /*
    std::cout << "----------------------------" << std::endl;
    std::cout << "new partition: " << std::endl;
    std::cout << "left: " << start << ", right: " << end << std::endl;
    std::cout << "bb: " << bb << std::endl;

    Vec3f midPlane2 = bb.min + ((bb.max - bb.min) * 0.25f);
    Vec3f midPlane3 = bb.min + ((bb.max - bb.min) * 0.50f);
    Vec3f midPlane4 = bb.min + ((bb.max - bb.min) * 0.75f);
    */
    //std::vector<Vec3f> splitPlanes = { midPlane2, midPlane3, midPlane4};

    // initialize the sah
    float minimalSah = std::numeric_limits<float>::max();
    uint32_t sahMid = start;
    Vec3f sahPlane;
    int sahAxis;

    for (int axis = 0; axis < 3; ++axis) { // each axis like in spacial median
        for (const Vec3f& splitPlane : splitPlanes) { // each plane
            //std::cout << "---------------------------------" << std::endl;

            //std::cout << "axis: " << axis << ", plane: " << splitPlane << std::endl;

            auto leftSide = start; // index of starting primitive
            auto rightSide = end - 1; // index of ending primitive

            // partition the primitives to the new splits

            // create copy out of the original index list
            std::vector<uint32_t> copiedIndiceList = indiceList;

            while (leftSide <= rightSide) {
                const Vec3f& centroid = triangles[copiedIndiceList[leftSide]].centroid();
                if ((axis == 0 && centroid.x < splitPlane.x) ||
                    (axis == 1 && centroid.y < splitPlane.y) ||
                    (axis == 2 && centroid.z < splitPlane.z)
                    ) {
                    leftSide++; // triangle belongs in the left side partition, move left index
                }
                else {
                    if (rightSide == 0) break;
                    std::swap(copiedIndiceList[leftSide], copiedIndiceList[rightSide]);
                    rightSide--;

                }
            }

            // leftSide contains the index of first element in right side partition = midpoint
            uint32_t currentPartitionMid = leftSide;


            // in case of infinite loop by bad split
            if (currentPartitionMid == start || currentPartitionMid == end) { //  TODO check thiz
                currentPartitionMid = (start + (end - start)) / 2;; // currentPartitionMid = start + (end - start) / 2;
            }

            AABB leftBB = bb;
            AABB rightBB = bb;

            // create child bbs from parent divided by plane
            if (axis == 0) {
                leftBB.max.x = splitPlane.x;
                rightBB.min.x = splitPlane.x;
            }
            else if (axis == 1) {
                leftBB.max.y = splitPlane.y;
                rightBB.min.y = splitPlane.y;
            }
            else {
                leftBB.max.z = splitPlane.z;
                rightBB.min.z = splitPlane.z;
            }

            // Compute the heuristic for each plane: SAH(i) = AL*NL + AR*NR,
            int numNodesRight = (end - currentPartitionMid);
            int numNodesLeft = (currentPartitionMid - start);
            //std::cout << "num triangles left " << numNodesLeft << ", right: " << numNodesRight << std::endl;
            float currentSah = leftBB.area() * numNodesLeft + rightBB.area() * numNodesRight;
            //std::cout << "sah: " << currentSah << std::endl;
            // keep the lowest sah
            if (currentSah < minimalSah) {


                minimalSah = currentSah;
                sahMid = currentPartitionMid;
                sahPlane = splitPlane;
                sahAxis = axis;
            }
        }
    }



    //std::cout << "axis: " << sahAxis << ", plane: " << sahPlane << ", mid: " << sahMid << std::endl;

    /*
    for (uint32_t x = 0; x < indiceList.size(); ++x) {
        std::cout << "index: " << indiceList[x] << ", centroid :" << triangles[indiceList[x]].centroid() << std::endl;
    }

    std::cout << "### sorted ###" << std::endl;
    */
    // after we have the correct plane and axis, partition the original index list based on it
    auto leftSide = start; // index of starting primitive
    auto rightSide = end - 1; // index of ending primitive
    while (leftSide <= rightSide) {
        const Vec3f& centroid = triangles[indiceList[leftSide]].centroid();
        if ((sahAxis == 0 && centroid.x < sahPlane.x) ||
            (sahAxis == 1 && centroid.y < sahPlane.y) ||
            (sahAxis == 2 && centroid.z < sahPlane.z)
            ) {
            leftSide++; // triangle belongs in the left side partition, move left index
        }
        else {
            if (rightSide == 0) break;
            std::swap(indiceList[leftSide], indiceList[rightSide]);
            rightSide--;

        }
    }
    /*

    for (uint32_t x = 0; x < indiceList.size(); ++x) {
        std::cout << "index: " << indiceList[x] << ", centroid :" << triangles[indiceList[x]].centroid() << std::endl;
    }
    */



    lBB = bb;
    rBB = bb;

    // create child bbs from parent divided by plane
    if (sahAxis == 0) {
        lBB.max.x = sahPlane.x;
        rBB.min.x = sahPlane.x;
    }
    else if (sahAxis == 1) {
        lBB.max.y = sahPlane.y;
        rBB.min.y = sahPlane.y;
    }
    else {
        lBB.max.z = sahPlane.z;
        rBB.min.z = sahPlane.z;
    }
    //std::cout << "bb L: " << lBB << "bb R: " << rBB << std::endl;
    mid = sahMid;
    if (mid == start || mid == end) {
        mid = start + (end - start) / 2;
    }

}


void RayTracer::constructBvh(std::vector<RTTriangle>& triangles, std::vector<uint32_t>& indiceList, BvhNode& node, uint32_t start, uint32_t end) {
    //std::cout << "start: " << start << ", end: " << end << std::endl;

    if (node.bb.area() == 0.0f) node.bb = computeBB(triangles, indiceList, start, end); // bounding box of the node
    node.left = nullptr;
    node.right = nullptr;
    uint32_t triCount = end - start;

    switch (m_bvh.splitMode()) {
    case SplitMode_SpatialMedian:
        if (triCount > 6) { // this leaf node triangle size proved to be ok
            //node.bb = computeBB(triangles, indiceList, start, end); // bounding box of the node
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
        break;
    case SplitMode_Sah:
        if (triCount > 6) {

            uint32_t mid;
            AABB lBB, rBB;
            // SAH partition
            partitionSAH(triangles, indiceList, start, end, mid, node.bb, lBB, rBB);
            //std::cout << "lbb: " << lBB << std::endl;
            node.left = std::make_unique<BvhNode>();
            node.right = std::make_unique<BvhNode>();

            node.left->bb = lBB;
            node.right->bb = rBB;


            constructBvh(triangles, indiceList, *node.left, start, mid);
            constructBvh(triangles, indiceList, *node.right, mid, end);
        }
        else {
            /*
            std::cout << " ------------------- " << std::endl;
            std::cout << "finished partitioning " << std::endl;
            std::cout << "primitives start, end: " << start << ", " << end << std::endl;
            */
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

    // precompute dir before recursive BVH traversing
    Vec3f inverseDir = Vec3f(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);

    RaycastResult castresult;
    castresult = traverseBvh(orig, dir, inverseDir, m_bvh.root());

    return castresult;
}


} // namespace FW