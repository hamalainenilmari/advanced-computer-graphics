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
    
	return Vec2f();
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
AABB computeBB(std::vector<RTTriangle>& triangles) {
    //std::cout << "num triangles in bb compute: " << triangles.size() << std::endl;
    // compute bounding box of each triangle
    std::vector<Vec3f> mins;
    std::vector<Vec3f> maxs;
    for (RTTriangle& triangle : triangles) {
        // the lower left corner of triangles BB
        mins.push_back(triangle.min());
        // the upper right corner
        maxs.push_back(triangle.max());
    }
    // compute bounding box for the group of objects from the min of mins and max maxs
    float minX = std::min_element(mins.begin(), mins.end(), [](const Vec3f& a, const Vec3f& b) { return a.x < b.x; })->x;
    float minY = std::min_element(mins.begin(), mins.end(), [](const Vec3f& a, const Vec3f& b) { return a.y < b.y; })->y;
    float minZ = std::min_element(mins.begin(), mins.end(), [](const Vec3f& a, const Vec3f& b) { return a.z < b.z; })->z;

    float maxX = std::max_element(maxs.begin(), maxs.end(), [](const Vec3f& a, const Vec3f& b) { return a.x < b.x; })->x;
    float maxY = std::max_element(maxs.begin(), maxs.end(), [](const Vec3f& a, const Vec3f& b) { return a.y < b.y; })->y;
    float maxZ = std::max_element(maxs.begin(), maxs.end(), [](const Vec3f& a, const Vec3f& b) { return a.z < b.z; })->z;

    AABB bb(Vec3f(minX, minY, minZ), Vec3f(maxX, maxY, maxZ));
    return bb;
}

void partitionPrimitives(std::vector<RTTriangle>& triangles, std::vector<RTTriangle>& left, std::vector<RTTriangle>& right, AABB bb) {
    // AABB of centroids to create actual split
    std::vector<Vec3f> centroids;
    for (const RTTriangle& triangle : triangles) {
        centroids.push_back(triangle.centroid());
    }
    Vec3f centroidMin = centroids[0];
    Vec3f centroidMax = centroids[0];
    for (const Vec3f& centroid : centroids) {
        centroidMin.x = std::min(centroidMin.x, centroid.x);
        centroidMin.y = std::min(centroidMin.y, centroid.y);
        centroidMin.z = std::min(centroidMin.z, centroid.z);

        centroidMax.x = std::max(centroidMax.x, centroid.x);
        centroidMax.y = std::max(centroidMax.y, centroid.y);
        centroidMax.z = std::max(centroidMax.z, centroid.z);
    }

    // longest axis
    float x = centroidMax.x - centroidMin.x;
    float y = centroidMax.y - centroidMin.y;
    float z = centroidMax.z - centroidMin.z;
    float max = std::max({ x, y, z });

    //std::cout << "x: " << x << " y: " << y << " z: " << z << " max: " << max << std::endl;

    float midX = (centroidMin.x + centroidMax.x) / 2;
    float midY = (centroidMin.y + centroidMax.y) / 2;
    float midZ = (centroidMin.z + centroidMax.z) / 2;

    if (max == x) {
        //std::cout << "we in x" << std::endl;
        for (RTTriangle& triangle : triangles) {
            // if triangle centroid is on the left side of the bb divided by half in x axis, add to left list
            if (triangle.centroid().x < midX) {
                left.push_back(triangle);
            }
            // else add to right
            else {
                right.push_back(triangle);
            }
        }
    }
    else if (max == y) {
        //std::cout << "we in y" << std::endl;

        for (RTTriangle& triangle : triangles) {
            // if triangle centroid is on the left side of the bb divided by half in x axis, add to left list
            if (triangle.centroid().y < midY) {
                left.push_back(triangle);
            }
            // else add to right
            else {
                right.push_back(triangle);
            }
        }
    }
    else {
        //std::cout << "we in z" << std::endl;

        for (RTTriangle& triangle : triangles) {
            // if triangle centroid is on the left side of the bb divided by half in x axis, add to left list
            //std::cout << "z: " << z << " midZ: " << midZ << std::endl;
            if (triangle.centroid().z < midZ) {
                left.push_back(triangle);
            }
            // else add to right
            else {
                right.push_back(triangle);
            }
        }
    }
    std::cout << "Triangles: " << triangles.size() << ", Left: " << left.size() << ", Right: " << right.size() << std::endl;
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode, BvhNode& node) { // we need list of all primitives in the scene & root node
    // YOUR CODE HERE (R1):
    // This is where you should construct your BVH.

    // check if bvh has been initalized to raytracerccs
    if (!m_bvhInitalized) {
        m_triangles = &triangles;

        //std::cout << "not initalized, initalizins" << std::endl;
        // if not, create initial bvh
        size_t start = 0;
        size_t end = m_triangles->size() - 1; // index of "last" triangle in the scene

        m_bvh = Bvh(splitMode, start, end);
        m_bvhInitalized = true;
        constructHierarchy(triangles, splitMode, m_bvh.root());

    }
    //std::cout << "recursion" << std::endl;

    //m_triangles = &triangles;

    node.bb = computeBB(triangles); // bounding box of the node
    //std::cout << "computed bb: " << node.bb << " for list size: " << triangles.size() << std::endl;
    node.left = NULL;
    node.right = NULL;

    if (triangles.size() > 3) { // TODO check the size later
        // partition the list of indexes of triangles into two lists of indexes
        std::vector<RTTriangle> leftChildList;
        std::vector<RTTriangle> rightChildList;
            
        partitionPrimitives(triangles, leftChildList, rightChildList, node.bb);

        node.left = std::make_unique<BvhNode>();
        constructHierarchy(leftChildList, splitMode, *node.left);

        node.right = std::make_unique<BvhNode>();
        constructHierarchy(rightChildList, splitMode, *node.right);
    }
    else {
        return;
        //node.startPrim = &triangles.front();
        //node.endPrim = &triangles.back();
    }

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
    
    //SplitMode split = SplitMode_SpatialMedian;

    //size_t start = 0;
    //size_t end = m_triangles->size() - 1; // index of "last" triangle in the scene

    //Bvh* bvh = new Bvh(split, start, end);
    //BvhNode& root = bvh->root();
 
    //constructHierarchy(*m_triangles, split, root);

    // You can use this existing code for leaf nodes of the BVH (you do want to
    // change the range of the loop to match the elements the leaf covers.)
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
    
    return castresult;
}


} // namespace FW