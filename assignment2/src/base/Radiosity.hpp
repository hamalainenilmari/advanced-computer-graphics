#pragma once

#include <base/Math.hpp>
#include <base/MulticoreLauncher.hpp>
#include <base/Random.hpp>
#include <3d/Mesh.hpp>

#include <vector>

namespace FW
{

class AreaLight;
class RayTracer;

float sphericalHarmonicFactor(int order, int index);

//------------------------------------------------------------------------

typedef Mesh<VertexPNTC>	MeshWithColors;

//------------------------------------------------------------------------

class Radiosity
{
public:
    Radiosity() { }
    ~Radiosity();

    // we'll compute radiosity asynchronously, meaning we'll be able to
    // fly around in the scene watching the process complete.
    void	startRadiosityProcess( MeshWithColors* scene, AreaLight* light, RayTracer* rt, int numBounces, int numDirectRays, int numHemisphereRays, bool useQMC, int bounceToRender);

    // are we still processing?
    bool	isRunning() const		{ return m_launcher.getNumTasks() > 0; }
    // sees if we need to switch bounces, etc.
    void	checkFinish();	

    // copy the current solution to the mesh colors for display. returns true if successful.
	bool	updateMeshColors(std::vector<Vec4f>& spherical1, std::vector<Vec4f>& spherical2, std::vector<float>& spherical3, bool spherical, int bounce);

protected:
    MulticoreLauncher	m_launcher;

    // This structure holds all the variables needed for computing the illumination
    // in multithreaded fashion. See Radiosity::vertexTaskFunc()
    struct RadiosityContext
    {
        RadiosityContext() : m_scene(nullptr), m_numBounces(1), m_numDirectRays(64), m_numHemisphereRays(256), m_currentBounce(0), m_bForceExit(false), m_useQMC(false) { }

        MeshWithColors*		m_scene;
        AreaLight*			m_light;
        RayTracer*			m_rt;

        int					m_numBounces;
        int					m_numDirectRays;
        int					m_numHemisphereRays;
        int					m_currentBounce;
        int                 m_bounceToRender;

        bool				m_bForceExit;
        bool                m_useQMC;

        // these are vectors with one value per vertex
        std::vector<Vec3f>	m_vecCurr;			// this one holds the results for the bounce currently being computed. Zero in the beginning.
        std::vector<Vec3f>	m_vecPrevBounce;	// Once a bounce finishes, the results are copied here to be used as the input illumination to the next bounce.
        std::vector<Vec3f>	m_vecResult;		// This vector should hold a sum of all bounces (used for display).
		std::vector<Vec3f>	m_vecSphericalC, m_vecSphericalX, m_vecSphericalY, m_vecSphericalZ;		// For the spherical harmonics extra: these hold the sums for spherical harmonic coefficients, similarly to m_vecResult.

        // Recommended extra 1: store and visualize specific bounce to render
        std::vector<Vec3f>	m_vecBounce0;  // bounce 0: direct irradiance
        std::vector<Vec3f>	m_vecBounce1;  // bounce 1: indirect
        std::vector<Vec3f>	m_vecBounce2;
        std::vector<Vec3f>	m_vecBounce3;
        std::vector<Vec3f>	m_vecBounce4;
        std::vector<Vec3f>	m_vecBounce5;
        std::vector<Vec3f>	m_vecBounce6;
        std::vector<Vec3f>	m_vecBounce7;
        std::vector<Vec3f>	m_vecBounce8;
        std::vector<Vec3f>	m_vecBounce9;


        std::vector<std::vector<Vec3f>*> m_bounceVectors;


    };

    static void vertexTaskFunc( MulticoreLauncher::Task& );

    RadiosityContext		m_context;

};

} // namepace FW
