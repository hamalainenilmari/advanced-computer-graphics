#include "Radiosity.hpp"
#include "AreaLight.hpp"
#include "RayTracer.hpp"



namespace FW {


// --------------------------------------------------------------------------

Radiosity::~Radiosity()
{
    if ( isRunning() )
    {
        m_context.m_bForceExit = true;
        while( m_launcher.getNumTasks() > m_launcher.getNumFinished() )
            Sleep( 1 );
        m_launcher.popAll();
    }
}


// --------------------------------------------------------------------------
void Radiosity::vertexTaskFunc( MulticoreLauncher::Task& task )
{
    RadiosityContext& ctx = *(RadiosityContext*)task.data;

    if( ctx.m_bForceExit )
        return;

    // which vertex are we to compute?
    int v = task.idx;

    // fetch vertex and its normal
    Vec3f n = ctx.m_scene->vertex(v).n.normalized();
    Vec3f o = ctx.m_scene->vertex(v).p + 0.01f*n;

    // YOUR CODE HERE (R3):
    // This starter code merely puts the color-coded normal into the result.
	// Remove the dummy solution to make your own implementation work.
    //
    // In the first bounce, your task is to compute the direct irradiance
    // falling on this vertex from the area light source.
    // In the subsequent passes, you should compute the irradiance by a
    // hemispherical gathering integral. The commented code below gives you
    // an idea of the loop structure. Note that you also have to account
    // for how diffuse textures modulate the irradiance.


	// This is the dummy implementation you should remove.
    ctx.m_vecResult[ v ] = n*0.5+0.5;
    Sleep(1);
    return;

    /*
    // direct lighting pass? => integrate direct illumination by shooting shadow rays to light source
    if ( ctx.m_currentBounce == 0 )
    {
        Vec3f E(0);
        for ( int r = 0; r < ctx.m_numDirectRays; ++r )
        {
            // draw sample on light source
            float pdf;
            Vec3f Pl;

            // construct vector from current vertex (o) to light sample

            // trace shadow ray to see if it's blocked
            {
                // if not, add the appropriate emission, 1/r^2 and clamped cosine terms, accounting for the PDF as well.
                // accumulate into E
            }
        }

        // The result we are computing is _irradiance_ (E), not radiosity, so no rho/pi terms.
        ctx.m_vecCurr[ v ] = E * (1.0f/ctx.m_numDirectRays);
        ctx.m_vecResult[ v ] = ctx.m_vecCurr[ v ];
    }
    else
    {
        // OK, time for indirect!
        // Implement hemispherical gathering integral for bounces > 1.

        // Get local coordinate system the rays are shot from.
        Mat3f B = formBasis( n );

        Vec3f E(0.0f);
        for ( int r = 0; r < ctx.m_numHemisphereRays; ++r )
        {
            // Draw a cosine weighted direction and find out where it hits (if anywhere)
            // You need to transform it from the local frame to the vertex' hemisphere using B.
            // Also compute the pdf (p(omega) in the handout) of the sample. We will need it for the Monte Carlo estimate

            // Make the direction long but not too long to avoid numerical instability in the ray tracer.
            // For our scenes, 100 is a good length. (I know, this special casing sucks.)

            // Shoot ray, see where we hit
            const RaycastResult result = ctx.m_rt->raycast( o, d );
            if ( result.tri != nullptr )
            {
                // interpolate lighting from previous pass
				const Vec3i& indices = result.tri->m_data.vertex_indices;

                // check for backfaces => don't accumulate if we hit a surface from below!

                // fetch barycentric coordinates

                // Ei = interpolated irradiance determined by ctx.m_vecPrevBounce from vertices using the barycentric coordinates
                Vec3f Ei = ...

                // Divide incident irradiance by PI so that we can turn it into outgoing
                // radiosity by multiplying by the reflectance factor below.
                Ei *= (1.0f / FW_PI);

                // check for texture
                const auto mat = result.tri->m_material;
                if ( mat->textures[MeshBase::TextureType_Diffuse].exists() )
                {
					
					// read diffuse texture like in assignment1

                    const Texture& tex = mat->textures[MeshBase::TextureType_Diffuse];
                    const Image& teximg = *tex.getImage();

                    // ...
                }
                else
                {
                    // no texture, use constant albedo from material structure.
                    // (this is just one line)
                }

                // We now have the radiosity Li incoming from the point we hit with the raytracer, written as Li = Ei * rho / pi
                // Finally we will need to multiply with the cosine (Equations 3,4 of the handout) and divide by the pdf, which you should have computed already
                // If you use a cosine-weighted sampler like we suggest, you will see that many terms actually cancel out. You can take this into account in your code,
                // or just implement Equation 4 of the handout as is. Both are fine and will give the same result.

                E += Ei;	// accumulate
            }
        }
        // Store result for this bounce

        ctx.m_vecCurr[ v ] = E * (1.0 / ctx.m_numHemisphereRays);
        // Also add to the global accumulator.
        ctx.m_vecResult[ v ] = ctx.m_vecResult[ v ] + ctx.m_vecCurr[ v ];

        // uncomment this to visualize only the current bounce
        //ctx.m_vecResult[ v ] = ctx.m_vecCurr[ v ];	
    }
    */
}
// --------------------------------------------------------------------------

void Radiosity::startRadiosityProcess( MeshWithColors* scene, AreaLight* light, RayTracer* rt, int numBounces, int numDirectRays, int numHemisphereRays, bool useQMC )
{
    // put stuff the asyncronous processor needs 
    m_context.m_scene				= scene;
    m_context.m_rt					= rt;
    m_context.m_light				= light;
    m_context.m_currentBounce		= 0;
    m_context.m_numBounces			= numBounces;
    m_context.m_numDirectRays		= numDirectRays;
    m_context.m_numHemisphereRays	= numHemisphereRays;
    m_context.m_useQMC              = useQMC;

    // resize all the buffers according to how many vertices we have in the scene
	m_context.m_vecResult.resize(scene->numVertices());
    m_context.m_vecCurr.resize( scene->numVertices() );
    m_context.m_vecPrevBounce.resize( scene->numVertices() );
    m_context.m_vecResult.assign( scene->numVertices(), Vec3f(0,0,0) );

	m_context.m_vecSphericalC.resize(scene->numVertices());
	m_context.m_vecSphericalX.resize(scene->numVertices());
	m_context.m_vecSphericalY.resize(scene->numVertices());
	m_context.m_vecSphericalZ.resize(scene->numVertices());

	m_context.m_vecSphericalC.assign(scene->numVertices(), Vec3f(0, 0, 0));
	m_context.m_vecSphericalX.assign(scene->numVertices(), Vec3f(0, 0, 0));
	m_context.m_vecSphericalY.assign(scene->numVertices(), Vec3f(0, 0, 0));
	m_context.m_vecSphericalZ.assign(scene->numVertices(), Vec3f(0, 0, 0));

    // fire away!
    m_launcher.setNumThreads(m_launcher.getNumCores());	// the solution exe is multithreaded
    //m_launcher.setNumThreads(1);							// but you have to make sure your code is thread safe before enabling this!
    m_launcher.popAll();
    m_launcher.push( vertexTaskFunc, &m_context, 0, scene->numVertices() );
}
// --------------------------------------------------------------------------

bool Radiosity::updateMeshColors(std::vector<Vec4f>& spherical1, std::vector<Vec4f>& spherical2, std::vector<float>& spherical3, bool spherical)
{
	if (!m_context.m_scene || m_context.m_vecResult.size()==0) return false;
    // Print progress.
    printf( "%.2f%% done     \r", 100.0f*m_launcher.getNumFinished()/m_context.m_scene->numVertices() );

    // Copy irradiance over to the display mesh.
    // Because we want outgoing radiosity in the end, we divide by PI here
    // and let the shader multiply the final diffuse reflectance in. See App::setupShaders() for details.
	for (int i = 0; i < m_context.m_scene->numVertices(); ++i) {

		// Packing data for the spherical harmonic extra.
		// In order to manage with fewer vertex attributes in the shader, the third component is stored as the w components of other actually three-dimensional vectors.
		if (spherical) {
			m_context.m_scene->mutableVertex(i).c = m_context.m_vecSphericalC[i] * (1.0f / FW_PI);
			spherical3[i] = m_context.m_vecSphericalZ[i].x * (1.0f / FW_PI);
			spherical1[i] = Vec4f(m_context.m_vecSphericalX[i], m_context.m_vecSphericalZ[i].y) * (1.0f / FW_PI);
			spherical2[i] = Vec4f(m_context.m_vecSphericalY[i], m_context.m_vecSphericalZ[i].z) * (1.0f / FW_PI);
		}
		else {
			m_context.m_scene->mutableVertex(i).c = m_context.m_vecResult[i] * (1.0f / FW_PI);
		}
	}
	return true;
}
// --------------------------------------------------------------------------

void Radiosity::checkFinish()
{
    // have all the vertices from current bounce finished computing?
    if ( m_launcher.getNumTasks() == m_launcher.getNumFinished() )
    {
        // yes, remove from task list
        m_launcher.popAll();

        // more bounces desired?
        if ( m_context.m_currentBounce < m_context.m_numBounces )
        {
            // move current bounce to prev
            m_context.m_vecPrevBounce = m_context.m_vecCurr;
            ++m_context.m_currentBounce;
            // start new tasks for all vertices
            m_launcher.push( vertexTaskFunc, &m_context, 0, m_context.m_scene->numVertices() );
            printf( "\nStarting bounce %d\n", m_context.m_currentBounce );
        }
        else printf( "\n DONE!\n" );
    }
}
// --------------------------------------------------------------------------

} // namespace FW
