#include "PathTraceRenderer.hpp"
#include "RayTracer.hpp"
#include "AreaLight.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <string>
#include <cmath>


namespace FW {

	bool PathTraceRenderer::m_normalMapped = false;
	bool PathTraceRenderer::debugVis = false;

	void PathTraceRenderer::getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular)
	{
		MeshBase::Material* mat = hit.tri->m_material;
		// YOUR CODE HERE (R1)
		// Read value from albedo texture into diffuse.
	    // If textured, use the texture; if not, use Material.diffuse.
	    // Note: You can probably reuse parts of the radiosity assignment.

        Vec2f t1 = hit.tri->m_vertices[0].t;
        Vec2f t2 = hit.tri->m_vertices[1].t;
        Vec2f t3 = hit.tri->m_vertices[2].t;

        float u = hit.u;
        float v = hit.v;

        Vec3f n0 = hit.tri->m_vertices[0].n;
        Vec3f n1 = hit.tri->m_vertices[1].n;
        Vec3f n2 = hit.tri->m_vertices[2].n;

        n = ((1.0f - u - v) * n0 + u * n1 + v * n2).normalized(); // barycentric interpolation of normals and hitpoint = smoothed normal

        // check if has texture
        if (mat->textures[MeshBase::TextureType_Diffuse].exists())
        {
            // yes, read diffuse texture

            // TODO:
            // Note that you will have to apply gamma correction to diffuse values read from a texture.
            // This means that you will have to raise all components of diffuse to power 2.2.
            // also check normal and specular usage

            const Texture& tex = mat->textures[MeshBase::TextureType_Diffuse];
            const Image& teximg = *tex.getImage();

            Vec2f uv = (1.0f - u - v) * t1 + u * t2 + v * t3; // barycentric interpolation
            Vec2i texelCoords = getTexelCoords(uv, teximg.getSize());

            Vec3f d0 = teximg.getVec4f(texelCoords).getXYZ();  // p(y)
            diffuse = Vec3f( pow(d0.x, 2.2f), pow(d0.y, 2.2f), pow(d0.z, 2.2f));
        }
        else {
            // no texture
            diffuse = mat->diffuse.getXYZ();
            //diffuse = Vec3f(pow(d0.x, 2.2f), pow(d0.y, 2.2f), pow(d0.z, 2.2f));
        }


	}


PathTracerContext::PathTracerContext()
    : m_bForceExit(false),
      m_bResidual(false),
      m_scene(nullptr),
      m_rt(nullptr),
      m_light(nullptr),
      m_pass(0),
      m_bounces(0),
      m_destImage(0),
      m_camera(nullptr)
{
}

PathTracerContext::~PathTracerContext()
{
}


PathTraceRenderer::PathTraceRenderer()
{
    m_raysPerSecond = 0.0f;
}

PathTraceRenderer::~PathTraceRenderer()
{
    stop();
}

// This function traces a single path and returns the resulting color value that will get rendered on the image. 
// Filling in the blanks here is all you need to do this time around.
Vec3f PathTraceRenderer::tracePath(float image_x, float image_y, PathTracerContext& ctx, int samplerBase, Random& R, std::vector<PathVisualizationNode>& visualization)
{
	const MeshWithColors* scene = ctx.m_scene;
	RayTracer* rt = ctx.m_rt;
	Image* image = ctx.m_image.get();
	const CameraControls& cameraCtrl = *ctx.m_camera;
	AreaLight* light = ctx.m_light;

	// make sure we're on CPU
	//image->getMutablePtr();

	// get camera orientation and projection
	Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
	Mat4f projection = Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize())*cameraCtrl.getCameraToClip();

	// inverse projection from clip space to world space
	Mat4f invP = (projection * worldToCamera).inverted();


	// Simple ray generation code, you can use this if you want to.

	// Generate a ray through the pixel.
	float x = (float)image_x / image->getSize().x *  2.0f - 1.0f;
	float y = (float)image_y / image->getSize().y * -2.0f + 1.0f;

	// point on front plane in homogeneous coordinates
	Vec4f P0(x, y, 0.0f, 1.0f);
	// point on back plane in homogeneous coordinates
	Vec4f P1(x, y, 1.0f, 1.0f);

	// apply inverse projection, divide by w to get object-space points
	Vec4f Roh = (invP * P0);
	Vec3f Ro = (Roh * (1.0f / Roh.w)).getXYZ();
	Vec4f Rdh = (invP * P1);
	Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();

	// Subtract front plane point from back plane point,
	// yields ray direction.
	// NOTE that it's not normalized; the direction Rd is defined
	// so that the segment to be traced is [Ro, Ro+Rd], i.e.,
	// intersections that come _after_ the point Ro+Rd are to be discarded.
	Rd = Rd - Ro;  // initial ray direction

    // trace!
    RaycastResult result = rt->raycast(Ro, Rd); // shoot the ray
    const RTTriangle* pHit = result.tri; // hit point triangle

    // Ray from camera directly hits an emitter. We can stop if we assume the emitter does not reflect light, only emits.
    if ((result.tri != nullptr)) {
        MeshBase::Material* mat = result.tri->m_material;
        if (mat->emission.lenSqr() > 0.0f) { // 'Very direct' light
            return Rd.dot(result.tri->normal()) > 0.0 ? mat->emission : Vec3f(0.0f);
        }
    }
    else {
        // ray does not hit anything in scene -> output zero for this pixel
        return Vec3f(0.0f);
        //Vec3f::setZero
    }

    // if we hit something, fetch a color and insert into image
    Vec3f Ei;
    Vec3f throughput(1, 1, 1);
    float p = 1.0f;

    int currentBounce = 0;
    Vec3f E(0.0f); // total radiance

    // we hit something, enter path tracing stage and loop the path for bounce times
    while (currentBounce < ctx.m_bounces) {
        if (result.tri != nullptr)
        {
            // YOUR CODE HERE (R2-R4):
            // Implement path tracing with direct light and shadows, scattering and Russian roulette.

            /*
            Determine the hit point coordinates, normal, barycentric coordinates and
            the surface color. Like in the previous assignments, the color is determined
            from the surface diffuse color or the texture if it is present. Implement the
            color fetching in PathTraceRenderer::getTextureParameters(). Note that
            you will have to apply gamma correction to diffuse values read from a texture. 
            This means that you will have to raise all components of diffuse to
            power 2.2. This is necessary, because images are gamma encoded in order
            to optimize bit usage. To get smooth normals, interpolate the vertex normals
            to the hit position and normalize it. The normal is interpolated according to
            the hit barycentrics similarly to texture coordinates.
            */ 

            result.u; // hit triangle bary u
            result.v; // hit triangle bary v
            result.point; // hit point in hit triangle
            pHit->normal(); // hit triangle normal
            pHit->m_material->specular; // specular / shiny
            pHit->m_material->diffuse.getXYZ(); // diffuse / color
            pHit->m_material->textures[MeshBase::TextureType_Diffuse]; // diffuse texture, check if exists before useis


            Vec3f diffuse(0.0f);
            Vec3f specular(0.0f);
            Vec3f smoothedN(0.0f);

            // 1. get surface (diffuse) color, smoothed/interpolated normal,
            getTextureParameters(result, diffuse, smoothedN, specular);

            const Vec3i& indices = result.tri->m_data.vertex_indices;

            // check for backfaces => don't accumulate if we hit a surface from below!
            /*
            float angle = Rd.dot(result.tri->normal());                           // angle between the ray and hit triangle normal
            if (angle > 0) {
                // ignore irradiance if we hit a surface from below
                continue;
            }
            */

            // fetch barycentric coordinates
            float u = result.u;
            float v = result.v;

            // Ei = interpolated irradiance

            Vec3f E0 = indices[0]; // Irradiance at a vertex of hit triangle
            Vec3f E1 = indices[1];
            Vec3f E2 = indices[2];

            //Vec3f Ei = u * E0 + v * E1 + (1 - u - v) * E2; // total incident irradiance

            // Divide incident irradiance by PI so that we can turn it into outgoing
            // radiosity by multiplying by the reflectance factor below.
            Ei = (1.0f / FW_PI);

            Ei = diffuse * Ei;
            //E += Ei;

            /*
            Draw a point from the light source surface, trace a shadow ray, and add the
            appropriate contribution to the radiance returned by the path. Be careful
            with the probabilities. This is what we called “next event estimation” at the
            lectures.
            */

            // at each hit, we also sample light for shadow rays in path tracing
            
            float pdfL; // probability distribution function of the light source sample
            Vec3f Pl;   // point on the light source

            // draw a point from light source surface
            ctx.m_light->sample(pdfL, Pl, samplerBase, R);

            // construct vector from hit point to light sample
            Vec3f vectorToLight = Vec3f(Pl - result.point);

            float dis = vectorToLight.length(); // store distance between vertex and light point to check if ray hits something before light

            // shoot ray from hit point to area light = trace shadow ray
            RaycastResult res = ctx.m_rt->raycast(result.point, vectorToLight);

            // TODO "next event estimation" & check probabilities

            if (res.t >= dis - 0.00001f) {
                // if not, add the appropriate emission, 1/r^2 and clamped cosine terms, accounting for the PDF as well.
                // accumulate into E

                Vec3f incomingLightDir = vectorToLight.normalized();                             // unit vector of direction to light

                float theta = std::max(0.0f, incomingLightDir.dot(smoothedN));                   // angle between the incoming direction w and the surface normal at x
                float thetaLight = std::max(0.0f, -vectorToLight.dot(ctx.m_light->getNormal())); // angle between the vector yx (from light to point) and the surface normal of the light
                Vec3f emission = ctx.m_light->getEmission();                                     // E(y): radiance emitted from point y
                float rr = (1.0f / (dis * dis));

                Vec3f irradiance = (emission * theta * thetaLight) * (rr) * (1.0f / pdfL);        // irradiance E
                /*
                std::cout << "emission: " << emission << std::endl;
                std::cout << "theta: " << theta << std::endl;
                std::cout << "thetaLight: " << thetaLight << std::endl;
                std::cout << "rr: " << rr << std::endl;
                std::cout << "pdf: " << pdf << std::endl;
                */
                E = E + irradiance;
            }

            //Ei = result.tri->m_material->diffuse.getXYZ(); // placeholder

            // INDIRECT RAY CASTING: cosine weighted direction, p(omega) = cos(theta) / pi

            float n1 = R.getF32(0.0f, 1.0f);
            float n2 = R.getF32(0.0f, 1.0f);

            // convert to spherical coordinates
            float phi = 2 * FW_PI * n1;
            float theta = std::acos(std::sqrt(n2));

            // cartesian
            float x = std::sin(theta) * std::cos(phi);
            float y = std::sin(theta) * std::sin(phi);
            float z = std::cos(theta);

            Vec3f d0 = Vec3f(x, y, z); // direction where to shoot next ray from hit point

            Mat3f B = formBasis(smoothedN);

            Rd = B * d0 * 100; // NEW DIRECTION: transformed to vertex hemisphere, stretch length
            float pdf = z / FW_PI;  // probability distrbution function of ray from hit to hemisphere

            // Shoot ray, see where we hit next
            result = ctx.m_rt->raycast(result.point, Rd);
            
            if (result.tri == nullptr) {
                break;
            }

            Pl = result.point;

            // then back to start of loop, and check if this ray hits anything


            if (debugVis)
            {
                // Example code for using the visualization system. You can expand this to include further bounces, 
                // shadow rays, and whatever other useful information you can think of.
                PathVisualizationNode node;
                node.lines.push_back(PathVisualizationLine(result.orig, result.point)); // Draws a line between two points
                node.lines.push_back(PathVisualizationLine(result.point, result.point + result.tri->normal() * .1f, Vec3f(1, 0, 0))); // You can give lines a color as optional parameter.
                node.labels.push_back(PathVisualizationLabel("diffuse: " + std::to_string(Ei.x) + ", " + std::to_string(Ei.y) + ", " + std::to_string(Ei.z), result.point)); // You can also render text labels with world-space locations.

                visualization.push_back(node);
            }
        }
        currentBounce += 1;
    }
    //std::cout << "E: " << E << std::endl;
	return E;
}

// This function is responsible for asynchronously generating paths for a given block.
void PathTraceRenderer::pathTraceBlock( MulticoreLauncher::Task& t )
{
    PathTracerContext& ctx = *(PathTracerContext*)t.data;

    const MeshWithColors* scene			= ctx.m_scene;
    RayTracer* rt						= ctx.m_rt;
    Image* image						= ctx.m_image.get();
    const CameraControls& cameraCtrl	= *ctx.m_camera;
    AreaLight* light					= ctx.m_light;

    // make sure we're on CPU
    image->getMutablePtr();

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1,-1), Vec2f(2,2), image->getSize())*cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // get the block which we are rendering
    PathTracerBlock& block = ctx.m_blocks[t.idx];

	// Not used but must be passed to tracePath
	std::vector<PathVisualizationNode> dummyVisualization; 

	static std::atomic<uint32_t> seed = 0;
	uint32_t current_seed = seed.fetch_add(1);
	Random R(t.idx + current_seed);	// this is bogus, just to make the random numbers change each iteration

    for ( int i = 0; i < block.m_width * block.m_height; ++i )
    {
        if( ctx.m_bForceExit ) {
            return;
        }

        // Use if you want.
        int pixel_x = block.m_x + (i % block.m_width);
        int pixel_y = block.m_y + (i / block.m_width);

		Vec3f Ei = tracePath(pixel_x, pixel_y, ctx, 0, R, dummyVisualization);

        // Put pixel.
        Vec4f prev = image->getVec4f( Vec2i(pixel_x, pixel_y) );
        prev += Vec4f( Ei, 1.0f );
        image->setVec4f( Vec2i(pixel_x, pixel_y), prev );
    }
}

void PathTraceRenderer::startPathTracingProcess( const MeshWithColors* scene, AreaLight* light, RayTracer* rt, Image* dest, int bounces, const CameraControls& camera )
{
    FW_ASSERT( !m_context.m_bForceExit );

    m_context.m_bForceExit = false;
    m_context.m_bResidual = false;
    m_context.m_camera = &camera;
    m_context.m_rt = rt;
    m_context.m_scene = scene;
    m_context.m_light = light;
    m_context.m_pass = 0;
    m_context.m_bounces = bounces;
    m_context.m_image.reset(new Image( dest->getSize(), ImageFormat::RGBA_Vec4f));

    m_context.m_destImage = dest;
    m_context.m_image->clear();

    // Add rendering blocks.
    m_context.m_blocks.clear();
    {
        int block_size = 32;
        int image_width = dest->getSize().x;
        int image_height = dest->getSize().y;
        int block_count_x = (image_width + block_size - 1) / block_size;
        int block_count_y = (image_height + block_size - 1) / block_size;

        for(int y = 0; y < block_count_y; ++y) {
            int block_start_y = y * block_size;
            int block_end_y = FW::min(block_start_y + block_size, image_height);
            int block_height = block_end_y - block_start_y;

            for(int x = 0; x < block_count_x; ++x) {
                int block_start_x = x * block_size;
                int block_end_x = FW::min(block_start_x + block_size, image_width);
                int block_width = block_end_x - block_start_x;

                PathTracerBlock block;
                block.m_x = block_size * x;
                block.m_y = block_size * y;
                block.m_width = block_width;
                block.m_height = block_height;

                m_context.m_blocks.push_back(block);
            }
        }
    }

    dest->clear();

    // Fire away!

    // If you change this, change the one in checkFinish too.
    m_launcher.setNumThreads(m_launcher.getNumCores());
    //m_launcher.setNumThreads(1);

    m_launcher.popAll();
    m_launcher.push( pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size() );
}

void PathTraceRenderer::updatePicture( Image* dest )
{
    FW_ASSERT( m_context.m_image != 0 );
    FW_ASSERT( m_context.m_image->getSize() == dest->getSize() );

    for ( int i = 0; i < dest->getSize().y; ++i )
    {
        for ( int j = 0; j < dest->getSize().x; ++j )
        {
            Vec4f D = m_context.m_image->getVec4f(Vec2i(j,i));
            if ( D.w != 0.0f )
                D = D*(1.0f/D.w);

            // Gamma correction.
            Vec4f color = Vec4f(
                FW::pow(D.x, 1.0f / 2.2f),
                FW::pow(D.y, 1.0f / 2.2f),
                FW::pow(D.z, 1.0f / 2.2f),
                D.w
            );

            dest->setVec4f( Vec2i(j,i), color );
        }
    }
}

void PathTraceRenderer::checkFinish()
{
    // have all the vertices from current bounce finished computing?
    if ( m_launcher.getNumTasks() == m_launcher.getNumFinished() )
    {
        // yes, remove from task list
        m_launcher.popAll();

        ++m_context.m_pass;

        // you may want to uncomment this to write out a sequence of PNG images
        // after the completion of each full round through the image.
        //String fn = sprintf( "pt-%03dppp.png", m_context.m_pass );
        //File outfile( fn, File::Create );
        //exportLodePngImage( outfile, m_context.m_destImage );

        if ( !m_context.m_bForceExit )
        {
            // keep going

            // If you change this, change the one in startPathTracingProcess too.
            m_launcher.setNumThreads(m_launcher.getNumCores());
            //m_launcher.setNumThreads(1);

            m_launcher.popAll();
            m_launcher.push( pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size() );
            //::printf( "Next pass!" );
        }
        else ::printf( "Stopped." );
    }
}

void PathTraceRenderer::stop() {
    m_context.m_bForceExit = true;
    
    if ( isRunning() )
    {
        m_context.m_bForceExit = true;
        while( m_launcher.getNumTasks() > m_launcher.getNumFinished() )
        {
            Sleep( 1 );
        }
        m_launcher.popAll();
    }

    m_context.m_bForceExit = false;
}



} // namespace FW
