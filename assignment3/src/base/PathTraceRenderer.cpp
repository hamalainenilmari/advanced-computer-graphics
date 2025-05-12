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

            // also check normal and specular usage

            const Texture& tex = mat->textures[MeshBase::TextureType_Diffuse];
            const Image& teximg = *tex.getImage();

            Vec2f uv = (1.0f - u - v) * t1 + u * t2 + v * t3; // barycentric interpolation
            Vec2i texelCoords = getTexelCoords(uv, teximg.getSize());

            Vec3f d0 = teximg.getVec4f(texelCoords).getXYZ();  // p(y)
            diffuse = Vec3f( pow(d0.x, 2.2f), pow(d0.y, 2.2f), pow(d0.z, 2.2f)); // gamma correction
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
    AreaLight* otherLight = ctx.m_otherLight;

    //Vec3f emission = light->getEmission();                                     // E(y): radiance emitted from point y, if only one light source this is constant

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
	Rd = (Rd - Ro);  // initial ray direction

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
        //std::cout <<" what?";
        return Vec3f(0.0f);
    }

    // if we hit something, fetch a color and insert into image
    Vec3f Ei;
    Vec3f throughput(1, 1, 1);

    bool rr = ctx.m_bounces < 0;  // Russian Roulette mode
    float eps = 0.001f;
    int currentBounce = 0;

    // we hit something, enter path tracing stage and loop until terminate by bounce limit or russian roulette
    while (true) {
        if (!rr && currentBounce > ctx.m_bounces) { break; }
        if (result.tri != nullptr)
        {   
            MeshBase::Material* mat = result.tri->m_material;
            if (mat->emission.lenSqr() > 0.0f) {
                // we hit light with indirect
                //Ei += throughput * mat->emission;
                break;
            }
            
            // YOUR CODE HERE (R2-R4):
            // Implement path tracing with direct light and shadows, scattering and Russian roulette.

            // hit point properties
            Vec3f diffuse(0.0f);
            Vec3f specular(0.0f);
            Vec3f smoothedN(0.0f);

            // get surface (diffuse) color, smoothed/interpolated normal,
            getTextureParameters(result, diffuse, smoothedN, specular);

            Vec3f brdf = diffuse * (1.0f / FW_PI);
            float pdfL; // probability distribution function of the light source sample
            Vec3f Pl;   // sampled point on the light source

            if (ctx.m_shedScene) {
                // which light source to use?
                R.getF32(0.0f, 1.0f) > 0.50f ? ctx.m_otherLight->sample(pdfL, Pl, samplerBase, R) : ctx.m_light->sample(pdfL, Pl, samplerBase, R);
            }
            else {
                ctx.m_light->sample(pdfL, Pl, samplerBase, R);
            }

            // draw a point from light source surface for shadow ray tracing
            //ctx.m_light->sample(pdfL, Pl, samplerBase, R);

            // small offset to prevent self-shadowing
            Vec3f hitOrigin = result.point + smoothedN * eps;

            // construct vector from hit point to the light sample
            Vec3f vectorToLight = Vec3f(Pl - hitOrigin);

            float dis = vectorToLight.length(); // distance between vertex and light point to check if ray hits something before light
            vectorToLight = vectorToLight.normalized(); // normalize for direction

            // shoot ray from hit point to area light = trace shadow ray
            RaycastResult shadowRayRes = ctx.m_rt->raycast(hitOrigin, vectorToLight );

            // check if we hit anything before area light source i.e. visibility
            const RTTriangle* shadowHit = shadowRayRes.tri;
            //if (shadowRayRes.t >= dis - 0.001f) {
            if (shadowRayRes.tri != nullptr && shadowRayRes.tri->m_material->emission.lenSqr() > 0.0f) {

                Vec3f incomingLightDir = vectorToLight;                             // unit vector of direction to light
                float theta0 = std::max( 0.0f, incomingLightDir.dot(smoothedN) );                  // angle between the incoming direction w and the surface normal at x
                float thetaLight = std::max( 0.0f, -incomingLightDir.dot(ctx.m_light->getNormal()) ); // angle between the vector yx (from light to point) and the surface normal of the light
                float r_cube = (1.0f / (dis * dis));                                                 // squared distance
        
                // result += V(hit,y)*E(y,y->hit)*BRDF*cos*G(hit,y)/pdf1
                // this is irradiance from light to the hit point (direct lighting)
                Ei += throughput * (shadowRayRes.tri->m_material->emission * brdf * theta0 * thetaLight) * r_cube * (1.0f / pdfL);
            }
 

            // Russian Roulette termination
            if (rr) {
                if (-1 * currentBounce <= ctx.m_bounces) {
                    bool terminate = R.getF32(0.0f, 1.0f) > 0.80f; // terminate with 20 % probability
                    if (terminate) break;
                    // contribute to the ray
                    throughput *= (1.0f / 0.80f);
                }
            }

            // no termination so INDIRECT RAY CASTING: cosine weighted direction,

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

            Rd = (B * d0);    // NEW DIRECTION: transformed to vertex hemisphere,

            // angle between outgoing direction and current point normal
            float cosTheta = std::max(0.0f, Rd.dot(smoothedN));

            // PDF: p(omega) = cos(theta) / pi
            float pdf = (cosTheta / FW_PI);

            throughput *= brdf * cosTheta / pdf;

            // Shoot ray, see where we hit next
            result = ctx.m_rt->raycast(hitOrigin, Rd);
            
            if (debugVis)
            {
                // Example code for using the visualization system. You can expand this to include further bounces, 
                // shadow rays, and whatever other useful information you can think of.
                PathVisualizationNode node;
                node.lines.push_back(PathVisualizationLine(result.orig, result.point)); // Draws a line between two points
                node.lines.push_back(PathVisualizationLine(result.point, result.point + result.tri->normal() * .1f, Vec3f(1, 0, 0))); // You can give lines a color as optional parameter.
                node.lines.push_back(PathVisualizationLine(hitOrigin, Pl, Vec3f(0, 1, 1))); // shadow ray.
                //node.labels.push_back(PathVisualizationLabel("radiance light: " + std::to_string(shadowE.x) + ", " + std::to_string(shadowE.y) + ", " + std::to_string(shadowE.z), result.point)); // You can also render text labels with world-space locations.
                //node.labels.push_back(PathVisualizationLabel("res T: " + std::to_string(shadowRayRes.t) + ", distance: " + std::to_string(dis) + ", res == nullprs: " + std::to_string(shadowRayRes.tri == nullptr), hitOrigin * 1.05f)); // You can also render text labels with world-space locations.
                //node.labels.push_back(PathVisualizationLabel("thetas: " + std::to_string(theta0) + ", " + std::to_string(thetaLight), result.point + result.tri->normal() * 0.05f)); // You can also render text labels with world-space locations.
                //node.labels.push_back(PathVisualizationLabel("shadow ray point: " + std::to_string(shadowRayRes.point.x) + ", " + std::to_string(shadowRayRes.point.y) + ", " + std::to_string(shadowRayRes.point.z), shadowRayRes.point)); // You can also render text labels with world-space locations.
                //node.labels.push_back(PathVisualizationLabel("PL: " + std::to_string(Pl.x) + ", " + std::to_string(Pl.y) + ", " + std::to_string(Pl.z), Pl)); // You can also render text labels with world-space locations.
                //node.labels.push_back(PathVisualizationLabel("origin: " + std::to_string(hitOrigin.x) + ", " + std::to_string(hitOrigin.y) + ", " + std::to_string(hitOrigin.z), hitOrigin)); // You can also render text labels with world-space locations.
                node.labels.push_back(PathVisualizationLabel("normal: " + std::to_string(result.tri->normal().x) + ", " + std::to_string(result.tri->normal().y) + ", " + std::to_string(result.tri->normal().z), hitOrigin)); // You can also render text labels with world-space locations.
                node.lines.push_back(PathVisualizationLine(light->getPosition(), light->getPosition() + light->getNormal() * .1f, Vec3f(1, 0, 1))); // You can give lines a color as optional parameter.
                //node.labels.push_back(PathVisualizationLabel("diffuse: " + std::to_string(Ei.x) + ", " + std::to_string(Ei.y) + ", " + std::to_string(Ei.z), result.point)); // You can also render text labels with world-space locations.

                visualization.push_back(node);
            }
        }
        else {
            break;
        }
        currentBounce++;
        // back to the loop

    }
	return Ei;
}

// simple box filtering
float boxFilter(float x, float y, float pX, float pY) {

    float x0 = x - (pX + 0.5f);
    float y0 = y - (pY + 0.5f);

    if (std::abs(x0) < 0.5f && std::abs(y0) < 0.5f) {
        return 1.0f;
        std::cout << "1";
    }
    else {
        std::cout << "0";

        return 0.0f;
    }
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

    Vec3f Ei; // = tracePath(pixel_x, pixel_y, ctx, 0, R, dummyVisualization);
    float w = 0; // total weight for pixel filtering

    for ( int i = 0; i < block.m_width * block.m_height; ++i )
    {
        if( ctx.m_bForceExit ) {
            return;
        }

        // Use if you want.
        int pixel_x = block.m_x + (i % block.m_width);
        int pixel_y = block.m_y + (i / block.m_width);
        //std::cout << "x,y: " << pixel_x << ", " << pixel_y << std::endl;
        
        // TODO add pixel filtering

        // random pixel samples for antialiasing
        for (int j = 0; j < 50; ++j) {
            float x_i = R.getF32(pixel_x, pixel_x + 1.0f);
            float y_i = R.getF32(pixel_y, pixel_y + 1.0f);
            
            //float weight = boxFilter(x_i, y_i, pixel_x, pixel_y);
            //std::cout << "x,y: " << x0 << ", " << y0 << std::endl;

            //if (weight > 0.0f) {
            Ei += tracePath(x_i, y_i, ctx, 0, R, dummyVisualization);
            //w += weight;
            //}
        }
        
        // normalize by dividing with weight
        Ei = Ei / 50;
        // Put pixel.
        Vec4f prev = image->getVec4f( Vec2i(pixel_x, pixel_y) );
        prev += Vec4f( Ei, 1.0f );
        image->setVec4f( Vec2i(pixel_x, pixel_y), prev );
    }
}

void PathTraceRenderer::startPathTracingProcess( const MeshWithColors* scene, AreaLight* light, bool shedScene, AreaLight* otherLight, RayTracer* rt, Image* dest, int bounces, const CameraControls& camera )
{
    FW_ASSERT( !m_context.m_bForceExit );

    m_context.m_bForceExit = false;
    m_context.m_bResidual = false;
    m_context.m_camera = &camera;
    m_context.m_rt = rt;
    m_context.m_scene = scene;
    m_context.m_light = light;
    m_context.m_shedScene = shedScene;
    m_context.m_otherLight = otherLight;
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
