
#include "AreaLight.hpp"


namespace FW {


void AreaLight::draw(const Mat4f& worldToCamera, const Mat4f& projection) {
    glUseProgram(0);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf((float*)&projection);
    glMatrixMode(GL_MODELVIEW);
    Mat4f S = Mat4f::scale(Vec3f(m_size,1));
    Mat4f M = worldToCamera *m_xform * S;
    glLoadMatrixf((float*)&M);
    glBegin(GL_TRIANGLES);
    //glColor3fv( &m_E.x );
    glColor3f(m_E.x, m_E.y, m_E.z);
    glVertex3f(1,1,0); glVertex3f(1,-1,0); glVertex3f( -1,-1,0 );
    glVertex3f(1,1,0); glVertex3f( -1,-1,0 ); glVertex3f(-1,1,0); 
    glEnd();
}

void AreaLight::writeTriangles(std::vector<RTTriangle>& triangles) {

    // Add triangle geometry to the vector of all triangles in the scene

    Mat4f M = m_xform * Mat4f::scale(Vec3f(m_size, 1));

    Vec3f A = (M * Vec4f(-1.0, -1.0, 0.0f, 1.0f)).getXYZ();
    Vec3f B = (M * Vec4f(-1.0, 1.0, 0.0f, 1.0f)).getXYZ();
    Vec3f C = (M * Vec4f(1.0, 1.0, 0.0f, 1.0f)).getXYZ();
    Vec3f D = (M * Vec4f(1.0, -1.0, 0.0f, 1.0f)).getXYZ();

    Vec3f normal = getNormal();
    // B --- C
    // |  /  |
    // A --- D

    VertexPNTC vA = VertexPNTC(A, normal, Vec2f(0.0f), Vec3f(0.0f));
    VertexPNTC vB = VertexPNTC(B, normal, Vec2f(0.0f), Vec3f(0.0f));
    VertexPNTC vC = VertexPNTC(C, normal, Vec2f(0.0f), Vec3f(0.0f));
    VertexPNTC vD = VertexPNTC(D, normal, Vec2f(0.0f), Vec3f(0.0f));

    RTTriangle t_1 = RTTriangle(vA, vC, vB);
    t_1.m_material = &m_material;

    RTTriangle t_2 = RTTriangle(vA, vD, vC);
    t_2.m_material = &m_material;

    triangles.push_back(t_1);
    triangles.push_back(t_2);



}

void AreaLight::sample(float& pdf, Vec3f& p, int base, Random& rnd) {
    // YOUR CODE HERE (R1): Integrate your area light implementation.

    float localX = rnd.getF32(-1.0f, 1.0f);
    float localY = rnd.getF32(-1.0f, 1.0f);

    float scaledX = localX * m_size.x;
    float scaledY = localY * m_size.y;

    Vec4f localPoint(scaledX, scaledY, 0.0f, 1.0f);
    Vec4f worldPoint = m_xform * localPoint;
    p = worldPoint.getXYZ();

    pdf = 1.0f / (4 * m_size.x * m_size.y); // probability  
}


} // namespace FW
