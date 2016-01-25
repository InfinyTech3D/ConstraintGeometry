//#ifndef SOFA_COMPONENT_SURFACECUBICINTERPOLATION_INL
//#define SOFA_COMPONENT_SURFACECUBICINTERPOLATION_INL

//#include "SurfaceCubicInterpolation.h"
//#include <sofa/helper/Quater.h>
//#include <sofa/core/visual/VisualParams.h>
//#include <SofaOpenglVisual/OglModel.h>
//#include <sofa/simulation/common/AnimateBeginEvent.h>
//#include <SofaConstraint/BilateralInteractionConstraint.h>
//#include <SofaBaseMechanics/MechanicalObject.h>
//#include <sofa/core/visual/VisualParams.h>
//#include <SofaOpenglVisual/OglModel.h>

//namespace sofa {

//namespace component {

//namespace constraintset {


//SurfaceCubicInterpolation::SurfaceCubicInterpolation()
//: Inherit()
//, f_use_bezier(initData(&f_use_bezier, true,"use_bezier","Use cubic bezier triangle. if false, project on flat triangle"))
//, f_bezier_maw_it(initData(&f_bezier_maw_it, (unsigned) 10,"bezier_maw_it","Max iteration in the Newton Raphson solver used for projection of points on bezier curve"))
//, f_bezier_tol(initData(&f_bezier_tol, (double) 0.01,"bezier_tol","Tolerance in the Newton Raphson solver used for projection of points on bezier curve"))
//, f_draw_bezier_inc(initData(&f_draw_bezier_inc, (double) 0.0,"draw_bezier_inc","Draw bezier triangle"))
//{}

//void SurfaceCubicInterpolation::prepareDetection() {
//    Inherit::prepareDetection();

//    m_beziertriangle_info.resize(this->m_container->getNbTriangles());
//    for (int t=0;t<this->m_container->getNbTriangles();t++) {
//        const TriangleInfo & tinfo = m_triangle_info[t];
//        BezierTriangleInfo & tbinfo = m_beziertriangle_info[t];

//        const Vector3 & p300 = tinfo.p300;
//        const Vector3 & p030 = tinfo.p030;
//        const Vector3 & p003 = tinfo.p003;

//        const Vector3 & n200 = m_point_normal[tinfo.id0];
//        const Vector3 & n020 = m_point_normal[tinfo.id1];
//        const Vector3 & n002 = m_point_normal[tinfo.id2];

//        double w12 = dot(p030 - p300,n200);
//        double w21 = dot(p300 - p030,n020);
//        double w23 = dot(p003 - p030,n020);
//        double w32 = dot(p030 - p003,n002);
//        double w31 = dot(p300 - p003,n002);
//        double w13 = dot(p003 - p300,n200);

//        tbinfo.p210 = (p300*2.0 + p030 - n200 * w12) / 3.0;
//        tbinfo.p120 = (p030*2.0 + p300 - n020 * w21) / 3.0;

//        tbinfo.p021 = (p030*2.0 + p003 - n020 * w23) / 3.0;
//        tbinfo.p012 = (p003*2.0 + p030 - n002 * w32) / 3.0;

//        tbinfo.p102 = (p003*2.0 + p300 - n002 * w31) / 3.0;
//        tbinfo.p201 = (p300*2.0 + p003 - n200 * w13) / 3.0;

//        Vector3 E = (tbinfo.p210+tbinfo.p120+tbinfo.p102+tbinfo.p201+tbinfo.p021+tbinfo.p012) / 6.0;
//        Vector3 V = (p300+p030+p003) / 3.0;
//        tbinfo.p111 =  E + (E-V) / 2.0;

//        //Compute Bezier Normals
//        double v12 = 2 * dot(p030-p300,n200+n020) / dot(p030-p300,p030-p300);
//        double v23 = 2 * dot(p003-p030,n020+n002) / dot(p003-p030,p003-p030);
//        double v31 = 2 * dot(p300-p003,n002+n200) / dot(p300-p003,p300-p003);

//        Vector3 h110 = n200 + n020 - (p030-p300) * v12;
//        Vector3 h011 = n020 + n002 - (p003-p030) * v23;
//        Vector3 h101 = n002 + n200 - (p300-p003) * v31;

//        tbinfo.n110 = h110 / h110.norm();
//        tbinfo.n011 = h011 / h011.norm();
//        tbinfo.n101 = h101 / h101.norm();
//    }
//}

//void SurfaceCubicInterpolation::newtonIterations(const Vector3 & P,const TriangleInfo & tinfo,const BezierTriangleInfo & tbinfo,double & fact_w,double & fact_u, double & fact_v) {
//    double epsilon = f_bezier_tol.getValue() * f_bezier_tol.getValue();
//    unsigned int it=0;
//    while(it< f_bezier_maw_it.getValue()) {
//        //Compute Point on bezier patch
//        Vector3 Q_i = tinfo.p300 *   fact_w*fact_w*fact_w +
//                      tinfo.p030 *   fact_u*fact_u*fact_u +
//                      tinfo.p003 *   fact_v*fact_v*fact_v +
//                      tbinfo.p210 * 3*fact_w*fact_w*fact_u +
//                      tbinfo.p120 * 3*fact_w*fact_u*fact_u +
//                      tbinfo.p201 * 3*fact_w*fact_w*fact_v +
//                      tbinfo.p021 * 3*fact_u*fact_u*fact_v +
//                      tbinfo.p102 * 3*fact_w*fact_v*fact_v +
//                      tbinfo.p012 * 3*fact_u*fact_v*fact_v +
//                      tbinfo.p111 * 6*fact_w*fact_u*fact_v;


//        Vector3 p = Q_i - P;

//        //Compute first derivative
//        Vector3 dpdu = -3.0 * tinfo.p300 * fact_w * fact_w +
//                        3.0 * tinfo.p030 * fact_u * fact_u -
//                        3.0 * tbinfo.p210 * (2.0 * fact_w * fact_u - fact_w * fact_w) -
//                        3.0 * tbinfo.p120 * (fact_u * fact_u - 2.0 * fact_w * fact_u) -
//                        6.0 * tbinfo.p201 * fact_w * fact_v +
//                        6.0 * tbinfo.p021 * fact_u * fact_v -
//                        3.0 * tbinfo.p102 * fact_v * fact_v +
//                        3.0 * tbinfo.p012 * fact_v * fact_v -
//                        6.0 * tbinfo.p111 * (fact_u * fact_v - fact_w * fact_v);

//        Vector3 dpdv = -3.0 * tinfo.p300 * fact_w * fact_w +
//                        3.0 * tinfo.p003 * fact_v * fact_v -
//                        6.0 * tbinfo.p210 * fact_w * fact_u -
//                        3.0 * tbinfo.p120 * fact_u * fact_u -
//                        3.0 * tbinfo.p201 * (2.0 * fact_w * fact_v - fact_w * fact_w) +
//                        3.0 * tbinfo.p021 * fact_u * fact_u -
//                        3.0 * tbinfo.p102 * (fact_v * fact_v - 2.0 * fact_w * fact_v) +
//                        6.0 * tbinfo.p012 * fact_u * fact_v -
//                        6.0 * tbinfo.p111 * (fact_u * fact_v - fact_w * fact_u);

//        // F(u,v) = ( (Q(u,v)-P).(dpdu(u,v)), (Q(u,v)-P).(dpdv(u,v)))
//        Vector2 F( p*dpdu, p*dpdv);

//        ////////// Convergence
//        if(F.norm()<epsilon) return;

//        Vector3 d2pdu2 = 6.0  * tinfo.p300 * fact_w +
//                         6.0  * tinfo.p030 * fact_u +
//                         6.0  * tbinfo.p210 * (fact_u - 2.0 * fact_w) -
//                         6.0  * tbinfo.p120 * (2.0 * fact_u - fact_w) +
//                         6.0  * tbinfo.p201 * fact_v +
//                         6.0  * tbinfo.p021 * fact_v -
//                         12.0 * tbinfo.p111 * fact_v;

//        Vector3 d2pdv2 = 6.0  * tinfo.p300 * fact_w +
//                         6.0  * tinfo.p003 * fact_v +
//                         6.0  * tbinfo.p210 * fact_u +
//                         6.0  * tbinfo.p201 * (fact_v - 2.0 * fact_w) -
//                         6.0  * tbinfo.p102 * (2.0 * fact_v - fact_w) +
//                         6.0  * tbinfo.p012 * fact_u -
//                         12.0 * tbinfo.p111 * fact_u;

//        Vector3 d2pduv = 6.0 * tinfo.p300 * fact_w +
//                         6.0 * tbinfo.p210 * (fact_u - fact_w) -
//                         6.0 * tbinfo.p120 * fact_u +
//                         6.0 * tbinfo.p201 * (fact_v - fact_w) +
//                         6.0 * tbinfo.p021 * fact_u -
//                         6.0 * tbinfo.p102 * fact_v +
//                         6.0 * tbinfo.p012 * fact_v -
//                         6.0 * tbinfo.p111 * (fact_u + fact_v - fact_w);

//        defaulttype::Mat<2,2,double> dFdUV, invM;
//        dFdUV[0][0] = dpdu*dpdu-d2pdu2*p;
//        dFdUV[1][0] = dFdUV[0][1] = dpdv*dpdu-d2pduv*p;
//        dFdUV[1][1] = dpdv*dpdv-d2pdv2*p;

//        double det = dFdUV[0][0]*dFdUV[1][1]-dFdUV[1][0]*dFdUV[1][0];
//        if (fabs(det) < epsilon) return; // fails to converge

//        invM[0][0] = dFdUV[1][1]/det;
//        invM[1][1] = dFdUV[0][0]/det;
//        invM[1][0] = invM[0][1] = -dFdUV[0][1]/det;

//        Vector2 dUV = invM*F;

//        double new_u = fact_u-dUV[0];
//        double new_v = fact_v-dUV[1];
//        double new_w = 1.0 - new_u - new_v;

//        if (dUV[0]*dUV[0] + dUV[1]*dUV[1] < epsilon) return;//check dx;

//        if (new_u < 0.0) new_u = 0.0;
//        if (new_u > 1.0) new_u = 1.0;
//        if (new_v < 0.0) new_v = 0.0;
//        if (new_v > 1.0) new_v = 1.0;
//        if (new_u+new_v>1.0) {
//            //project the point on edge 2 and recompute barycoord
//            Vector3 edge_e0 = tinfo.p003 - tinfo.p030;
//            Vector3 edge_v2 = Q_i - tinfo.p030;

//            double dot2 = dot(edge_v2,edge_e0) / dot(edge_e0,edge_e0);
//            if (dot2<0.0) dot2 = 0.0;
//            else if (dot2>1.0) dot2 = 1.0;

//            Vector3 edge_P = tinfo.p030 + edge_e0 * dot2;

//            Vector3 v0 = tinfo.p030 - tinfo.p300;
//            Vector3 v1 = tinfo.p003 - tinfo.p300;

//            double d00 = dot(v0, v0);
//            double d01 = dot(v0, v1);
//            double d11 = dot(v1, v1);
//            double invDenom = 1.0 / (d00 * d11 - d01 * d01);

//            Vector3 v2 = edge_P - tinfo.p300;
//            double d20 = dot(v2, v0);
//            double d21 = dot(v2, v1);
//            new_u = (d11 * d20 - d01 * d21) * invDenom;
//            new_v = (d00 * d21 - d01 * d20) * invDenom;
//        }
//        new_w = 1.0 - new_u - new_v;

//        double dx = (fact_u-new_u) * (fact_u-new_u) +
//                    (fact_v-new_v) * (fact_u-new_v) +
//                    (fact_w-new_w) * (fact_u-new_w);

//        fact_u = new_u;
//        fact_v = new_v;
//        fact_w = new_w;

//        if (dx < epsilon) return;

//        it++;
//    }
//}

//////Bezier triangle are computed according to :
//////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
//void SurfaceCubicInterpolation::processCollisionDetectionWithBezierCurve(const Vector3 & P,Proximity & pinfo) {
//    //helper::ReadAccessor<Data <VecCoord> > xfree = *this->m_state->read(core::VecCoordId::freePosition());
//    pinfo.pid.resize(3);
//    pinfo.dot.resize(3);

//    //build edges
//    double minDist = 0.0;
//    for(int t=0;t<this->m_container->getNbTriangles();t++) {
//        const TriangleInfo & tinfo = m_triangle_info[t];
//        const BezierTriangleInfo & tbinfo = m_beziertriangle_info[t];

//        double fact_u;
//        double fact_v;
//        double fact_w;

//        projectPointOnTriangle(P,tinfo,fact_w,fact_u,fact_v); // initialize fact_u,fact_v and fact_w with projected point on the triangle
//        newtonIterations(P,tinfo,tbinfo,fact_w,fact_u,fact_v);

//        Vector3 Q = tinfo.p300 *   fact_w*fact_w*fact_w +
//                    tinfo.p030 *   fact_u*fact_u*fact_u +
//                    tinfo.p003 *   fact_v*fact_v*fact_v +
//                    tbinfo.p210 * 3*fact_w*fact_w*fact_u +
//                    tbinfo.p120 * 3*fact_w*fact_u*fact_u +
//                    tbinfo.p201 * 3*fact_w*fact_w*fact_v +
//                    tbinfo.p021 * 3*fact_u*fact_u*fact_v +
//                    tbinfo.p102 * 3*fact_w*fact_v*fact_v +
//                    tbinfo.p012 * 3*fact_u*fact_v*fact_v +
//                    tbinfo.p111 * 6*fact_w*fact_u*fact_v;

//        //1.0 + 0.1 i.e. 0.1 is to avoid zero
//        double dist = (Q-P).norm();

//        if ((t==0) || (dist < minDist)) {
//            pinfo.eid = t;
//            pinfo.pid[0] = tinfo.id0;
//            pinfo.pid[1] = tinfo.id1;
//            pinfo.pid[2] = tinfo.id2;

//            pinfo.dot[0] = fact_w;
//            pinfo.dot[1] = fact_u;
//            pinfo.dot[2] = fact_v;

//            minDist = dist;
//        }
//    }
//}

//SurfaceCubicInterpolation::Vector3 SurfaceCubicInterpolation::getContactPosition(const Proximity & pinfo) {
//    const TriangleInfo & tinfo = m_triangle_info[pinfo.eid];
//    const BezierTriangleInfo & tbinfo = m_beziertriangle_info[pinfo.eid];

//    double fact_w = pinfo.dot[0];
//    double fact_u = pinfo.dot[1];
//    double fact_v = pinfo.dot[2];

//    return tinfo.p300 *   fact_w*fact_w*fact_w +
//           tinfo.p030 *   fact_u*fact_u*fact_u +
//           tinfo.p003 *   fact_v*fact_v*fact_v +
//           tbinfo.p210 * 3*fact_w*fact_w*fact_u +
//           tbinfo.p120 * 3*fact_w*fact_u*fact_u +
//           tbinfo.p201 * 3*fact_w*fact_w*fact_v +
//           tbinfo.p021 * 3*fact_u*fact_u*fact_v +
//           tbinfo.p102 * 3*fact_w*fact_v*fact_v +
//           tbinfo.p012 * 3*fact_u*fact_v*fact_v +
//           tbinfo.p111 * 6*fact_w*fact_u*fact_v;
//}

//SurfaceCubicInterpolation::Vector3 SurfaceCubicInterpolation::getContactFreePosition(const Proximity & pinfo) {
//    double fact_w = pinfo.dot[0];
//    double fact_u = pinfo.dot[1];
//    double fact_v = pinfo.dot[2];

//    const helper::ReadAccessor<Data <VecCoord> >& xfree = *m_state->read(core::VecCoordId::freePosition());
//    const TriangleInfo & tinfo = m_triangle_info[pinfo.eid];

//    const Vector3 & p300_Free = xfree[tinfo.id0];
//    const Vector3 & p030_Free = xfree[tinfo.id1];
//    const Vector3 & p003_Free = xfree[tinfo.id2];

//    const Vector3 & n200_Free = m_point_normal[tinfo.id0];
//    const Vector3 & n020_Free = m_point_normal[tinfo.id1];
//    const Vector3 & n002_Free = m_point_normal[tinfo.id2];

//    double w12_free = dot(p030_Free - p300_Free,n200_Free);
//    double w21_free = dot(p300_Free - p030_Free,n020_Free);
//    double w23_free = dot(p003_Free - p030_Free,n020_Free);
//    double w32_free = dot(p030_Free - p003_Free,n002_Free);
//    double w31_free = dot(p300_Free - p003_Free,n002_Free);
//    double w13_free = dot(p003_Free - p300_Free,n200_Free);

//    const Vector3 & p210_Free = (p300_Free*2.0 + p030_Free - n200_Free * w12_free) / 3.0;
//    const Vector3 & p120_Free = (p030_Free*2.0 + p300_Free - n020_Free * w21_free) / 3.0;

//    const Vector3 & p021_Free = (p030_Free*2.0 + p003_Free - n020_Free * w23_free) / 3.0;
//    const Vector3 & p012_Free = (p003_Free*2.0 + p030_Free - n002_Free * w32_free) / 3.0;

//    const Vector3 & p102_Free = (p003_Free*2.0 + p300_Free - n002_Free * w31_free) / 3.0;
//    const Vector3 & p201_Free = (p300_Free*2.0 + p003_Free - n200_Free * w13_free) / 3.0;

//    const Vector3 & E_Free = (p210_Free+p120_Free+p102_Free+p201_Free+p021_Free+p012_Free) / 6.0;
//    const Vector3 & V_Free = (p300_Free+p030_Free+p003_Free) / 3.0;
//    const Vector3 & p111_Free =  E_Free + (E_Free-V_Free) / 2.0;

//    return p300_Free *   fact_w*fact_w*fact_w +
//           p030_Free *   fact_u*fact_u*fact_u +
//           p003_Free *   fact_v*fact_v*fact_v +
//           p210_Free * 3*fact_w*fact_w*fact_u +
//           p120_Free * 3*fact_w*fact_u*fact_u +
//           p201_Free * 3*fact_w*fact_w*fact_v +
//           p021_Free * 3*fact_u*fact_u*fact_v +
//           p102_Free * 3*fact_w*fact_v*fact_v +
//           p012_Free * 3*fact_u*fact_v*fact_v +
//           p111_Free * 6*fact_w*fact_u*fact_v;
//}

//bool SurfaceCubicInterpolation::fillProximity(const Vector3 & P, Proximity & pinfo){
//    prepareDetection();

//    (f_use_bezier.getValue()) ? processCollisionDetectionWithBezierCurve(P,pinfo) : processCollisionDetectionWithTriangle(P,pinfo);

//    return dot(m_triangle_normal[pinfo.eid],getContactPosition(pinfo) - P) < 0;
//}


//void SurfaceCubicInterpolation::draw(const core::visual::VisualParams * vparams) {
//    if (!d_draw.getValue()) return;

//    //build edges
//    if (f_draw_bezier_inc.getValue()>0.0) {
//        glColor4f(0.0f,0.0f,1.0f,1.0f);
//        glBegin(GL_POINTS);
//        for(int t=0;t<this->m_container->getNbTriangles();t++) {
//            const TriangleInfo & tinfo = m_triangle_info[t];
//            const BezierTriangleInfo & tbinfo = m_beziertriangle_info[t];


//            double inc = f_draw_bezier_inc.getValue();
//            for (double fact_u=0.0;fact_u<1.0;fact_u+=inc) {
//                for (double fact_v=0.0;fact_v<1.0;fact_v+=inc) {
//                    if (fact_u+fact_v>1.0) continue;
//                    double fact_w=1.0-(fact_u+fact_v);

//                    Vector3 Q = tinfo.p300 *   fact_w*fact_w*fact_w +
//                                tinfo.p030 *   fact_u*fact_u*fact_u +
//                                tinfo.p003 *   fact_v*fact_v*fact_v +
//                                tbinfo.p210 * 3*fact_w*fact_w*fact_u +
//                                tbinfo.p120 * 3*fact_w*fact_u*fact_u +
//                                tbinfo.p201 * 3*fact_w*fact_w*fact_v +
//                                tbinfo.p021 * 3*fact_u*fact_u*fact_v +
//                                tbinfo.p102 * 3*fact_w*fact_v*fact_v +
//                                tbinfo.p012 * 3*fact_u*fact_v*fact_v +
//                                tbinfo.p111 * 6*fact_w*fact_u*fact_v;

////                    Vector3 N = tinfo.n200 * fact_w*fact_w +
////                                tinfo.n020 * fact_u*fact_u +
////                                tinfo.n002 * fact_v*fact_v +
////                                tbinfo.n110 * fact_w*fact_u +
////                                tbinfo.n011 * fact_u*fact_v +
////                                tbinfo.n101 * fact_w*fact_v;

//                    helper::gl::glVertexT(Q);
//                }
//            }
//        }
//        glEnd();
//    }

//    for(unsigned t=0;t<m_triangle_info.size();t++) {
//        vparams->drawTool()->drawArrow(m_triangle_info[t].p300,m_triangle_info[t].p300 + m_triangle_info[t].n200,0.01, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
//        vparams->drawTool()->drawArrow(m_triangle_info[t].p030,m_triangle_info[t].p030 + m_triangle_info[t].n020,0.01, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));
//        vparams->drawTool()->drawArrow(m_triangle_info[t].p003,m_triangle_info[t].p003 + m_triangle_info[t].n002,0.01, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));

//    }

//}

//} //controller

//} //component

//}//Sofa

//#endif
