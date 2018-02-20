//#ifndef SOFA_COMPONENT_BEZIERTRIANGLEGEOMETRY_INL
//#define SOFA_COMPONENT_BEZIERTRIANGLEGEOMETRY_INL

//#include "BezierTriangleGeometry.h"
//#include "ConstraintProximity.h"

//namespace sofa {

//namespace core {

//namespace behavior {

//BezierTriangleGeometry::BezierTriangleGeometry()
//: Inherit()
//, d_nonlin_max_it(initData(&d_nonlin_max_it, (unsigned) 20,"nonlin_max_it","Max iteration in the Newton Raphson solver used for projection of points on non linear triangle"))
//, d_nonlin_tolerance(initData(&d_nonlin_tolerance, (double) 0.001,"nonlin_tol","Tolerance in the Newton Raphson solver used for projection of points on non linear triangle"))
//, d_nonlin_threshold(initData(&d_nonlin_threshold, (double) 0.00001,"nonlin_th","Threshold in the Newton Raphson solver used for projection of points on non linear triangle"))
//, d_draw_tesselation(initData(&d_draw_tesselation, (unsigned) 0.0,"tesselation","Draw tesselated triangles"))
//{}

//void BezierTriangleGeometry::prepareDetection() {
//    Inherit::prepareDetection();

//    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

//    m_beziertriangle_info.resize(this->getTopology()->getNbTriangles());
//    for (int t=0;t<this->getTopology()->getNbTriangles();t++) {
//        BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[t];
//        const sofa::core::topology::BaseMeshTopology::Triangle trpids = this->getTopology()->getTriangle(t);

//        const Vector3 & p300 = x[trpids[2]];
//        const Vector3 & p030 = x[trpids[1]];
//        const Vector3 & p003 = x[trpids[0]];

//        const Vector3 & n200 = this->m_pointNormal[trpids[2]];
//        const Vector3 & n020 = this->m_pointNormal[trpids[1]];
//        const Vector3 & n002 = this->m_pointNormal[trpids[0]];

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

//ConstraintProximityPtr BezierTriangleGeometry::getNonLinearTriangleProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) const {
//    return std::make_shared<BezierConstraintProximity>(this, eid, p1,f1,p2,f2,p3,f3);
//}

//ConstraintProximityPtr BezierTriangleGeometry::getElementProximity(unsigned eid) const {
//    const sofa::core::topology::BaseMeshTopology::Triangle tri = getTopology()->getTriangle(eid);
//    return getNonLinearTriangleProximity(eid, tri[0],0.3333,tri[1],0.3333,tri[2],0.3333);
//}

//void BezierTriangleGeometry::projectPoint(const defaulttype::Vector3 & P, BezierConstraintProximity *pfrom) const {
//    TriangleConstraintProximity plinear(this,pfrom->m_eid,pfrom->m_pid[0],pfrom->m_fact[0],pfrom->m_pid[1],pfrom->m_fact[1],pfrom->m_pid[2],pfrom->m_fact[2]);
//    TriangleGeometry::projectPoint(P,&plinear);
//    pfrom->m_fact[0] = plinear.getFact()[0];
//    pfrom->m_fact[1] = plinear.getFact()[1];
//    pfrom->m_fact[2] = plinear.getFact()[2];
////    ConstraintProximityPtr((ConstraintProximity *)pfrom)->refineToClosestPoint(P);

//    unsigned max_it = d_nonlin_max_it.getValue();
//    double tolerance = d_nonlin_tolerance.getValue();
//    double threshold = d_nonlin_threshold.getValue();

//    unsigned int it=0;
//    double delta = 0.00001;

//    BezierConstraintProximity pinfo(this,pfrom->m_eid,pfrom->m_pid[0],pfrom->m_fact[0],pfrom->m_pid[1],pfrom->m_fact[1],pfrom->m_pid[2],pfrom->m_fact[2]);

//    int p0 = pinfo.m_pid[0];
//    int p1 = pinfo.m_pid[1];
//    int p2 = pinfo.m_pid[2];

//    while(it< max_it) {
//        defaulttype::Vector3 Q = pinfo.getPosition();

//        defaulttype::Vector3 nQP = P - Q;
//        if (nQP.norm() < tolerance) break;
//        nQP.normalize();

//        defaulttype::Vector3 N1 = pinfo.getNormal();
//        N1.normalize();

//        if (pinfo.m_fact[0] < 0 || pinfo.m_fact[1] < 0 || pinfo.m_fact[2] < 0) break;

//        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(1,0,0)))>0.99) ? defaulttype::Vector3(0,1,0) : defaulttype::Vector3(1,0,0)));
//        N2.normalize();

//        defaulttype::Vector3 N3 = cross(N1,N2);
//        N3.normalize();

//        defaulttype::Vector2 e_0(dot(nQP,N2),dot(nQP,N3));

//        if(e_0.norm() < tolerance) break;

//        double fact_u = (pinfo.m_fact[2] - delta < 0.0 || pinfo.m_fact[1] + delta > 1.0) ? -1.0 : 1.0;
//        double fact_v = (pinfo.m_fact[2] - delta < 0.0 || pinfo.m_fact[0] + delta > 1.0) ? -1.0 : 1.0;

//        //variation point along v
//        double P_v_fact0 = pinfo.m_fact[0] + delta * fact_v;
//        double P_v_fact1 = pinfo.m_fact[1];
//        double P_v_fact2 = pinfo.m_fact[2] - delta * fact_v;

//        BezierConstraintProximity P_v(this,pinfo.m_eid,p0,P_v_fact0,p1,P_v_fact1,p2,P_v_fact2);
//        defaulttype::Vector3 p_v = (P - P_v.getPosition()).normalized();
//        defaulttype::Vector2 e_v(dot(p_v,N2)*fact_v,dot(p_v,N3)*fact_v);

//        //variation point along u
//        double P_u_fact0 = pinfo.m_fact[0];
//        double P_u_fact1 = pinfo.m_fact[1] + delta * fact_u;
//        double P_u_fact2 = pinfo.m_fact[2] - delta * fact_u;
//        BezierConstraintProximity P_u(this,pinfo.m_eid,p0,P_u_fact0,p1,P_u_fact1,p2,P_u_fact2);
//        defaulttype::Vector3 p_u = (P - P_u.getPosition()).normalized();
//        defaulttype::Vector2 e_u(dot(p_u,N2)*fact_u,dot(p_u,N3)*fact_u);

//        if (P_v.m_fact[0] < 0 || P_v.m_fact[1] < 0 || P_v.m_fact[2] < 0) break;
//        if (P_u.m_fact[0] < 0 || P_u.m_fact[1] < 0 || P_u.m_fact[2] < 0) break;

//        defaulttype::Mat<2,2,double> J, invJ;
//        J[0][0] = (e_v[0] - e_0[0])/delta;
//        J[1][0] = (e_v[1] - e_0[1])/delta;
//        J[0][1] = (e_u[0] - e_0[0])/delta;
//        J[1][1] = (e_u[1] - e_0[1])/delta;

//        invertMatrix(invJ, J);

//        // dUV is the optimal direction
//        defaulttype::Vector2 dUV = -invJ * e_0;
//        if(dUV.norm() < threshold) break;

//        //bary coords of the solution of the 2D problem
//        double sol_v = pinfo.m_fact[0] + dUV[0];
//        double sol_u = pinfo.m_fact[1] + dUV[1];
//        double sol_w = 1.0 - sol_u - sol_v;

//        // we now search what is the optimal displacmeent along this path
//        defaulttype::Vector3 dir2d(sol_v - pinfo.m_fact[0],
//                                   sol_u - pinfo.m_fact[1],
//                                   sol_w - pinfo.m_fact[2]);

//        if(dir2d.norm() < threshold) break;

//        //we apply a small perturbation arond the 2d direction
//        dir2d.normalize();

//        double fact_a = (pinfo.m_fact[0] + dir2d[0] * delta < 0 || pinfo.m_fact[1] + dir2d[1] * delta < 0 || pinfo.m_fact[2] + dir2d[2] * delta < 0) ? -1.0 : 1.0;
//        double P_a_fact0 = pinfo.m_fact[0] + dir2d[0] * delta * fact_a;
//        double P_a_fact1 = pinfo.m_fact[1] + dir2d[1] * delta * fact_a;
//        double P_a_fact2 = pinfo.m_fact[2] + dir2d[2] * delta * fact_a;
//        BezierConstraintProximity P_a(this,pinfo.m_eid,p0,P_a_fact0,p1,P_a_fact1,p2,P_a_fact2);

//        if (P_a.m_fact[0] < 0 || P_a.m_fact[1] < 0 || P_a.m_fact[2] < 0) break;

//        defaulttype::Vector3 QA = P_a.getPosition();

//        double fact;
//        if (fabs(dot(nQP,N1))>0.8) {
//            double fx = acos(fabs(dot(nQP,N1)));
//            double fxdx = acos(fabs(dot((P - QA).normalized(),P_a.getNormal())));
//            double j = (fxdx - fx) / delta;
//            fact = -fx / j;
//        } else {
//            defaulttype::Vector3 nQA = (Q-QA).normalized();
//            double fx = dot(P-Q, nQA);
//            double fxdx = dot(P-QA, nQA);
//            double j = (fxdx - fx) / delta;
//            fact = -fx / j;
//        }

//        if(fabs(fact) < threshold) break;

//        dir2d *= fact * fact_a;

//        double new_v = pinfo.m_fact[0] + dir2d[0];
//        double new_u = pinfo.m_fact[1] + dir2d[1];
//        double new_w = pinfo.m_fact[2] + dir2d[2];

//        if (new_v<0 && fabs(dir2d[0])>0) dir2d *= -pinfo.m_fact[0] / dir2d[0];
//        if (new_u<0 && fabs(dir2d[1])>0) dir2d *= -pinfo.m_fact[1] / dir2d[1];
//        if (new_w<0 && fabs(dir2d[2])>0) dir2d *= -pinfo.m_fact[2] / dir2d[2];

//        pinfo.m_fact[0] += dir2d[0];
//        pinfo.m_fact[1] += dir2d[1];
//        pinfo.m_fact[2] += dir2d[2];

//        it++;
//    }

//    pfrom->m_fact[0] = pinfo.m_fact[0];
//    pfrom->m_fact[1] = pinfo.m_fact[1];
//    pfrom->m_fact[2] = pinfo.m_fact[2];
//}

//void BezierTriangleGeometry::tesselate(const core::visual::VisualParams * vparams, unsigned level,int tid, const defaulttype::Vector3 & bary_A,const defaulttype::Vector3 & bary_B, const defaulttype::Vector3 & bary_C) {
//    if (level >= d_draw_tesselation.getValue()) {
//        const topology::BaseMeshTopology::Triangle & tri = this->getTopology()->getTriangle(tid);

//        defaulttype::Vector3 pA = getNonLinearTriangleProximity(tid,tri[0],bary_A[0],tri[1],bary_A[1],tri[2],bary_A[2])->getPosition();
//        defaulttype::Vector3 pB = getNonLinearTriangleProximity(tid,tri[0],bary_B[0],tri[1],bary_B[1],tri[2],bary_B[2])->getPosition();
//        defaulttype::Vector3 pC = getNonLinearTriangleProximity(tid,tri[0],bary_C[0],tri[1],bary_C[1],tri[2],bary_C[2])->getPosition();

//        this->drawTriangle(vparams,pA,pB,pC);

//        return;
//    }

//    defaulttype::Vector3 bary_D = (bary_A + bary_B)/2.0;
//    defaulttype::Vector3 bary_E = (bary_A + bary_C)/2.0;
//    defaulttype::Vector3 bary_F = (bary_B + bary_C)/2.0;

//    defaulttype::Vector3 bary_G = (bary_A + bary_B + bary_C)/3.0;

//    tesselate(vparams,level+1,tid,bary_A,bary_D,bary_G);
//    tesselate(vparams,level+1,tid,bary_D,bary_B,bary_G);

//    tesselate(vparams,level+1,tid,bary_G,bary_B,bary_F);
//    tesselate(vparams,level+1,tid,bary_G,bary_F,bary_C);

//    tesselate(vparams,level+1,tid,bary_G,bary_C,bary_E);
//    tesselate(vparams,level+1,tid,bary_A,bary_G,bary_E);
//}

//void BezierTriangleGeometry::draw(const core::visual::VisualParams * vparams) {
//        if (! vparams->displayFlags().getShowCollisionModels()) return;

//        if (m_triangle_info.empty()) {
//            Inherit::draw((vparams));
//            return;
//        }

//        glDisable(GL_LIGHTING);

//        if (vparams->displayFlags().getShowWireFrame()) glBegin(GL_LINES);
//        else {
//            glEnable(GL_CULL_FACE);
//            glBegin(GL_TRIANGLES);
//        }

//        for(int t=0;t<this->getTopology()->getNbTriangles();t++) {
//            tesselate(vparams,0,t,defaulttype::Vector3(1,0,0),defaulttype::Vector3(0,1,0),defaulttype::Vector3(0,0,1));
//        }

//        glEnd();
//    }

//} //behavior

//} //core

//}//sofa

//#endif
