#ifndef SOFA_COMPONENT_BEZIERTRIANGLENONLINEARINTERPOLATION_INL
#define SOFA_COMPONENT_BEZIERTRIANGLENONLINEARINTERPOLATION_INL

#include "BezierTriangleNonLinearInterpolation.h"

namespace sofa {

namespace core {

namespace behavior {


template<class DataTypes>
BezierTriangleNonLinearInterpolation<DataTypes>::BezierTriangleNonLinearInterpolation()
: Inherit()
{}

template<class DataTypes>
void BezierTriangleNonLinearInterpolation<DataTypes>::prepareDetection() {
    Inherit::prepareDetection();

    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    m_beziertriangle_info.resize(this->getTopology()->getNbTriangles());
    for (int t=0;t<this->getTopology()->getNbTriangles();t++) {
        BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[t];
        const topology::Triangle trpids = this->getTopology()->getTriangle(t);

        const Vector3 & p300 = x[trpids[2]];
        const Vector3 & p030 = x[trpids[1]];
        const Vector3 & p003 = x[trpids[0]];

        const Vector3 & n200 = this->m_pointNormal[trpids[2]];
        const Vector3 & n020 = this->m_pointNormal[trpids[1]];
        const Vector3 & n002 = this->m_pointNormal[trpids[0]];

        double w12 = dot(p030 - p300,n200);
        double w21 = dot(p300 - p030,n020);
        double w23 = dot(p003 - p030,n020);
        double w32 = dot(p030 - p003,n002);
        double w31 = dot(p300 - p003,n002);
        double w13 = dot(p003 - p300,n200);

        tbinfo.p210 = (p300*2.0 + p030 - n200 * w12) / 3.0;
        tbinfo.p120 = (p030*2.0 + p300 - n020 * w21) / 3.0;

        tbinfo.p021 = (p030*2.0 + p003 - n020 * w23) / 3.0;
        tbinfo.p012 = (p003*2.0 + p030 - n002 * w32) / 3.0;

        tbinfo.p102 = (p003*2.0 + p300 - n002 * w31) / 3.0;
        tbinfo.p201 = (p300*2.0 + p003 - n200 * w13) / 3.0;

        Vector3 E = (tbinfo.p210+tbinfo.p120+tbinfo.p102+tbinfo.p201+tbinfo.p021+tbinfo.p012) / 6.0;
        Vector3 V = (p300+p030+p003) / 3.0;
        tbinfo.p111 =  E + (E-V) / 2.0;

        //Compute Bezier Normals
        double v12 = 2 * dot(p030-p300,n200+n020) / dot(p030-p300,p030-p300);
        double v23 = 2 * dot(p003-p030,n020+n002) / dot(p003-p030,p003-p030);
        double v31 = 2 * dot(p300-p003,n002+n200) / dot(p300-p003,p300-p003);

        Vector3 h110 = n200 + n020 - (p030-p300) * v12;
        Vector3 h011 = n020 + n002 - (p003-p030) * v23;
        Vector3 h101 = n002 + n200 - (p300-p003) * v31;

        tbinfo.n110 = h110 / h110.norm();
        tbinfo.n011 = h011 / h011.norm();
        tbinfo.n101 = h101 / h101.norm();
    }
}


////Bezier triangle are computed according to :
////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
template<class DataTypes>
typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
BezierTriangleNonLinearInterpolation<DataTypes>::getPosition(const ConstraintProximity & pinfo) {
    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[pinfo.getEid()];

    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    const Vector3 & p300 = x[pinfo.m_pid[2]];
    const Vector3 & p030 = x[pinfo.m_pid[1]];
    const Vector3 & p003 = x[pinfo.m_pid[0]];

    double fact_w = pinfo.m_fact[2];
    double fact_u = pinfo.m_fact[1];
    double fact_v = pinfo.m_fact[0];

    return p300 *   fact_w*fact_w*fact_w +
           p030 *   fact_u*fact_u*fact_u +
           p003 *   fact_v*fact_v*fact_v +
           tbinfo.p210 * 3*fact_w*fact_w*fact_u +
           tbinfo.p120 * 3*fact_w*fact_u*fact_u +
           tbinfo.p201 * 3*fact_w*fact_w*fact_v +
           tbinfo.p021 * 3*fact_u*fact_u*fact_v +
           tbinfo.p102 * 3*fact_w*fact_v*fact_v +
           tbinfo.p012 * 3*fact_u*fact_v*fact_v +
           tbinfo.p111 * 6*fact_w*fact_u*fact_v;
}


template<class DataTypes>
typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
BezierTriangleNonLinearInterpolation<DataTypes>::getFreePosition(const ConstraintProximity & pinfo) {
    double fact_w = pinfo.m_fact[2];
    double fact_u = pinfo.m_fact[1];
    double fact_v = pinfo.m_fact[0];

    const helper::ReadAccessor<Data <VecCoord> >& xfree = *this->getMstate()->read(core::VecCoordId::freePosition());

    const Vector3 & p300_Free = xfree[pinfo.m_pid[2]];
    const Vector3 & p030_Free = xfree[pinfo.m_pid[1]];
    const Vector3 & p003_Free = xfree[pinfo.m_pid[0]];

    const Vector3 & n200_Free = this->m_pointNormal[pinfo.m_pid[2]];
    const Vector3 & n020_Free = this->m_pointNormal[pinfo.m_pid[1]];
    const Vector3 & n002_Free = this->m_pointNormal[pinfo.m_pid[0]];

    double w12_free = dot(p030_Free - p300_Free,n200_Free);
    double w21_free = dot(p300_Free - p030_Free,n020_Free);
    double w23_free = dot(p003_Free - p030_Free,n020_Free);
    double w32_free = dot(p030_Free - p003_Free,n002_Free);
    double w31_free = dot(p300_Free - p003_Free,n002_Free);
    double w13_free = dot(p003_Free - p300_Free,n200_Free);

    const Vector3 & p210_Free = (p300_Free*2.0 + p030_Free - n200_Free * w12_free) / 3.0;
    const Vector3 & p120_Free = (p030_Free*2.0 + p300_Free - n020_Free * w21_free) / 3.0;

    const Vector3 & p021_Free = (p030_Free*2.0 + p003_Free - n020_Free * w23_free) / 3.0;
    const Vector3 & p012_Free = (p003_Free*2.0 + p030_Free - n002_Free * w32_free) / 3.0;

    const Vector3 & p102_Free = (p003_Free*2.0 + p300_Free - n002_Free * w31_free) / 3.0;
    const Vector3 & p201_Free = (p300_Free*2.0 + p003_Free - n200_Free * w13_free) / 3.0;

    const Vector3 & E_Free = (p210_Free+p120_Free+p102_Free+p201_Free+p021_Free+p012_Free) / 6.0;
    const Vector3 & V_Free = (p300_Free+p030_Free+p003_Free) / 3.0;
    const Vector3 & p111_Free =  E_Free + (E_Free-V_Free) / 2.0;

    return p300_Free *   fact_w*fact_w*fact_w +
           p030_Free *   fact_u*fact_u*fact_u +
           p003_Free *   fact_v*fact_v*fact_v +
           p210_Free * 3*fact_w*fact_w*fact_u +
           p120_Free * 3*fact_w*fact_u*fact_u +
           p201_Free * 3*fact_w*fact_w*fact_v +
           p021_Free * 3*fact_u*fact_u*fact_v +
           p102_Free * 3*fact_w*fact_v*fact_v +
           p012_Free * 3*fact_u*fact_v*fact_v +
           p111_Free * 6*fact_w*fact_u*fact_v;
}

template<class DataTypes>
defaulttype::Vector3 BezierTriangleNonLinearInterpolation<DataTypes>::getSurfaceNormal(const ConstraintProximity & pinfo) {
    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[pinfo.getEid()];

    const Vector3 &n200 = this->m_pointNormal[pinfo.m_pid[2]];
    const Vector3 &n020 = this->m_pointNormal[pinfo.m_pid[1]];
    const Vector3 &n002 = this->m_pointNormal[pinfo.m_pid[0]];

    double fact_w = pinfo.m_fact[2];
    double fact_u = pinfo.m_fact[1];
    double fact_v = pinfo.m_fact[0];

    Vector3 normal = n200 * fact_w*fact_w +
                     n020 * fact_u*fact_u +
                     n002 * fact_v*fact_v +
                     tbinfo.n110 * fact_w*fact_u +
                     tbinfo.n011 * fact_u*fact_v +
                     tbinfo.n101 * fact_w*fact_v;

    Vector3 N1 = normal;
    N1.normalize();

    return N1;
}


//template<class DataTypes>
//typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
//BezierTriangleNonLinearInterpolation<DataTypes>::getdpdu(const ConstraintProximity & pinfo) {
//    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

//    unsigned trid = this->m_container->getTriangleIndex(pinfo.pid[0],pinfo.pid[1],pinfo.pid[2]);
//    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[trid];
//    const Vector3 & pointsP300 = x[pinfo.pid[2]];
//    const Vector3 & pointsP030 = x[pinfo.pid[1]];
//    const Vector3 & pointsP003 = x[pinfo.pid[0]];

//    const Vector3 & pointsP210 = tbinfo.p210;
//    const Vector3 & pointsP120 = tbinfo.p120;
//    const Vector3 & pointsP201 = tbinfo.p201;
//    const Vector3 & pointsP021 = tbinfo.p021;
//    const Vector3 & pointsP102 = tbinfo.p102;
//    const Vector3 & pointsP012 = tbinfo.p012;
//    const Vector3 & pointsP111 = tbinfo.p111;

//    double fact_w = pinfo.fact[2];
//    double fact_u = pinfo.fact[1];
//    double fact_v = pinfo.fact[0];

//    return -3.0 * pointsP300 * (-fact_u - fact_v + 1)*(-fact_u - fact_v + 1)
//           +3.0 * pointsP030 * fact_u * fact_u

//           -3.0 * pointsP210 * (2.0 * fact_u * (-fact_u - fact_v + 1) - (fact_u - fact_v + 1) * (-fact_u - fact_v + 1))
//           -3.0 * pointsP120 * (fact_u * fact_u - 2*fact_u * (-fact_u - fact_v + 1))
//           -6.0 * pointsP201 * fact_v * (-fact_u - fact_v + 1)
//           +6.0 * pointsP021 * fact_u * fact_v
//           -3.0 * pointsP102 * fact_v * fact_v
//           +3.0 * pointsP012 * fact_v * fact_v
//           -6.0 * pointsP111 * (fact_u * fact_v - fact_v * (-fact_u - fact_v + 1));
//}

//template<class DataTypes>
//typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
//BezierTriangleNonLinearInterpolation<DataTypes>::getdpdv(const ConstraintProximity & pinfo) {
//    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

//    unsigned trid = this->m_container->getTriangleIndex(pinfo.pid[0],pinfo.pid[1],pinfo.pid[2]);
//    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[trid];


//    const Vector3 & pointsP300 = x[pinfo.pid[2]];
//    const Vector3 & pointsP030 = x[pinfo.pid[1]];
//    const Vector3 & pointsP003 = x[pinfo.pid[0]];

//    const Vector3 & pointsP210 = tbinfo.p210;
//    const Vector3 & pointsP120 = tbinfo.p120;
//    const Vector3 & pointsP201 = tbinfo.p201;
//    const Vector3 & pointsP021 = tbinfo.p021;
//    const Vector3 & pointsP102 = tbinfo.p102;
//    const Vector3 & pointsP012 = tbinfo.p012;
//    const Vector3 & pointsP111 = tbinfo.p111;

//    double fact_w = pinfo.fact[2];
//    double fact_u = pinfo.fact[1];
//    double fact_v = pinfo.fact[0];

//    return -3.0 * pointsP300 * (-fact_u - fact_v + 1) * (-fact_u - fact_v + 1)

//           +3.0 * pointsP003 * fact_v * fact_v
//           -6.0 * pointsP210 * (-fact_u - fact_v + 1) * fact_u
//           -3.0 * pointsP120 * fact_u * fact_u
//           -3.0 * pointsP201 * (2.0 * fact_v * (-fact_u - fact_v + 1) - (-fact_u - fact_v + 1) * (-fact_u - fact_v + 1))
//           +3.0 * pointsP021 * fact_u * fact_u
//           -3.0 * pointsP102 * (fact_v * fact_v - 2.0 * fact_v * (-fact_u - fact_v + 1))
//           +6.0 * pointsP012 * fact_u * fact_v
//           -6.0 * pointsP111 * (fact_u * fact_v - fact_u * (-fact_u - fact_v + 1));
//}

//template<class DataTypes>
//typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
//BezierTriangleNonLinearInterpolation<DataTypes>::getd2pdu2(const ConstraintProximity & pinfo) {
//    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

//    unsigned trid = this->m_container->getTriangleIndex(pinfo.pid[0],pinfo.pid[1],pinfo.pid[2]);
//    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[trid];
//    const Vector3 & p300 = x[pinfo.pid[2]];
//    const Vector3 & p030 = x[pinfo.pid[1]];

//    double fact_w = pinfo.fact[2];
//    double fact_u = pinfo.fact[1];
//    double fact_v = pinfo.fact[0];

//    return  6.0  * p300 * (-fact_u - fact_v + 1)
//           +6.0  * p030 * fact_u

//           +6.0  * tbinfo.p210 * (3.0 * fact_u + 2.0 * fact_v - 2.0)
//           -6.0  * tbinfo.p120 * (3.0 * fact_u + fact_v - 1)
//           +6.0  * tbinfo.p201 * fact_v
//           +6.0  * tbinfo.p021 * fact_v


//           -12.0 * tbinfo.p111 * fact_v;
//}

//template<class DataTypes>
//typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
//BezierTriangleNonLinearInterpolation<DataTypes>::getd2pdv2(const ConstraintProximity & pinfo) {
//    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

//    unsigned trid = this->m_container->getTriangleIndex(pinfo.pid[0],pinfo.pid[1],pinfo.pid[2]);
//    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[trid];
//    const Vector3 & p300 = x[pinfo.pid[2]];
//    const Vector3 & p003 = x[pinfo.pid[0]];

//    double fact_w = pinfo.fact[2];
//    double fact_u = pinfo.fact[1];
//    double fact_v = pinfo.fact[0];


//    return  6.0  * p300 * (-fact_u - fact_v + 1)

//           +6.0  * p003 * fact_v
//           +6.0  * tbinfo.p210 * fact_u

//           +6.0  * tbinfo.p201 * (2.0 * fact_u + 3.0 * fact_v - 2.0)

//           -6.0  * tbinfo.p102 * (fact_u + 3.0 * fact_v - 1.0)
//           +6.0  * tbinfo.p012 * fact_u
//           -12.0 * tbinfo.p111 * fact_u;
//}

//template<class DataTypes>
//typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
//BezierTriangleNonLinearInterpolation<DataTypes>::getd2pduv(const ConstraintProximity & pinfo) {
//    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

//    unsigned trid = this->m_container->getTriangleIndex(pinfo.pid[0],pinfo.pid[1],pinfo.pid[2]);
//    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[trid];
//    const Vector3 & p300 = x[pinfo.pid[2]];

//    double fact_w = pinfo.fact[2];
//    double fact_u = pinfo.fact[1];
//    double fact_v = pinfo.fact[0];

//    return  6.0 * p300 * (-fact_u - fact_v + 1)


//           +6.0 * tbinfo.p210 * (2.0 * fact_u + fact_v - 1.0)
//           -6.0 * tbinfo.p120 * fact_u
//           +6.0 * tbinfo.p201 * (fact_u + 2.0 * fact_v - 1.0)
//           +6.0 * tbinfo.p021 * fact_u
//           -6.0 * tbinfo.p102 * fact_v
//           +6.0 * tbinfo.p012 * fact_v
//           -6.0 * tbinfo.p111 * (2.0*fact_u + 2.0*fact_v - 1.0);
//}

//template<class DataTypes>
//typename BezierTriangleNonLinearInterpolation<DataTypes>::Vector3
//BezierTriangleNonLinearInterpolation<DataTypes>::getd2pdvu(const ConstraintProximity & pinfo) {
//    const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());

//    unsigned trid = this->m_container->getTriangleIndex(pinfo.pid[0],pinfo.pid[1],pinfo.pid[2]);
//    const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[trid];
//    const Vector3 & p300 = x[pinfo.pid[2]];

//    double fact_w = pinfo.fact[2];
//    double fact_u = pinfo.fact[1];
//    double fact_v = pinfo.fact[0];

//    return  6.0 * p300 * (-fact_u - fact_v + 1)


//           +6.0 * tbinfo.p210 * (2.0 * fact_u + fact_v - 1.0)
//           -6.0 * tbinfo.p120 * fact_u
//           +6.0 * tbinfo.p201 * (fact_u + 2.0 * fact_v - 1.0)
//           +6.0 * tbinfo.p021 * fact_u
//           -6.0 * tbinfo.p102 * fact_v
//           +6.0 * tbinfo.p012 * fact_v
//           -6.0 * tbinfo.p111 * (2.0*fact_u + 2.0*fact_v - 1.0);
//}




} //behavior

} //core

}//sofa

#endif
