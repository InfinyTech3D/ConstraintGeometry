///******************************************************************************
//*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
//*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
//*                                                                             *
//* This library is free software; you can redistribute it and/or modify it     *
//* under the terms of the GNU Lesser General Public License as published by    *
//* the Free Software Foundation; either version 2.1 of the License, or (at     *
//* your option) any later version.                                             *
//*                                                                             *
//* This library is distributed in the hope that it will be useful, but WITHOUT *
//* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
//* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
//* for more details.                                                           *
//*                                                                             *
//* You should have received a copy of the GNU Lesser General Public License    *
//* along with this library; if not, write to the Free Software Foundation,     *
//* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
//*******************************************************************************
//*                               SOFA :: Modules                               *
//*                                                                             *
//* Authors: The SOFA Team and external contributors (see Authors.txt)          *
//*                                                                             *
//* Contact information: contact@sofa-framework.org                             *
//******************************************************************************/
//#ifndef SOFA_COMPONENT_BEZIERTRIANGLEGEOMETRY_H
//#define SOFA_COMPONENT_BEZIERTRIANGLEGEOMETRY_H

//#include "TriangleGeometry.h"

//namespace sofa {

//namespace core {

//namespace behavior {

//class BezierTriangleGeometry : public TriangleGeometry
//{
//public:
//    SOFA_CLASS(BezierTriangleGeometry , TriangleGeometry );

//    typedef TriangleGeometry Inherit;
//    typedef typename Inherit::TriangleInfo TriangleInfo;

//    typedef typename defaulttype::Vector2 Vector2;
//    typedef typename defaulttype::Vector3 Vector3;
//    typedef typename DataTypes::Coord Coord;
//    typedef typename DataTypes::Real Real;
//    typedef typename DataTypes::VecCoord VecCoord;
//    typedef typename DataTypes::VecDeriv VecDeriv;

//    Data <unsigned> d_nonlin_max_it;
//    Data <double> d_nonlin_tolerance;
//    Data <double> d_nonlin_threshold;
//    Data <unsigned> d_draw_tesselation;

//    BezierTriangleGeometry();

//    class BezierConstraintProximity : public ConstraintProximity {
//    public:
//        friend class BezierTriangleGeometry;

//        BezierConstraintProximity(const BezierTriangleGeometry * geo, unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) : ConstraintProximity(geo) {
//            m_eid = eid;

//            m_pid.resize(3);
//            m_fact.resize(3);

//            m_pid[0] = p1;
//            m_fact[0] = f1;

//            m_pid[1] = p2;
//            m_fact[1] = f2;

//            m_pid[2] = p3;
//            m_fact[2] = f3;
//        }

//        ////Bezier triangle are computed according to :
//        ////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
//        defaulttype::Vector3 getPosition() const {
//            const BezierTriangleInfo & tbinfo = ((BezierTriangleGeometry *)m_geo)->m_beziertriangle_info[m_eid];
//            const helper::ReadAccessor<Data <VecCoord> > & x = ((BezierTriangleGeometry *)m_geo)->getMstate()->read(core::VecCoordId::position());

//            const Vector3 & p300 = x[m_pid[2]];
//            const Vector3 & p030 = x[m_pid[1]];
//            const Vector3 & p003 = x[m_pid[0]];

//            double fact_w = m_fact[2];
//            double fact_u = m_fact[1];
//            double fact_v = m_fact[0];

//            return p300 *   fact_w*fact_w*fact_w +
//                   p030 *   fact_u*fact_u*fact_u +
//                   p003 *   fact_v*fact_v*fact_v +
//                   tbinfo.p210 * 3*fact_w*fact_w*fact_u +
//                   tbinfo.p120 * 3*fact_w*fact_u*fact_u +
//                   tbinfo.p201 * 3*fact_w*fact_w*fact_v +
//                   tbinfo.p021 * 3*fact_u*fact_u*fact_v +
//                   tbinfo.p102 * 3*fact_w*fact_v*fact_v +
//                   tbinfo.p012 * 3*fact_u*fact_v*fact_v +
//                   tbinfo.p111 * 6*fact_w*fact_u*fact_v;
//        }

//        defaulttype::Vector3 getFreePosition() const {
//            double fact_w = m_fact[2];
//            double fact_u = m_fact[1];
//            double fact_v = m_fact[0];

//            const helper::ReadAccessor<Data <VecCoord> > & x = ((BezierTriangleGeometry *)m_geo)->getMstate()->read(core::VecCoordId::freePosition());

//            const Vector3 & p300_Free = x[m_pid[2]];
//            const Vector3 & p030_Free = x[m_pid[1]];
//            const Vector3 & p003_Free = x[m_pid[0]];

//            const Vector3 & n200_Free = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[2]];
//            const Vector3 & n020_Free = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[1]];
//            const Vector3 & n002_Free = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[0]];

//            double w12_free = dot(p030_Free - p300_Free,n200_Free);
//            double w21_free = dot(p300_Free - p030_Free,n020_Free);
//            double w23_free = dot(p003_Free - p030_Free,n020_Free);
//            double w32_free = dot(p030_Free - p003_Free,n002_Free);
//            double w31_free = dot(p300_Free - p003_Free,n002_Free);
//            double w13_free = dot(p003_Free - p300_Free,n200_Free);

//            const Vector3 & p210_Free = (p300_Free*2.0 + p030_Free - n200_Free * w12_free) / 3.0;
//            const Vector3 & p120_Free = (p030_Free*2.0 + p300_Free - n020_Free * w21_free) / 3.0;

//            const Vector3 & p021_Free = (p030_Free*2.0 + p003_Free - n020_Free * w23_free) / 3.0;
//            const Vector3 & p012_Free = (p003_Free*2.0 + p030_Free - n002_Free * w32_free) / 3.0;

//            const Vector3 & p102_Free = (p003_Free*2.0 + p300_Free - n002_Free * w31_free) / 3.0;
//            const Vector3 & p201_Free = (p300_Free*2.0 + p003_Free - n200_Free * w13_free) / 3.0;

//            const Vector3 & E_Free = (p210_Free+p120_Free+p102_Free+p201_Free+p021_Free+p012_Free) / 6.0;
//            const Vector3 & V_Free = (p300_Free+p030_Free+p003_Free) / 3.0;
//            const Vector3 & p111_Free =  E_Free + (E_Free-V_Free) / 2.0;

//            return p300_Free *   fact_w*fact_w*fact_w +
//                   p030_Free *   fact_u*fact_u*fact_u +
//                   p003_Free *   fact_v*fact_v*fact_v +
//                   p210_Free * 3*fact_w*fact_w*fact_u +
//                   p120_Free * 3*fact_w*fact_u*fact_u +
//                   p201_Free * 3*fact_w*fact_w*fact_v +
//                   p021_Free * 3*fact_u*fact_u*fact_v +
//                   p102_Free * 3*fact_w*fact_v*fact_v +
//                   p012_Free * 3*fact_u*fact_v*fact_v +
//                   p111_Free * 6*fact_w*fact_u*fact_v;
//        }

//        Vector3 getNormal() {
//            const BezierTriangleInfo & tbinfo = ((BezierTriangleGeometry *)m_geo)->m_beziertriangle_info[m_eid];

//            const Vector3 &n200 = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[2]];
//            const Vector3 &n020 = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[1]];
//            const Vector3 &n002 = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[0]];

//            double fact_w = m_fact[2];
//            double fact_u = m_fact[1];
//            double fact_v = m_fact[0];

//            Vector3 normal = n200 * fact_w*fact_w +
//                             n020 * fact_u*fact_u +
//                             n002 * fact_v*fact_v +
//                             tbinfo.n110 * fact_w*fact_u +
//                             tbinfo.n011 * fact_u*fact_v +
//                             tbinfo.n101 * fact_w*fact_v;

//            Vector3 N1 = normal;
//            N1.normalize();

//            return N1;
//        }

//        void refineToClosestPoint(const Coord & P) {
//            ((const BezierTriangleGeometry *) m_geo)->projectPoint(P,this);
//        }

//    protected:
//        unsigned m_eid;
//    };

//    void prepareDetection();

//    ConstraintProximityPtr getNonLinearTriangleProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) const;

//    ConstraintProximityPtr getElementProximity(unsigned eid) const;

//    void draw(const core::visual::VisualParams * vparams);

//    void projectPoint(const defaulttype::Vector3 & s,BezierConstraintProximity * pinfo) const;

//private :
//    typedef struct {
//        Vector3 p210,p120,p021,p012,p102,p201,p111;
//        Vector3 n110,n011,n101;
//    } BezierTriangleInfo;

//    helper::vector<BezierTriangleInfo> m_beziertriangle_info;

//    void tesselate(const core::visual::VisualParams * vparams, unsigned level,int tid, const defaulttype::Vector3 & bary_A,const defaulttype::Vector3 & bary_B, const defaulttype::Vector3 & bary_C);

//    ConstraintProximityPtr newtonTriangularIterations(const defaulttype::Vector3 & P,unsigned eid,const ConstraintProximityPtr & pfrom,unsigned max_it, double tolerance, double threshold);

//};


//} // namespace forcefield

//} // namespace component

//} // namespace sofa


//#endif // NeedleLinearDescription_H
