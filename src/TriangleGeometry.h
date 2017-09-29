/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_TRIANGLEGEOMETRY_H
#define SOFA_COMPONENT_TRIANGLEGEOMETRY_H

#include "ConstraintProximity.h"
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>
#include "EdgeGeometry.h"

namespace sofa {

namespace core {

namespace behavior {

class TriangleGeometry : public EdgeGeometry
{
public:
    SOFA_CLASS(TriangleGeometry , EdgeGeometry );

    class TriangleConstraintProximity : public ConstraintProximity {
    public:
        friend class TriangleGeometry;

        TriangleConstraintProximity(const TriangleGeometry * geo,unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) : ConstraintProximity(geo) {
            m_eid = eid;

            m_pid.resize(3);
            m_fact.resize(3);

            m_pid[0] = p1;
            m_fact[0] = f1;

            m_pid[1] = p2;
            m_fact[1] = f2;

            m_pid[2] = p3;
            m_fact[2] = f3;
        }

        defaulttype::Vector3 getPosition() const {
            const helper::ReadAccessor<Data <VecCoord> > & x = m_geo->getMstate()->read(core::VecCoordId::position());
            return x[m_pid[0]] * m_fact[0] + x[m_pid[1]] * m_fact[1] + x[m_pid[2]] * m_fact[2];
        }

        defaulttype::Vector3 getFreePosition() const {
            const helper::ReadAccessor<Data <VecCoord> > & x = m_geo->getMstate()->read(core::VecCoordId::freePosition());
            return x[m_pid[0]] * m_fact[0] + x[m_pid[1]] * m_fact[1] + x[m_pid[2]] * m_fact[2];
        }

        defaulttype::Vector3 getNormal() {
            return ((const TriangleGeometry *)m_geo)->m_triangle_info[m_eid].tn;
        }

        void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) {
            DataMatrixDeriv & c_d = *cId[m_geo->getMstate()].write();
            MatrixDeriv & c = *c_d.beginEdit();
            MatrixDerivRowIterator c_it1 = c.writeLine(cline);
            c_it1.addCol(m_pid[0],N*m_fact[0]);
            c_it1.addCol(m_pid[1],N*m_fact[1]);
            c_it1.addCol(m_pid[2],N*m_fact[2]);
            c_d.endEdit();
        }

        void getControlPoints(helper::vector<defaulttype::Vector3> & controlPoints) {
            const helper::ReadAccessor<Data <VecCoord> > & x = m_geo->getMstate()->read(core::VecCoordId::position());
            controlPoints.push_back(x[m_pid[0]]);
            controlPoints.push_back(x[m_pid[1]]);
            controlPoints.push_back(x[m_pid[2]]);
        }

        unsigned m_eid;
    };

    Data<bool> d_phong;

    TriangleGeometry();

    ConstraintProximityPtr getTriangleProximity(unsigned eid, unsigned p1, double f1, unsigned p2, double f2, unsigned p3, double f3) const;

    void projectPoint(const defaulttype::Vector3 & s,TriangleConstraintProximity * pinfo) const;

    void draw(const core::visual::VisualParams */*vparams*/);

    int getNbTriangles() const;

    int getNbElements() const;

    ConstraintProximityPtr getElementProximity(unsigned eid) const;

protected:

    typedef struct {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 tn,ax1,ax2;
    } TriangleInfo;

    virtual void prepareDetection();

    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_w,double & fact_u, double & fact_v) const;

    helper::vector<TriangleInfo> m_triangle_info;
    helper::vector<defaulttype::Vector3> m_pointNormal;

    void drawTriangle(const core::visual::VisualParams * vparams,const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C);
};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
