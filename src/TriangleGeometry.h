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

#include "EdgeGeometry.h"
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa {

namespace core {

namespace behavior {

class TriangleGeometry : public EdgeGeometry
{
public:
    SOFA_CLASS(TriangleGeometry , BaseGeometry );

    class TriangleConstraintProximity : public EdgeConstraintProximity {
    public:
        TriangleConstraintProximity(const TriangleGeometry * geo, unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3)
        : EdgeConstraintProximity(geo,p1,f1,p2,f2) {
            m_eid = eid;
            m_pid.push_back(p3);
            m_fact.push_back(f3);
        }

        defaulttype::Vector3 getNormal() const {
            return ((TriangleGeometry *) m_cg)->m_triangle_info[m_eid].tn;
        }

    protected:
        unsigned m_eid;
    };

    class TrianglePhongConstraintProximity : public EdgeConstraintProximity {
    public:

        TrianglePhongConstraintProximity(const TriangleGeometry * geo, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3)
        : EdgeConstraintProximity(geo,p1,f1,p2,f2) {
            m_pid.push_back(p3);
            m_fact.push_back(f3);
        }

        defaulttype::Vector3 getNormal() const {
            return ((TriangleGeometry *) m_cg)->m_pointNormal[m_pid[0]] * m_fact[0] +
                   ((TriangleGeometry *) m_cg)->m_pointNormal[m_pid[1]] * m_fact[1] +
                   ((TriangleGeometry *) m_cg)->m_pointNormal[m_pid[2]] * m_fact[2];
        }
    };

    Data<bool> d_phong;

    TriangleGeometry();

    ConstraintProximityPtr getTriangleProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) const;

    ConstraintProximityPtr projectPoint(const defaulttype::Vector3 & s,unsigned eid) const;

    void draw(const core::visual::VisualParams */*vparams*/);

    int getNbElements() const;

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
