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
#ifndef SOFA_COMPONENT_EDGEGEOMETRY_H
#define SOFA_COMPONENT_EDGEGEOMETRY_H

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include "ConstraintProximity.h"
#include "PointGeometry.h"

namespace sofa {

namespace core {

namespace behavior {



class EdgeGeometry : public PointGeometry
{
public:
    SOFA_CLASS(EdgeGeometry , PointGeometry );


    class EdgeConstraintProximity : public ConstraintProximity {
    public:
        friend class EdgeGeometry;

        EdgeConstraintProximity(const EdgeGeometry * geo, unsigned eid, unsigned p1, double f1,unsigned p2, double f2) : ConstraintProximity(geo) {
            m_eid = eid;

            m_pid.resize(2);
            m_fact.resize(2);

            m_pid[0] = p1;
            m_fact[0] = f1;

            m_pid[1] = p2;
            m_fact[1] = f2;
        }

        defaulttype::Vector3 getPosition() const {
            const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getMstate()->read(core::VecCoordId::position());
            return x[m_pid[0]] * m_fact[0] + x[m_pid[1]] * m_fact[1];
        }

        defaulttype::Vector3 getFreePosition() const {
            const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getMstate()->read(core::VecCoordId::freePosition());
            return x[m_pid[0]] * m_fact[0] + x[m_pid[1]] * m_fact[1];
        }


        defaulttype::Vector3 getNormal() {
            const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getMstate()->read(core::VecCoordId::position());

            defaulttype::Vector3 En = x[m_pid[1]] - x[m_pid[0]];
            defaulttype::Vector3 Z = defaulttype::Vector3(0,0,1);

            En.normalize();
            if (dot(En,Z) < 0.000000000001) Z=defaulttype::Vector3(0,1,0);

            return cross(En,Z);
        }

        void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) {
            DataMatrixDeriv & c_d = *cId[m_geo->getMstate()].write();
            MatrixDeriv & c = *c_d.beginEdit();
            MatrixDerivRowIterator c_it1 = c.writeLine(cline);
            c_it1.addCol(m_pid[0],N*m_fact[0]);
            c_it1.addCol(m_pid[1],N*m_fact[1]);
            c_d.endEdit();
        }

        void getControlPoints(helper::vector<defaulttype::Vector3> & controlPoints) {
            const helper::ReadAccessor<Data <VecCoord> > & x = m_geo->getMstate()->read(core::VecCoordId::position());
            controlPoints.push_back(x[m_pid[0]]);
            controlPoints.push_back(x[m_pid[1]]);
        }

        unsigned m_eid;
        const EdgeGeometry * m_geo;
    };

    void projectPoint(const defaulttype::Vector3 & T,EdgeConstraintProximity * pinfo) const;

    virtual int getNbEdges() const ;

    ConstraintProximityPtr getEdgeProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2) const;

    void draw(const core::visual::VisualParams * /*vparams*/);

    int getNbElements() const;

    ConstraintProximityPtr getElementProximity(unsigned eid) const;

};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
