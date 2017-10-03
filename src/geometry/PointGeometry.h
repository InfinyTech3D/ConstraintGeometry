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
#ifndef SOFA_COMPONENT_POINTGEOMETRY_H
#define SOFA_COMPONENT_POINTGEOMETRY_H

#include "BaseGeometry.h"
#include "ConstraintProximity.h"
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa {

namespace core {

namespace behavior {


class PointGeometry : public BaseGeometry
{
public:
    SOFA_CLASS(PointGeometry , BaseGeometry );

    class PointConstraintProximity : public ConstraintProximity {
    public:

        PointConstraintProximity(const PointGeometry * geo, unsigned pid) {
            m_geo = geo;

            m_pid.resize(1);
            m_fact.resize(1);

            m_pid[0] = pid;
            m_fact[0] = 1.0;
        }

        defaulttype::Vector3 getPosition() const {
            const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getMstate()->read(core::VecCoordId::position());
            return x[m_pid[0]];
        }

        defaulttype::Vector3 getFreePosition() const {
            const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getMstate()->read(core::VecCoordId::freePosition());
            return x[m_pid[0]];
        }

        defaulttype::Vector3 getNormal() {
            return defaulttype::Vector3();
        }

        void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) {
            DataMatrixDeriv & c_d = *cId[m_geo->getMstate()].write();
            MatrixDeriv & c = *c_d.beginEdit();
            MatrixDerivRowIterator c_it1 = c.writeLine(cline);
            c_it1.addCol(m_pid[0],N);
            c_d.endEdit();
        }

        void refineToClosestPoint(const Coord & /*P*/) {}

        void getControlPoints(helper::vector<defaulttype::Vector3> & controlPoints) {
            const helper::ReadAccessor<Data <VecCoord> > & x = m_geo->getMstate()->read(core::VecCoordId::position());
            controlPoints.push_back(x[m_pid[0]]);
        }

    protected:
        const PointGeometry * m_geo;
    };


    virtual ConstraintProximityPtr getPointProximity(unsigned eid) const;

    virtual int getNbPoints() const;

    void draw(const core::visual::VisualParams * vparams);

    int getNbElements() const;

    ConstraintProximityPtr getElementProximity(unsigned eid) const;

};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
