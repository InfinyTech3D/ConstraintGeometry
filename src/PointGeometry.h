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

#include "ConstraintGeometry.h"
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

class PointGeometry : public BaseGeometry
{
public:
    SOFA_CLASS(PointGeometry , BaseGeometry );

    class PointConstraintProximity : public ConstraintProximity {
    public:

        PointConstraintProximity(const PointGeometry * geo, unsigned pid,double fact = 1.0)
        : ConstraintProximity(geo) {
            m_pid.push_back(pid);
            m_fact.push_back(fact);
        }
    };

    virtual ConstraintProximityPtr getPointProximity(unsigned eid) const;

    virtual ConstraintProximityPtr projectPoint(const defaulttype::Vector3 & T,unsigned eid) const;

    virtual int getNbElements() const;

    void draw(const core::visual::VisualParams * vparams);

};

} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
