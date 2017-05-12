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

#include "PointGeometry.h"
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>


namespace sofa {

namespace core {

namespace behavior {

class EdgeGeometry : public PointGeometry
{
public:
    SOFA_CLASS(EdgeGeometry , BaseGeometry );

    typedef defaulttype::Vector3 Vector3;

    virtual defaulttype::Vector3 getNormal(const ConstraintProximity & /*pinfo*/);

    ConstraintProximity getEdgeProximity(unsigned eid, double fact_u,double fact_v);

    virtual ConstraintProximity projectPoint(unsigned eid,const defaulttype::Vector3 & /*T*/);

    unsigned getNbElements();

    void draw(const core::visual::VisualParams * /*vparams*/);
};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
