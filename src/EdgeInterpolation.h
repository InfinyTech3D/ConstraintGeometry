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
#ifndef SOFA_COMPONENT_NEEDLEINTERPOLATION_H
#define SOFA_COMPONENT_NEEDLEINTERPOLATION_H

#include "ConstraintGeometry.h"
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>

namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
class EdgeInterpolation: public ConstraintGeometry<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(EdgeInterpolation,DataTypes) , SOFA_TEMPLATE(ConstraintGeometry,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;

    void init() {
        this->getContext()->get(m_state);
        if (m_state == NULL) {
            serr << " Error cannot find the mstate" << sendl;
            return;
        }

        this->getContext()->get(m_container);
        if (m_container == NULL) {
            serr << " Error cannot find the edgeContainer" << sendl;
            return;
        }
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getMstate() {
        return m_state;
    }

    virtual void fillProximity(unsigned id,ConstraintProximity & pinfo) = 0;

    virtual void fillProximity(const Coord & P,ConstraintProximity & pinfo) = 0;

    virtual void fillConstraintNormal(const ConstraintProximity & pinfo, ConstraintNormal & ninfo) = 0;

    sofa::component::topology::EdgeSetTopologyContainer* getTopologyContainer() {
        return m_container;
    }

protected :
    sofa::core::behavior::MechanicalState<DataTypes> * m_state;
    sofa::component::topology::EdgeSetTopologyContainer* m_container;
};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleDescription_H
