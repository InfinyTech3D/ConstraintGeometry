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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <constraint/ConstraintPair.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <math.h>
#include <assert.h>     /* assert */

namespace sofa
{

namespace core
{

namespace behavior
{

ConstraintPair::ConstraintPair()
: d_from(initData(&d_from, "from", "Collision reponse algorithm"))
, d_dest(initData(&d_dest, "dest", "Collision reponse algorithm")) {}


void ConstraintPair::init() {
    Inherit1::init();

    this->getContext()->get(m_from,d_from.getValue());
    if (m_from == NULL) serr << "Error cannot find the algo" << std::endl;

    this->getContext()->get(m_dest,d_dest.getValue());
    if (m_dest == NULL) serr << "Error cannot find the algo" << std::endl;
}

void ConstraintPair::processGeometricalData() {
    if (m_algo == NULL) return;

    m_detections.clear();
    m_algo->processAlgorithm(m_from, m_dest, m_detections);
}


SOFA_DECL_CLASS(ConstraintPair)

int ConstraintPairClass = core::RegisterObject("Triangle liear interpolation")
.add<ConstraintPair >()
;

} // namespace controller

} // namespace component

} // namespace sofa

