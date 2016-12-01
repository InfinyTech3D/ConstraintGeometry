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
#ifndef SOFA_COMPONENT_FORCEFIELD_PLANECUTTING_INL
#define SOFA_COMPONENT_FORCEFIELD_PLANECUTTING_INL


#include "PlaneCutting.h"
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa {

namespace component {

namespace collision {

template<class DataTypes>
PlaneCutting<DataTypes>::PlaneCutting()
: _draw( initData(&_draw, false ,"draw_cutline",""))
, f_useRest( initData(&f_useRest, false, "use_rest", "Use Rest Position"))
, f_remesh( initData(&f_remesh, true, "remesh", "Remesh the cut"))
, f_nbCutLine( initData(&f_nbCutLine, (unsigned int)0 ,"nbCutLine",""))
, f_intialCutLeft( initData(&f_intialCutLeft, Coord() ,"cutLeft", "Left point of cut line"))
, f_intialCutRight( initData(&f_intialCutRight, Coord() ,"cutRight", "Right point of cut line"))
, f_cutDirection( initData(&f_cutDirection, Coord() ,"cutDirection", "Cut direction"))
{
    this->f_listening.setValue(true);
    cutLineIdx = 0;
}

template<class DataTypes>
void PlaneCutting<DataTypes>::init() {
    this->getContext()->get(tetrahedronAlg);

    if (tetrahedronAlg==NULL) serr << "Tetra algo not found" << std::endl;

    //cutting tool
    currCutLine[0] = f_intialCutLeft.getValue();
    currCutLine[1] = f_intialCutRight.getValue();
    preCutLine[0] = currCutLine[0];
    preCutLine[1] = currCutLine[1];
}

template<class DataTypes>
void PlaneCutting<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event) {
    if (dynamic_cast<simulation::AnimateEndEvent*>(event)) {
        doCut();
    }
}

template<class DataTypes>
void PlaneCutting<DataTypes>::doCut() {
    if (cutLineIdx < f_nbCutLine.getValue()) {
        cutLineIdx++;

        preCutLine[0] = currCutLine[0];
        preCutLine[1] = currCutLine[1];
        currCutLine[0] = preCutLine[0] + f_cutDirection.getValue();
        currCutLine[1] = preCutLine[1] + f_cutDirection.getValue();

        if (tetrahedronAlg == NULL) return;

        if (f_remesh.getValue()) {
            if (f_useRest.getValue()) tetrahedronAlg->subDivideRestTetrahedronsWithPlane(preCutLine[0], preCutLine[1], currCutLine[0], currCutLine[1]);
            else tetrahedronAlg->subDivideTetrahedronsWithPlane(preCutLine[0], preCutLine[1], currCutLine[0], currCutLine[1]);
        } else {
            if (f_useRest.getValue()) tetrahedronAlg->carveRestTetrahedronsWithPlane(preCutLine[0], preCutLine[1], currCutLine[0], currCutLine[1]);
            else tetrahedronAlg->carveTetrahedronsWithPlane(preCutLine[0], preCutLine[1], currCutLine[0], currCutLine[1]);
        }
    }
}

template<class DataTypes>
void PlaneCutting<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/) {
    if (_draw.getValue()) {
        glColor4f(1,0,0,1);
        glDisable(GL_LIGHTING);
        glBegin(GL_QUADS);

        glVertex3f(currCutLine[0][0],currCutLine[0][1],currCutLine[0][2]);
        glVertex3f(preCutLine[0][0],preCutLine[0][1],preCutLine[0][2]);
        glVertex3f(preCutLine[1][0],preCutLine[1][1],preCutLine[1][2]);
        glVertex3f(currCutLine[1][0],currCutLine[1][1],currCutLine[1][2]);

        glEnd();
    }
}



} // namespace collision

} // namespace component

} // namespace sofa

#endif
