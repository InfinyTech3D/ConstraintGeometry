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
#ifndef SOFA_COMPONENT_FORCEFIELD_PLANECUTTING_H
#define SOFA_COMPONENT_FORCEFIELD_PLANECUTTING_H

#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/behavior/BaseController.h>
#include "TetrahedronCutting.h"

namespace sofa {

namespace component {

namespace collision {
  
using namespace sofa::defaulttype;

template<class DataTypes>
class PlaneCutting : public core::behavior::BaseController {

public:
    SOFA_CLASS(SOFA_TEMPLATE(PlaneCutting,DataTypes),sofa::core::behavior::BaseController);

    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::value_type Real;

    Data<bool> _draw;
    Data <bool> f_useRest;
    Data<bool> f_remesh;

    //cut with predefined path
    Data<unsigned int> f_nbCutLine;
    Data<Coord> f_intialCutLeft;
    Data<Coord> f_intialCutRight;
    Data<Coord> f_cutDirection;

    virtual void init();

    virtual void handleEvent(sofa::core::objectmodel::Event* event);

    void draw(const core::visual::VisualParams* vparams);

    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg) {
        if (dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes>*>(context->getMechanicalState()) == NULL)
                return false;
        return BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const	{
        return templateName(this);
    }

    static std::string templateName(const PlaneCutting<DataTypes>* = NULL) {
        return DataTypes::Name();
    }

protected:
    sofa::component::topology::TetrahedronCutting<DataTypes> * tetrahedronAlg;

	//Cut line
    Coord  cutDirection;
    Coord currCutLine[2];
    Coord preCutLine[2];
    unsigned cutLineIdx;

    void doCut();

    PlaneCutting();
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
