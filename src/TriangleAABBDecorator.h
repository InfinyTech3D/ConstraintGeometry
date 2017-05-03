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
#ifndef SOFA_COMPONENT_TRIANGLEAABBDECORATOR_H
#define SOFA_COMPONENT_TRIANGLEAABBDECORATOR_H

#include "ConstraintGeometry.h"
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>

namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
class TriangleAABBDecorator : public TriangleGeometry<DataTypes>::TriangleDecorator
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TriangleAABBDecorator,DataTypes) , SOFA_TEMPLATE(GeometryDecorator,DataTypes) );

//    typedef TriangleDecorator<DataTypes> Inherit;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef defaulttype::Vector3 Vector3;
    typedef defaulttype::Vec3i Vec3i;

    TriangleAABBDecorator();

    virtual ConstraintProximity findClosestTriangle(TriangleGeometry<DataTypes> * geo, const defaulttype::Vector3 & P);

    void draw(const core::visual::VisualParams */*vparams*/);

    Data<Vec3i> d_nbox;
    Data<bool> d_drawBbox;

protected:

    virtual void init();

    virtual void reinit();

    virtual void prepareDetection();

    ConstraintProximity findClosestTriangle(TriangleGeometry<DataTypes> * geo, const defaulttype::Vector3 & P,const std::set<unsigned> & triangles);

    void fillTriangleSet(int d,const Vec3i & cbox,std::set<unsigned> & vecpinfo);

    Vector3 m_Bmin,m_Bmax,m_cellSize;    
    helper::vector<helper::vector<helper::vector<helper::vector<unsigned> > > >  m_triangleboxes;
};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
