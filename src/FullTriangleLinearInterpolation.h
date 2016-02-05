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
#ifndef SOFA_COMPONENT_FULLTRIANGLELINEARINTERPOLATION_H
#define SOFA_COMPONENT_FULLTRIANGLELINEARINTERPOLATION_H

#include "TriangleInterpolation.h"
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>

namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
class FullTriangleLinearInterpolation : public TriangleInterpolation<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(FullTriangleLinearInterpolation,DataTypes) , SOFA_TEMPLATE(TriangleInterpolation,DataTypes) );

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

    FullTriangleLinearInterpolation();

    virtual void fillProximity(const Coord & P,ConstraintProximity & pinfo);

    virtual void fillConstraintNormal(const ConstraintProximity & pinfo,ConstraintNormal & ninfo);

    void draw(const core::visual::VisualParams */*vparams*/);

    void handleEvent(sofa::core::objectmodel::Event* event);



protected:

    typedef struct {
        Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        Vector3 tn,ax1,ax2;
    } TriangleInfo;

    virtual void prepareDetection();

    void projectPointOnTriangle(const Vector3 & s,const TriangleInfo & tinfo, const Vector3 & p0, const Vector3 & p1,const Vector3 & p2, double & fact_w,double & fact_u, double & fact_v);

    void computeBaryCoords(const Vector3 & proj_P,const TriangleInfo & tinfo, const Vector3 & p0, double & fact_w,double & fact_u, double & fact_v);

    helper::vector<TriangleInfo> m_triangle_info;
    helper::vector<Vector3> m_point_normal;
};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
