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
#ifndef SOFA_COMPONENT_BEZIERTRIANGLENONLINEARINTERPOLATION_H
#define SOFAa_COMPONENT_BEZIERTRIANGLENONLINEARINTERPOLATION_H

#include "TriangleNonLinearInterpolation.h"

namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
class BezierTriangleNonLinearInterpolation : public TriangleNonLinearInterpolation<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(BezierTriangleNonLinearInterpolation,DataTypes) , SOFA_TEMPLATE(TriangleNonLinearInterpolation,DataTypes) );

    typedef  TriangleNonLinearInterpolation<DataTypes> Inherit;
    typedef typename Inherit::TriangleInfo TriangleInfo;

    typedef typename defaulttype::Vector2 Vector2;
    typedef typename defaulttype::Vector3 Vector3;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;


    BezierTriangleNonLinearInterpolation();

    virtual void prepareDetection();

    Vector3 getContactPosition(const ConstraintProximity & pinfo);
    Vector3 getContactPosition(const ConstraintProximity & pinfo,double & fact_w,double & fact_u, double & fact_v);

    Vector3 getContactFreePosition(const ConstraintProximity & pinfo);

    void draw(const core::visual::VisualParams *vparams);

    Data <bool> f_use_bezier;
    Data <double> f_draw_bezier_inc;


private :
    typedef struct {
        Vector3 p210,p120,p021,p012,p102,p201,p111;
        Vector3 n110,n011,n101;
    } BezierTriangleInfo;

    ///Get first derivative
    Vector3 getdpdu(const ConstraintProximity & pinfo);
    Vector3 getdpdv(const ConstraintProximity & pinfo);

    ///Get second derivative
    Vector3 getd2pdu2(const ConstraintProximity & pinfo);
    Vector3 getd2pdv2(const ConstraintProximity & pinfo);
    Vector3 getd2pduv(const ConstraintProximity & pinfo);

    helper::vector<BezierTriangleInfo> m_beziertriangle_info;
};


} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
