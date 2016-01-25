///******************************************************************************
//*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
//*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
//*                                                                             *
//* This library is free software; you can redistribute it and/or modify it     *
//* under the terms of the GNU Lesser General Public License as published by    *
//* the Free Software Foundation; either version 2.1 of the License, or (at     *
//* your option) any later version.                                             *
//*                                                                             *
//* This library is distributed in the hope that it will be useful, but WITHOUT *
//* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
//* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
//* for more details.                                                           *
//*                                                                             *
//* You should have received a copy of the GNU Lesser General Public License    *
//* along with this library; if not, write to the Free Software Foundation,     *
//* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
//*******************************************************************************
//*                               SOFA :: Modules                               *
//*                                                                             *
//* Authors: The SOFA Team and external contributors (see Authors.txt)          *
//*                                                                             *
//* Contact information: contact@sofa-framework.org                             *
//******************************************************************************/
//#ifndef SOFA_COMPONENT_SURFACECUBICINTERPOLATION_H
//#define SOFA_COMPONENT_SURFACECUBICINTERPOLATION_H

//#include <sofa/core/behavior/ForceField.h>
//#include <sofa/core/behavior/MechanicalState.h>
//#include <sofa/core/objectmodel/Data.h>
//#include <sofa/defaulttype/VecTypes.h>
//#include <SofaConstraint/BilateralInteractionConstraint.h>
//#include "SurfaceLinearInterpolation.h"

//namespace sofa {

//namespace component {

//namespace constraintset {

//class SurfaceCubicInterpolation : public SurfaceLinearInterpolation
//{
//public:
//    SOFA_CLASS(SurfaceCubicInterpolation, SurfaceLinearInterpolation);

//    typedef  SurfaceLinearInterpolation Inherit;
//    typedef typename defaulttype::Vector2 Vector2;
//    typedef typename defaulttype::Vector3 Vector3;

//    SurfaceCubicInterpolation();

//    virtual void prepareDetection();

//    virtual bool fillProximity(const Vector3 & P,Proximity & pinfo);

//    Vector3 getContactPosition(const Proximity & pinfo);

//    Vector3 getContactFreePosition(const Proximity & pinfo);

//    void draw(const core::visual::VisualParams *vparams);

//    Data <bool> f_use_bezier;
//    Data <unsigned> f_bezier_maw_it;
//    Data <double> f_bezier_tol;
//    Data <double> f_draw_bezier_inc;


//private :

//    typedef struct {
//        Vector3 p210,p120,p021,p012,p102,p201,p111;
//        Vector3 n110,n011,n101;
//    } BezierTriangleInfo;

//    void processCollisionDetectionWithBezierCurve(const Vector3 & P, Proximity & pinfo);

//    void newtonIterations(const Vector3 & P,const TriangleInfo & tinfo,const BezierTriangleInfo & tbinfo,double & fact_w,double & fact_u, double & fact_v);

//    helper::vector<BezierTriangleInfo> m_beziertriangle_info;
//};


//} // namespace forcefield

//} // namespace component

//} // namespace sofa


//#endif // NeedleLinearDescription_H
