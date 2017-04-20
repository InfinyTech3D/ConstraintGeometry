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
#ifndef SOFA_COMPONENT_TRIANGLENONLINEARINTERPOLATION_H
#define SOFA_COMPONENT_TRIANGLENONLINEARINTERPOLATION_H

#include "TriangleLinearInterpolation.h"

namespace sofa {

namespace core {

namespace behavior {

template<class DataTypes>
class TriangleNonLinearInterpolation : public TriangleLinearInterpolation<DataTypes>
{
public:
   SOFA_CLASS(SOFA_TEMPLATE(TriangleNonLinearInterpolation,DataTypes) , SOFA_TEMPLATE(TriangleLinearInterpolation,DataTypes) );

   typedef  TriangleLinearInterpolation<DataTypes> Inherit;
   typedef typename defaulttype::Vector2 Vector2;
   typedef typename defaulttype::Vector3 Vector3;
   typedef typename DataTypes::Coord Coord;
   typedef typename DataTypes::Real Real;
   typedef typename DataTypes::VecCoord VecCoord;
   typedef typename DataTypes::VecDeriv VecDeriv;

   Data <bool> f_activateNewton;
   Data <unsigned> f_nonlin_max_it;
   Data <double> f_nonlin_tolerance;
   Data <double> f_nonlin_threshold;

   TriangleNonLinearInterpolation()
   : Inherit()
   , f_activateNewton(initData(&f_activateNewton, (bool) true,"activateNewton","activate newton iterator in order to refine position on non linear triangle"))
   , f_nonlin_max_it(initData(&this->f_nonlin_max_it, (unsigned) 10,"nonlin_max_it","Max iteration in the Newton Raphson solver used for projection of points on non linear triangle"))
   , f_nonlin_tolerance(initData(&f_nonlin_tolerance, (double) 0.01,"nonlin_tol","Tolerance in the Newton Raphson solver used for projection of points on non linear triangle"))
   , f_nonlin_threshold(initData(&f_nonlin_threshold, (double) 0.001,"nonlin_th","Threshold in the Newton Raphson solver used for projection of points on non linear triangle"))
   {}

protected:
   void newtonIterations(const Vector3 & P,ConstraintProximity &pinfo) {
       const double &tolerance = f_nonlin_tolerance.getValue();
       const double &threshold = f_nonlin_threshold.getValue();

       unsigned int it=0;

       double & fact_w = pinfo.fact[2];
       double & fact_u = pinfo.fact[1];
       double & fact_v = pinfo.fact[0];

       double delta = 0.000001;

       while(it< f_nonlin_max_it.getValue()) {

            Vector3 Q_i = getContactPosition(pinfo);

            Vector3 p = P - Q_i;

            ConstraintNormal C_i;
            fillConstraintNormal(pinfo,C_i);

            Vector2 e_0(dot(p,C_i.normals[1]),dot(p,C_i.normals[2]));

            if(e_0.norm() < tolerance) break;

            ConstraintProximity P_v = pinfo;
            P_v.fact[0] += delta;
            P_v.fact[2] = 1.0 - P_v.fact[0] - P_v.fact[1];
            Vector3 Q_v = getContactPosition(P_v);
            Vector3 p_v = (P - Q_v);
            Vector2 e_v(dot(p_v,C_i.normals[1]),dot(p_v,C_i.normals[2]));

            ConstraintProximity P_u = pinfo;
            P_u.fact[1] += delta;
            P_u.fact[2] = 1.0 - P_u.fact[0] - P_u.fact[1];
            Vector3 Q_u = getContactPosition(P_u);
            Vector3 p_u = (P - Q_u);
            Vector2 e_u(dot(p_u,C_i.normals[1]),dot(p_u,C_i.normals[2]));

            defaulttype::Mat<2,2,double> J, invJ;
            J[0][0] = (e_u[0] - e_0[0])/delta;
            J[1][0] = (e_u[1] - e_0[1])/delta;
            J[0][1] = (e_v[0] - e_0[0])/delta;
            J[1][1] = (e_v[1] - e_0[1])/delta;

            invertMatrix(invJ, J);

            Vector2 dUV = -invJ * e_0;

            double new_u = fact_u+dUV[0];
            double new_v = fact_v+dUV[1];
            double new_w = 1.0 - new_u - new_v;

            if (new_u < 0.0) break;
            if (new_u > 1.0) break;
            if (new_v < 0.0) break;
            if (new_v > 1.0) break;
            if (new_w < 0.0) break;
            if (new_w > 1.0) break;

            fact_u = (fact_u + new_u) * 0.5;
            fact_v = (fact_v + new_v) * 0.5;
            fact_w = (fact_w + new_w) * 0.5;

            if(dUV.norm() < threshold) break;

            it++;
       }
   }

   virtual void fillProximity(const Vector3 & P,ConstraintProximity & pinfo) {
        Inherit::fillProximity(P,pinfo);

        if(f_activateNewton.getValue())
            newtonIterations(P,pinfo);
   }

   virtual Vector3 getContactPosition    (const ConstraintProximity & pinfo) = 0;
   virtual Vector3 getContactFreePosition(const ConstraintProximity & pinfo) = 0;
   virtual void    fillConstraintNormal  (const ConstraintProximity & pinfo,ConstraintNormal & ninfo) = 0;

};

} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
