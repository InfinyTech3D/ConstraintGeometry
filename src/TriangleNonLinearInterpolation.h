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
   Data <double> f_nonlin_tol;

   TriangleNonLinearInterpolation()
   : Inherit()
   , f_activateNewton(initData(&f_activateNewton, (bool) true,"activateNewton","activate newton iterator in order to refine position on non linear triangle"))
   , f_nonlin_max_it(initData(&this->f_nonlin_max_it, (unsigned) 10,"nonlin_max_it","Max iteration in the Newton Raphson solver used for projection of points on bezier curve"))
   , f_nonlin_tol(initData(&f_nonlin_tol, (double) 0.1,"nonlin_tol","Tolerance in the Newton Raphson solver used for projection of points on bezier curve"))
   {}

   void newtonIterations(const Vector3 & P,ConstraintProximity &pinfo) {
       double epsilon = f_nonlin_tol.getValue() * f_nonlin_tol.getValue();
       unsigned int it=0;

       const helper::ReadAccessor<Data <VecCoord> >& x = *this->m_state->read(core::VecCoordId::position());
       const Vector3 & p300 = x[pinfo.pid[2]];
       const Vector3 & p030 = x[pinfo.pid[1]];
       const Vector3 & p003 = x[pinfo.pid[0]];

       double & fact_w = pinfo.fact[2];
       double & fact_u = pinfo.fact[1];
       double & fact_v = pinfo.fact[0];

       while(it< f_nonlin_max_it.getValue()) {
           ///Compute Point on bezier patch

           Vector3 Q_i = getContactPosition(pinfo);
           Vector3 p = Q_i - P;

           ///First derivative
           Vector3 dpdu = getdpdu(pinfo);
           Vector3 dpdv = getdpdv(pinfo);

           /// F(u,v) = ( (Q(u,v)-P).(dpdu(u,v)), (Q(u,v)-P).(dpdv(u,v)))
           Vector2 F( p*dpdu, p*dpdv);

           ////////// Convergence
           if(F.norm()<epsilon) return;

           ///Second derivative
           Vector3 d2pdu2 = getd2pdu2(pinfo);
           Vector3 d2pdv2 = getd2pdv2(pinfo);
           Vector3 d2pduv = getd2pduv(pinfo);

           defaulttype::Mat<2,2,double> dFdUV, invM;
           dFdUV[0][0] = dpdu*dpdu-d2pdu2*p;
           dFdUV[1][0] = dFdUV[0][1] = dpdv*dpdu-d2pduv*p;
           dFdUV[1][1] = dpdv*dpdv-d2pdv2*p;

           invertMatrix(invM, dFdUV);

//           double det = dFdUV[0][0]*dFdUV[1][1]-dFdUV[1][0]*dFdUV[1][0];
//           if (fabs(det) < 0.000000000000000001) {
//               msg_error(this) << "fail to converge";
//               return; /// fails to converge
//           }

//           invM[0][0] = dFdUV[1][1]/det;
//           invM[1][1] = dFdUV[0][0]/det;
//           invM[1][0] = invM[0][1] = -dFdUV[0][1]/det;

           Vector2 dUV = invM*F;

           //if (dUV[0]*dUV[0] + dUV[1]*dUV[1] < epsilon) return;///check dx;

           double new_u = fact_u-dUV[0];
           double new_v = fact_v-dUV[1];
           //double new_w = 1.0 - new_u - new_v;

           if (new_u < 0.0) new_u = 0.0;
           if (new_u > 1.0) new_u = 1.0;
           if (new_v < 0.0) new_v = 0.0;
           if (new_v > 1.0) new_v = 1.0;

           if (new_u+new_v>1.0) {
               printf("HOHO\n");
               ///project the point on edge 2 and recompute barycoord
               Vector3 edge_e0 = p003 - p030;
               Vector3 edge_v2 = Q_i - p030;

               double dot2 = dot(edge_v2,edge_e0) / dot(edge_e0,edge_e0);
               if (dot2<0.0) dot2 = 0.0;
               else if (dot2>1.0) dot2 = 1.0;

               Vector3 edge_P = p030 + edge_e0 * dot2;

               Vector3 v0 = p030 - p300;
               Vector3 v1 = p003 - p300;

               double d00 = dot(v0, v0);
               double d01 = dot(v0, v1);
               double d11 = dot(v1, v1);
               double invDenom = 1.0 / (d00 * d11 - d01 * d01);

               Vector3 v2 = edge_P - p300;
               double d20 = dot(v2, v0);
               double d21 = dot(v2, v1);
               new_u = (d11 * d20 - d01 * d21) * invDenom;
               new_v = (d00 * d21 - d01 * d20) * invDenom;
           }
           double new_w = 1.0 - new_u - new_v;

           double dx = (fact_u-new_u) * (fact_u-new_u) +
                       (fact_v-new_v) * (fact_v-new_v) +
                       (fact_w-new_w) * (fact_w-new_w);

           fact_u = new_u;
           fact_v = new_v;
           fact_w = new_w;

           if (dx < epsilon) return;

           it++;
       }
   }

   virtual void fillProximity(const Vector3 & P,ConstraintProximity & pinfo) {
        Inherit::fillProximity(P,pinfo);

        if(f_activateNewton.getValue())
            newtonIterations(P,pinfo);
   }

//   void draw(const core::visual::VisualParams *vparams);

   virtual Vector3 getContactPosition(const ConstraintProximity & pinfo) = 0;
   virtual Vector3 getContactFreePosition(const ConstraintProximity & pinfo) = 0;


   ///Get first derivative
   virtual Vector3 getdpdu(const ConstraintProximity & pinfo) = 0;
   virtual Vector3 getdpdv(const ConstraintProximity & pinfo) = 0;

   ///Get second derivative
   virtual Vector3 getd2pdu2(const ConstraintProximity & pinfo) = 0;
   virtual Vector3 getd2pdv2(const ConstraintProximity & pinfo) = 0;
   virtual Vector3 getd2pduv(const ConstraintProximity & pinfo) = 0;

};

} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
