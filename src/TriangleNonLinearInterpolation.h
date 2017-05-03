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

   Data <bool> d_activateNewton;
   Data <unsigned> d_nonlin_max_it;
   Data <double> d_nonlin_tolerance;
   Data <double> d_nonlin_threshold;
   Data <unsigned> d_draw_tesselation;

   TriangleNonLinearInterpolation()
   : Inherit()
   , d_activateNewton(initData(&d_activateNewton, (bool) true,"activateNewton","activate newton iterator in order to refine position on non linear triangle"))
   , d_nonlin_max_it(initData(&d_nonlin_max_it, (unsigned) 20,"nonlin_max_it","Max iteration in the Newton Raphson solver used for projection of points on non linear triangle"))
   , d_nonlin_tolerance(initData(&d_nonlin_tolerance, (double) 0.001,"nonlin_tol","Tolerance in the Newton Raphson solver used for projection of points on non linear triangle"))
   , d_nonlin_threshold(initData(&d_nonlin_threshold, (double) 0.00001,"nonlin_th","Threshold in the Newton Raphson solver used for projection of points on non linear triangle"))
   , d_draw_tesselation(initData(&d_draw_tesselation, (unsigned) 0.0,"tesselation","Draw tesselated triangles"))
   {}

//   helper::vector<defaulttype::Vector3> m_pointsNewton;
//   helper::vector<defaulttype::Vector3> m_normalNewton;
//   unsigned m_tid;

protected:
   void newtonIterations(const Vector3 & P,ConstraintProximity & pinfo) {
          const double &tolerance = d_nonlin_tolerance.getValue();
          const double &threshold = d_nonlin_threshold.getValue();

          unsigned int it=0;

          double & fact_w = pinfo.m_fact[2];
          double & fact_u = pinfo.m_fact[1];
          double & fact_v = pinfo.m_fact[0];

          double delta = 0.00001;

//          m_tid = pinfo.getEid();

//          m_pointsNewton.clear();
//          m_normalNewton.clear();
          while(it< d_nonlin_max_it.getValue()) {
               Vector3 Q = pinfo.getPosition();
               Vector3 nQP = (P - Q).normalized();

//               m_pointsNewton.push_back(pinfo.getPosition());

               defaulttype::Vector3 N1 = this->getSurfaceNormal(pinfo);
               N1.normalize();

               defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(1,0,0)))>0.99) ? defaulttype::Vector3(0,1,0) : defaulttype::Vector3(1,0,0)));
               N2.normalize();

               defaulttype::Vector3 N3 = cross(N1,N2);
               N3.normalize();

               Vector2 e_0(dot(nQP,N2),dot(nQP,N3));

               if(e_0.norm() < tolerance) break;

               ConstraintProximity P_v = pinfo;
               P_v.m_fact[0] += delta;
               P_v.m_fact[2] = 1.0 - P_v.m_fact[0] - P_v.m_fact[1];
               Vector3 p_v = (P - P_v.getPosition()).normalized();
               Vector2 e_v(dot(p_v,N2),dot(p_v,N3));

               ConstraintProximity P_u = pinfo;
               P_u.m_fact[1] += delta;
               P_u.m_fact[2] = 1.0 - P_u.m_fact[0] - P_u.m_fact[1];
               Vector3 p_u = (P - P_u.getPosition()).normalized();
               Vector2 e_u(dot(p_u,N2),dot(p_u,N3));

               defaulttype::Mat<2,2,double> J, invJ;
               J[0][0] = (e_v[0] - e_0[0])/delta;
               J[1][0] = (e_v[1] - e_0[1])/delta;
               J[0][1] = (e_u[0] - e_0[0])/delta;
               J[1][1] = (e_u[1] - e_0[1])/delta;

               invertMatrix(invJ, J);

               // dUV is the optimal direction
               Vector2 dUV = -invJ * e_0;
               if(dUV.norm() < threshold) break;

               //bary coords of the solution of the 2D problem
               double sol_v = fact_v + dUV[0];
               double sol_u = fact_u + dUV[1];
               double sol_w = 1.0 - sol_u - sol_v;

               // we now search what is the optimal displacmeent along this path
               defaulttype::Vector3 dir2d(sol_v - fact_v,
                                          sol_u - fact_u,
                                          sol_w - fact_w);

               if(dir2d.norm() < threshold) {
                   printf("b2\n");
                   break;
               }

               //we apply a small perturbation arond the 2d direction
               dir2d.normalize();

               ConstraintProximity P_a = pinfo;
               P_a.m_fact[0] = fact_v + dir2d[0] * delta;
               P_a.m_fact[1] = fact_u + dir2d[1] * delta;
               P_a.m_fact[2] = fact_w + dir2d[2] * delta;
               Vector3 QA = P_a.getPosition();

//               m_normalNewton.push_back((Q - QA).normalized()*0.1);
               double fact;

               if (fabs(dot(nQP,N1))>0.8) {
                   double fx = acos(fabs(dot(nQP,N1)));
                   double fxdx = acos(fabs(dot((P - QA).normalized(),this->getSurfaceNormal(P_a))));
                   double j = (fxdx - fx) / delta;
                   fact = -fx / j;
               } else {
                   Vector3 nQA = (Q-QA).normalized();
                   double fx = dot(P-Q, nQA);
                   double fxdx = dot(P-QA, nQA);
                   double j = (fxdx - fx) / delta;
                   fact = -fx / j;
               }

               if(fabs(fact) < threshold) break;

               dir2d *= fact;

               double new_v = fact_v + dir2d[0];
               double new_u = fact_u + dir2d[1];
               double new_w = fact_w + dir2d[2];

               if (new_v<0 && fabs(dir2d[0])>0) dir2d *= -fact_v / dir2d[0];
               if (new_u<0 && fabs(dir2d[1])>0) dir2d *= -fact_u / dir2d[1];
               if (new_w<0 && fabs(dir2d[2])>0) dir2d *= -fact_w / dir2d[2];

               fact_v = fact_v+dir2d[0];
               fact_u = fact_u+dir2d[1];
               fact_w = fact_w+dir2d[2];

               it++;
          }


//          printf("ITER %d       %f %f %f\n",it,fact_u,fact_v,fact_w);
   }

   virtual ConstraintProximity projectPointOnTriangle(unsigned tid,const Vector3 & P) {
         ConstraintProximity pinfo = Inherit::projectPointOnTriangle(tid,P);

         if (d_activateNewton.getValue()) newtonIterations(P,pinfo);

         return pinfo;
   }

   virtual defaulttype::Vector3 getPosition(const ConstraintProximity & pinfo) = 0;

   virtual defaulttype::Vector3 getSurfaceNormal  (const ConstraintProximity & pinfo) = 0;

   void tesselate(const core::visual::VisualParams * vparams, unsigned level,int tid, const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C) {
        if (level >= d_draw_tesselation.getValue()) {
           defaulttype::Vector3 pA = this->getPosition(this->getTriangleProximity(tid,A[0],A[1],A[2]));
           defaulttype::Vector3 pB = this->getPosition(this->getTriangleProximity(tid,B[0],B[1],B[2]));
           defaulttype::Vector3 pC = this->getPosition(this->getTriangleProximity(tid,C[0],C[1],C[2]));

           this->drawTriangle(vparams,pA,pB,pC);

           return;
        }

        defaulttype::Vector3 D = (A + B)/2.0;
        defaulttype::Vector3 E = (A + C)/2.0;
        defaulttype::Vector3 F = (B + C)/2.0;

        defaulttype::Vector3 G = (A + B + C)/3.0;

        tesselate(vparams,level+1,tid,A,D,G);
        tesselate(vparams,level+1,tid,D,B,G);

        tesselate(vparams,level+1,tid,G,B,F);
        tesselate(vparams,level+1,tid,G,F,C);

        tesselate(vparams,level+1,tid,G,C,E);
        tesselate(vparams,level+1,tid,A,G,E);
   }

   void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

       glDisable(GL_LIGHTING);

//       glBegin(GL_LINE_STRIP);
//       for (unsigned i=0;i<m_pointsNewton.size();i++) {
//           glColor3f(i*1.0/m_pointsNewton.size(),0,1.0-(i*1.0/m_pointsNewton.size()));
//           helper::gl::glVertexT(m_pointsNewton[i]);
//       }
//       glEnd();
//       for (unsigned i=0;i<m_pointsNewton.size();i++) {
//           glColor3f(i*1.0/m_pointsNewton.size(),0,1.0-(i*1.0/m_pointsNewton.size()));
//           vparams->drawTool()->drawSphere(m_pointsNewton[i],0.05);
//           vparams->drawTool()->drawArrow(m_pointsNewton[i],m_pointsNewton[i] + m_normalNewton[i],0.05, defaulttype::Vec<4,float>(1.0f,0.0f,1.0f,1.0f));

//       }

//       const helper::ReadAccessor<Data <typename DataTypes::VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());
//       sofa::core::topology::Topology::Triangle tri = this->getTopology()->getTriangle(m_tid);

//       glColor3f(0,1,0);
//       glBegin(GL_TRIANGLES);
//       helper::gl::glVertexT(x[tri[0]]);
//       helper::gl::glVertexT(x[tri[1]]);
//       helper::gl::glVertexT(x[tri[2]]);
//       glEnd();


//       glBegin(GL_LINES);
//       for (unsigned i=0;i<m_pointsNewton.size();i++) {
//           glColor3f(1,0,0);helper::gl::glVertexT(Pq);helper::gl::glVertexT(Pu);
//           glColor3f(0,1,0);helper::gl::glVertexT(Pq);helper::gl::glVertexT(Pv);
////           glColor3f(1,0,1);helper::gl::glVertexT(Pq);helper::gl::glVertexT(PJ);
////           glColor3f(0,1,1);helper::gl::glVertexT(Pq);helper::gl::glVertexT(PJn);
//           glColor3f(0,0,1);helper::gl::glVertexT(Pq);helper::gl::glVertexT(Pa);

//       }
//       glEnd();










//        double fact = 1.0 - d_baryC.getValue()[0] - d_baryC.getValue()[1];
//        d_baryC.setValue(defaulttype::Vector3(d_baryC.getValue()[0],d_baryC.getValue()[1],fact));


//        ConstraintProximity pinfo1 = this->getTriangleProximity(0,
//                                                                d_baryC.getValue()[0],
//                                                                d_baryC.getValue()[1],
//                                                                d_baryC.getValue()[2]);

//        ConstraintProximity pinfo2 = this->getTriangleProximity(0,d_baryC.getValue()[0]+d_duv.getValue()[0],
//                                                                  d_baryC.getValue()[1]+d_duv.getValue()[1],
//                                                                  1.0 - (d_baryC.getValue()[0]+d_duv.getValue()[0]) - (d_baryC.getValue()[1]+d_duv.getValue()[1]));


//        defaulttype::Vector3 dir(pinfo2.m_fact[0] - pinfo1.m_fact[0],
//                                 pinfo2.m_fact[1] - pinfo1.m_fact[1],
//                                 pinfo2.m_fact[2] - pinfo1.m_fact[2]);


////        pinfo2.m_fact[2] = 1.0 - pinfo2.m_fact[0] - pinfo2.m_fact[1];

//        glColor3f(0,0,1);
//        vparams->drawTool()->drawSphere(pinfo1.getPosition(),0.1);
//        vparams->drawTool()->drawArrow(pinfo1.getPosition(),pinfo2.getPosition(),0.01, defaulttype::Vec<4,float>(1.0f,0.0f,0.0f,1.0f));

//        double u = d_baryC.getValue()[0]+dir[0];
//        double v = d_baryC.getValue()[1]+dir[1];
//        double w = d_baryC.getValue()[2]+dir[2];

//        std::cout << dir << std::endl;

//        if (u<0) dir *= -d_baryC.getValue()[0] / dir[0];
//        if (v<0) dir *= -d_baryC.getValue()[1] / dir[1];
//        if (w<0) dir *= -d_baryC.getValue()[2] / dir[2];

//        u = d_baryC.getValue()[0]+dir[0];
//        v = d_baryC.getValue()[1]+dir[1];
//        w = d_baryC.getValue()[2]+dir[2];

//        printf("v %f\n",v);

//        ConstraintProximity pinfof = this->getTriangleProximity(0,u,v,w);
//        glColor3f(1,0,1);
//        vparams->drawTool()->drawSphere(pinfof.getPosition(),0.1);








       if (vparams->displayFlags().getShowWireFrame()) glBegin(GL_LINES);
       else {
           glEnable(GL_CULL_FACE);
           glBegin(GL_TRIANGLES);
       }

       for(int t=0;t<this->getTopology()->getNbTriangles();t++) {
           tesselate(vparams,0,t,defaulttype::Vector3(1,0,0),defaulttype::Vector3(0,1,0),defaulttype::Vector3(0,0,1));
       }

       glEnd();
   }
};

} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
