#pragma once

#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/constraintGeometry/normals/ContactNormal.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The ProjectiveNormals class represents the projective component
 * generating a list of projective constraint directions
 */
class ProjectiveNormals : public sofa::core::objectmodel::BaseObject {
public :

    SOFA_CLASS(ProjectiveNormals, core::objectmodel::BaseObject);

    Data<DataConstraintNormal> d_direction;
    Data<defaulttype::Mat3x4d> d_projectionMatrix;

    ProjectiveNormals()
        : d_direction( // => output
              initData(
                  &d_direction,
                  DataConstraintNormal(
                      std::bind(&ProjectiveNormals::getProjectiveNormals, this, std::placeholders::_1)),
                  "directions",
                  "Link to detection output"))
        , d_projectionMatrix(
              initData(
                  &d_projectionMatrix,
                  defaulttype::Mat3x4d(
                      defaulttype::Vec<4,float>(1.0,0.0,0.0,0.0),
                      defaulttype::Vec<4,float>(0.0,1.0,0.0,0.0),
                      defaulttype::Vec<4,float>(0.0,0.0,1.0,0.0)),
                  "projectionMatrix",
                  "Projection Matrix"))
    {
        init() ;
    }

    /// \brief computes camera position
    void init () {
        defaulttype::Vector3 T;
        defaulttype::Matrix3 C, iC ;
        defaulttype::Mat3x4d proj = d_projectionMatrix.getValue() ;

        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                C[j][i] = proj[j][i];
            }
            T[j] = proj[j][3];
        }

        iC.invert(C);
        m_A = -iC * T; //camera position
    }

protected:
    defaulttype::Vector3 m_A ;
    ConstraintNormal getProjectiveNormals(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        defaulttype::Vector3 N1 = d.first->getPosition() - d.second->getPosition() ;
        defaulttype::Vector3 P = d.second->getPosition() ;
        defaulttype::Vector3 Y = cross (P-m_A,N1);
        Y.normalized();
        defaulttype::Vector3 N2 = cross(m_A-P,Y);
        N2.normalized();
        ConstraintNormal CN (N2);
        return CN ;
    }
};

}

}
