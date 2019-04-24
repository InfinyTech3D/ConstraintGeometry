#pragma once

#include <sofa/constraintGeometry/constraint/ConstraintBilateral.h>
#include <sofa/collisionAlgorithm/DataDetectionOutput.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa {

namespace constraintGeometry {

class ConstraintProjective2D : public constraintGeometry::ConstraintBilateral {
public :
    SOFA_CLASS (ConstraintProjective2D, constraintGeometry::ConstraintBilateral) ;

//    Data<double> d_maxForce;
//    Data<collisionAlgorithm::DetectionOutput> d_input ;
    Data<defaulttype::Mat3x4d> d_projectionMatrix;

    /// \brief Constraint Constructor
    ConstraintProjective2D ()
    : d_projectionMatrix(
        initData(
            &d_projectionMatrix,
            defaulttype::Mat3x4d(
                defaulttype::Vec<4,float>(1.0,0.0,0.0,0.0),
                defaulttype::Vec<4,float>(0.0,1.0,0.0,0.0),
                defaulttype::Vec<4,float>(0.0,0.0,1.0,0.0)),
            "projectionMatrix",
            "Projection Matrix")) {}

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

    /*!
     * \brief createConstraints (called in update() method in baseConstraint => called by sofa or forced)
     * processes from and dest geometries using specified algorithm
     * \param[out] constraints : ConstraintContainer
     */
    void createConstraints(ConstraintContainer & constraints) {
        //*
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            // project data from 3D => 2D
            defaulttype::Vector3 N1 = d.first->getPosition() - d.second->getPosition() ;
            defaulttype::Vector3 P = d.second->getPosition() ;
            defaulttype::Vector3 Y = cross (P-m_A,N1);
            Y.normalized();
            defaulttype::Vector3 N2 = cross(m_A-P,Y);
            N2.normalized();
            ConstraintNormal CN (N2);

            if (CN.size() == 1) {
                constraints.push_back(this, d, CN, &ConstraintProjective2D::createConstraintResolution1);
            } else if (CN.size() == 2) {
                constraints.push_back(this, d, CN, &ConstraintProjective2D::createConstraintResolution2);
            } else if (CN.size() == 3) {
                constraints.push_back(this, d, CN, &ConstraintProjective2D::createConstraintResolution3);
            }
        }
        //*/
    }

protected :
    defaulttype::Vector3 m_A ;
//    defaulttype::Matrix3 m_C, m_iC ;
} ;

}

}
