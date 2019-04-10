#pragma once

#include <sofa/constraintGeometry/BaseConstraint.h>
#include <sofa/constraintGeometry/resolution/UnilateralResolution.h>
#include <sofa/collisionAlgorithm/algorithm/FindClosestPointAlgorithm.h>
#include <sofa/constraintGeometry/normals/ContactNormal.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The ConstraintContact class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ConstraintContact : public BaseConstraint {
public:
    SOFA_CLASS(ConstraintContact , BaseConstraint);

    Data<double> d_maxForce;
    Data<double> d_friction;
    Data<collisionAlgorithm::DetectionOutput> d_input;


    /*!
     * \brief ConstraintContact constructor
     */
    ConstraintContact()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction"))
    , d_input(initData(&d_input, "input", "Link to detection output")) {}

    /*!
     * \brief createConstraintResolution : factory method for constraint solvers
     * \param cst : InternalConstraint
     * \return (UnilateralConstraint|UnilateralFriction)Resolution => abstract ConstraintResolution ptr
     */
    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint * cst) const {
        if (cst->size() == 1)
            return new UnilateralConstraintResolution(d_maxForce.getValue());
        else
            return new UnilateralFrictionResolution(d_friction.getValue(),d_maxForce.getValue());
    }

    /*!
     * \brief createConstraints
     * processes from and dest geometries using specified algorith;
     * \param[out] constraints : ConstraintContainer
     */
    virtual void createConstraints(ConstraintContainer & constraints) {
        const collisionAlgorithm::DetectionOutput & input = d_input.getValue();

        for (unsigned i=0;i<input.size();i++) {
            const collisionAlgorithm::DetectionOutput::PairDetection & d = input[i];

            ContactNormal CN(d);

            if (d_friction.getValue() != 0.0)
                CN.addFriction();

            constraints.push_back(this, d, CN, &ConstraintContact::createConstraintResolution);
        }
    }


};

}

}
