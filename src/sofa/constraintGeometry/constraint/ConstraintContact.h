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

    core::objectmodel::SingleLink<ConstraintContact,collisionAlgorithm::BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<ConstraintContact,collisionAlgorithm::BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
    core::objectmodel::SingleLink<ConstraintContact,collisionAlgorithm::BaseAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_algo;

    /*!
     * \brief ConstraintContact constructor
     */
    ConstraintContact()
    : d_maxForce(initData(&d_maxForce, std::numeric_limits<double>::max(), "maxForce", "Max force"))
    , d_friction(initData(&d_friction, 0.0, "mu", "Friction"))
    , l_from(initLink("from", "Link to src geometry"))
    , l_dest(initLink("dest", "Link to dst geometry"))
    , l_algo(initLink("algo", "Link to detection output")) {}

    /*!
     * \brief createConstraintResolution : factory method for constraint solvers
     * \param cst : InternalConstraint
     * \return (UnilateralConstraint|UnilateralFriction)Resolution => abstract ConstraintResolution ptr
     */
    core::behavior::ConstraintResolution* createConstraintResolution(const InternalConstraint * cst) const {
        if (cst->size() == 1)
            return new UnilateralConstraintResolution(d_maxForce.getValue());
        else
            return new UnilateralFrictionResolution(d_maxForce.getValue(),d_friction.getValue());
    }

    /*!
     * \brief createConstraints
     * processes from and dest geometries using specified algorith;
     * \param[out] constraints : ConstraintContainer
     */
    virtual void createConstraints(ConstraintContainer & constraints) {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        helper::vector<collisionAlgorithm::PairDetection> detection;

        if (l_algo == NULL) {
            collisionAlgorithm::FindClosestPointAlgorithm algo;
            algo.processAlgorithm(l_from.get(),l_dest.get(),detection);
        } else {
            l_algo->processAlgorithm(l_from.get(),l_dest.get(),detection);
        }

        //solves contactNormal for each detected collision
        for (unsigned i=0;i<detection.size();i++) {
            const collisionAlgorithm::PairDetection & d = detection[i];

            ContactNormal CN(d);

            if (d_friction.getValue() != 0.0) CN.addFriction();

            constraints.push_back(this, d, CN, &ConstraintContact::createConstraintResolution);
        }
    }


};

}

}
