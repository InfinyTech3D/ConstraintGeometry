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
class Constraintless : public BaseConstraint {
public:
    SOFA_CLASS(Constraintless , BaseConstraint);

    core::objectmodel::SingleLink<
        Constraintless,
        collisionAlgorithm::BaseGeometry,
        BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<
        Constraintless,
        collisionAlgorithm::BaseGeometry,
        BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
    core::objectmodel::SingleLink<
        Constraintless,
        collisionAlgorithm::BaseAlgorithm,
        BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_algo;

    /*!
     * \brief ConstraintContact constructor
     */
    Constraintless()
    : l_from(initLink("from", "Link to src geometry"))
    , l_dest(initLink("dest", "Link to dst geometry"))
    , l_algo(initLink("algo", "Link to detection output")) {}

    /*!
     * \brief createConstraints
     * processes from and dest geometries using specified algorith;
     * \param[out] constraints : ConstraintContainer
     */
    virtual void createConstraints(ConstraintContainer & constraints) {
//        if (l_from == NULL) return;
//        if (l_dest == NULL) return;

//        helper::vector<collisionAlgorithm::PairDetection> detection;

//        if (l_algo == NULL) {
//            collisionAlgorithm::FindClosestPointAlgorithm algo;
//            algo.processAlgorithm(l_from.get(),l_dest.get(),detection);
//        } else {
//            l_algo->processAlgorithm(l_from.get(),l_dest.get(),detection);
//        }
    }


};

}

}
