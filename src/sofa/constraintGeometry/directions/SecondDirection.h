#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The SecondDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class SecondDirection : public ConstraintDirection {
public:
    SOFA_CLASS(SecondDirection , ConstraintDirection);

    core::objectmodel::SingleLink<SecondDirection,BaseNormalHandler, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_normalHandler;

    SecondDirection()
    : l_normalHandler(initLink("handler", "link to the default normal handler")) {}

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & d) const override {
        if (l_normalHandler==NULL) {
            std::cerr << " Error you need to specify a normal handler" << std::endl;
            return ConstraintNormal();
        }

        type::Vector3 N;

        if (!l_normalHandler->getNormal(d.second,N)) return ConstraintNormal();

        return ConstraintNormal(N);
    }

};

}
