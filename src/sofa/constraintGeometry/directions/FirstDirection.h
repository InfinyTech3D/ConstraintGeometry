#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The FirstDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class FirstDirection : public ConstraintDirection {
public:
    SOFA_CLASS(FirstDirection , ConstraintDirection);

    core::objectmodel::SingleLink<FirstDirection,BaseNormalHandler, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_normalHandler;

    FirstDirection()
    : l_normalHandler(initLink("handler", "link to the default normal handler")) {}

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & d) const override {
        if (l_normalHandler==NULL) {
            std::cerr << " Error you need to specify a normal handler" << std::endl;
            return ConstraintNormal();
        }

        type::Vector3 N = l_normalHandler->getNormal(d.first);

        return ConstraintNormal(-N);
    }

};

}
