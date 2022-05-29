#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa::constraintGeometry {

/*!
 * \brief The ContactDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ContactDirection : public ConstraintDirection {
public:
    SOFA_CLASS(ContactDirection , ConstraintDirection);

    core::objectmodel::SingleLink<ContactDirection,BaseNormalHandler, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_normalHandler;

    ContactDirection()
    : l_normalHandler(initLink("handler", "link to the default normal handler")) {}

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & d) const override {
        if (l_normalHandler==NULL) {
            std::cerr << " Error you need to specify a normal handler" << std::endl;
            return ConstraintNormal();
        }

        type::Vector3 mainDir = (d.first->getPosition() - d.second->getPosition()).normalized();

        type::Vector3 secondDir = l_normalHandler->getNormal(d.second);

        if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;

        return ConstraintNormal(mainDir);
    }


};

}
