#pragma once

#include <sofa/constraintGeometry/ConstraintDirection.h>

namespace sofa {

namespace constraintGeometry {

/*!
 * \brief The ContactDirection class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ContactDirection : public ConstraintDirection {
public:
    SOFA_CLASS(ContactDirection , ConstraintDirection);

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    ConstraintNormal createConstraintsNormal(const collisionAlgorithm::PairDetection & d) const override {
        type::Vector3 mainDir = (d.first->getPosition() - d.second->getPosition()).normalized();

    //            type::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
        type::Vector3 secondDir = d.second->getNormal().normalized();

//        if (mainDir.norm() < std::numeric_limits<double>::epsilon()) mainDir = secondDir;
        if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;

        return ConstraintNormal(mainDir);
    }

    ConstraintNormal UpdateConstraintNormalWithProximityPosition(const collisionAlgorithm::PairDetection & d, type::Vec3 pf, bool getF, type::Vec3 pd, bool getD) const override {
        type::Vector3 pfrom = d.first->getPosition();
        type::Vector3 pdest = d.second->getPosition();
//        type::Vec3 freeMotion = d.first->getPosition(core::VecCoordId::freePosition()) - d.first->getPosition(core::VecCoordId::position());
//        std::cout<<"free Motion = "<<freeMotion<<std::endl;
        if(getF) pfrom = pf;
        if(getD) pdest = pd;
        type::Vector3 dir = pfrom- pdest;
        type::Vector3 mainDir = dir.normalized();
        type::Vector3 secondDir = d.second->getNormal().normalized();
        if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;

        return ConstraintNormal(mainDir);

    }

};

}

}
