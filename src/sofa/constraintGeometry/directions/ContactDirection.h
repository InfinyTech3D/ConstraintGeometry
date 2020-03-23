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
        defaulttype::Vector3 mainDir = (d.first->getPosition() - d.second->getPosition()).normalized();

    //            defaulttype::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
        defaulttype::Vector3 secondDir = d.second->getNormal().normalized();

//        if (mainDir.norm() < std::numeric_limits<double>::epsilon()) mainDir = secondDir;
        if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;

        return ConstraintNormal(mainDir);
    }

    ConstraintNormal UpdateConstraintsNormalWithProximityPosition(const collisionAlgorithm::PairDetection & d, defaulttype::Vec3 pf, bool getF, defaulttype::Vec3 pd, bool getD) const override {
        if(getF&&getD){
            defaulttype::Vector3 mainDir = (pf - pd).normalized();
            defaulttype::Vector3 secondDir = d.second->getNormal().normalized();
            if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;
            return ConstraintNormal(mainDir);
        }
        else if(getF){
            defaulttype::Vector3 mainDir = (pf- d.second->getPosition()).normalized();
            defaulttype::Vector3 secondDir = d.second->getNormal().normalized();
            if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;
            return ConstraintNormal(mainDir);
        }
        else if(getD){
            defaulttype::Vector3 mainDir = (d.first->getPosition() - pd).normalized();
            defaulttype::Vector3 secondDir = d.second->getNormal().normalized();
            if (dot(mainDir,secondDir)<=std::numeric_limits<double>::epsilon()) mainDir=secondDir;
            return ConstraintNormal(mainDir);
        }
        else return ConstraintNormal(defaulttype::Vector3(0, 0, 0));

    }

};

}

}
