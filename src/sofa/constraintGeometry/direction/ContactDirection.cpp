#include <sofa/constraintGeometry/BaseDirection.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace constraintGeometry {

class ContactDirection : public BaseDirection {
public:

    SOFA_CLASS(ContactDirection,BaseDirection);

    ConstraintNormal createConstraintNormal(unsigned size, const collisionAlgorithm::DetectionOutput::PairDetection & d) {

    //            defaulttype::Vector3 mainDir = d.getFirstProximity()->getPosition() - d.getSecondProximity()->getPosition();

    //////            if (mainDir.norm()>0.00000001) return ConstraintNormal(mainDir);

    //            defaulttype::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
            defaulttype::Vector3 secondDir = d.second->getNormal().normalized();

            return ConstraintNormal::createFrame(secondDir, size);
    //            return ConstraintNormal(mainDir.normalized() + firstDir + secondDir);
    //            return ConstraintNormal(firstDir);

    //            defaulttype::Vector3 mainDir = d.getSecondProximity()->getNormal();//pair.first->getPosition() - pair.second->getPosition();
    //            defaulttype::Vector3 secondDir = -d.getFirstProximity()->getNormal();

    //            if (mainDir.norm()<0.01) mainDir = secondDir;
    //            else {
    //                mainDir.normalize();
    //                if (dot(mainDir,secondDir) < 0) mainDir = secondDir;
    //            }

    //            return ConstraintNormal(mainDir);
    }

    ConstraintNormal createConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        defaulttype::Vector3 secondDir = d.second->getNormal().normalized();
        return ConstraintNormal(secondDir);
    }
};


int ContactDirectionClass = core::RegisterObject("ContactDirection")
.add< ContactDirection >();

}

}

