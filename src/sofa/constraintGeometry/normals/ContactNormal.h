#pragma once

#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint;

class ContactNormal : public ConstraintNormal {
    friend class InternalConstraint;

public:

    ContactNormal(const collisionAlgorithm::PairDetection & d) {
        defaulttype::Vector3 mainDir = (d.first->getPosition() - d.second->getPosition()).normalized();

    //            defaulttype::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
        defaulttype::Vector3 secondDir = d.second->getNormal().normalized();

        if (dot(mainDir,secondDir)) mainDir=secondDir;

        m_dirs.push_back(secondDir);
    //            return ConstraintNormal::createFrame(secondDir, size);
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

    void addFriction() {
        const defaulttype::Vector3 & N1 = m_dirs[0];
        const defaulttype::Vector3 N2 = cross(
            N1,
            ((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ?
                 defaulttype::Vector3(0,0,1) :
                 defaulttype::Vector3(0,1,0)
            )
        );
        const defaulttype::Vector3 N3 = cross(N1,N2);

        m_dirs.push_back(N2);
        m_dirs.push_back(N3);
    }
};

}

}
