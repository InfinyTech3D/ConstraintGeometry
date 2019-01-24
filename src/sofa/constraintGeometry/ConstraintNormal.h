#pragma once

#include <sofa/helper/vector_device.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint;

class ConstraintNormal {
    friend class InternalConstraint;

public:
    ConstraintNormal(defaulttype::Vector3 n1) {
        m_dirs.push_back(n1.normalized());
    }

    ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2) {
        m_dirs.push_back(n1.normalized());
        m_dirs.push_back(n2.normalized());
    }

    ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2,defaulttype::Vector3 n3) {
        m_dirs.push_back(n1.normalized());
        m_dirs.push_back(n2.normalized());
        m_dirs.push_back(n3.normalized());
    }

    static ConstraintNormal createFrame(defaulttype::Vector3 N1 = defaulttype::Vector3()) {
        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
        defaulttype::Vector3 N3 = cross(N1,N2);

        return ConstraintNormal(N1,N2,N3);
    }

    static ConstraintNormal createFromDetection(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
//            defaulttype::Vector3 mainDir = d.getFirstProximity()->getPosition() - d.getSecondProximity()->getPosition();

//////            if (mainDir.norm()>0.00000001) return ConstraintNormal(mainDir);

//            defaulttype::Vector3 firstDir = -d.getFirstProximity()->getNormal().normalized();
        defaulttype::Vector3 secondDir = d.second->getNormal().normalized();

//            return ConstraintNormal(mainDir.normalized() + firstDir + secondDir);
//            return ConstraintNormal(firstDir);
        return ConstraintNormal(secondDir);


//            defaulttype::Vector3 mainDir = d.getSecondProximity()->getNormal();//pair.first->getPosition() - pair.second->getPosition();
//            defaulttype::Vector3 secondDir = -d.getFirstProximity()->getNormal();

//            if (mainDir.norm()<0.01) mainDir = secondDir;
//            else {
//                mainDir.normalize();
//                if (dot(mainDir,secondDir) < 0) mainDir = secondDir;
//            }

//            return ConstraintNormal(mainDir);
    }

    static ConstraintNormal createFrameFromDetection(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return createFrame(ConstraintNormal::createFromDetection(d).m_dirs[0]);
    }

    unsigned size() {
        return m_dirs.size();
    }

protected:
    helper::vector<defaulttype::Vector3> m_dirs;
};

}

}
