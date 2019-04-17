#pragma once

#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/constraintGeometry/ConstraintNormal.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint;

/*!
 * \brief The DataConstraintNormal class is the default class
 * generating a list of constraint directions
 * The class also serves as a template for directionGenerators
 * only the constructor and static defaultGetNormals should be redefined though
 */
class DataConstraintNormal : public helper::vector<defaulttype::Vector3> {
public :
    typedef std::function<ConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d)> GeneratorFunction;
    typedef std::pair<collisionAlgorithm::BaseProximity::SPtr,collisionAlgorithm::BaseProximity::SPtr> PairDetection;

    // std::placeholders::_1 is the first parameter
    DataConstraintNormal(GeneratorFunction fct = std::bind(&defaultGetNormals, std::placeholders::_1))
        : m_functor(fct)
    {}

    ConstraintNormal getConstraintNormal(const collisionAlgorithm::DetectionOutput::PairDetection & d) const {
        if (size() == 0)
            return m_functor(d); // return the functor value
        return ConstraintNormal(*this); //use helper::vector Constructor
    }

protected:

    static ConstraintNormal defaultGetNormals(const collisionAlgorithm::DetectionOutput::PairDetection & d) {
        return ConstraintNormal(
            (d.first->getPosition() - d.second->getPosition())
            .normalized()
        );
    }

    GeneratorFunction m_functor;
};

/*!
 * \brief The ContactNormal class is the container class for direction constraints
 */
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
