#pragma once

#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint;

/*!
 * \brief The ConstraintNormal class
 */
class ConstraintNormal {
    friend class InternalConstraint;

public:

    typedef std::function<double(const collisionAlgorithm::PairDetection & , const defaulttype::Vector3 &)> ViolationFunction;

    static double defaultViolationFunction(const collisionAlgorithm::PairDetection & detection, const defaulttype::Vector3 & normal) {
        const defaulttype::Vector3 & PFree = detection.first->getPosition(core::VecCoordId::freePosition());
        const defaulttype::Vector3 & QFree = detection.second->getPosition(core::VecCoordId::freePosition());
        return dot(PFree - QFree, normal);
    }

    ConstraintNormal() {}

    ConstraintNormal(const defaulttype::Vector3 & N, ViolationFunction f = std::bind(&ConstraintNormal::defaultViolationFunction, std::placeholders::_1, std::placeholders::_2)) {
        m_dirs.push_back(N.normalized());
        m_functions.push_back(f);
    }

    ConstraintNormal & add(const defaulttype::Vector3 & N, ViolationFunction f = std::bind(&ConstraintNormal::defaultViolationFunction, std::placeholders::_1, std::placeholders::_2)) {
        m_dirs.push_back(N.normalized());
        m_functions.push_back(f);
        return *this;
    }

    /// returns ConstraintNormal vector's (m_dirs) size
    unsigned size() const {
        return m_dirs.size();
    }

    ConstraintNormal & addOrthogonalDirection(ViolationFunction f = std::bind(&ConstraintNormal::defaultViolationFunction, std::placeholders::_1, std::placeholders::_2)) {
        if (m_dirs.size() == 0) {
            m_dirs.push_back(defaulttype::Vector3(1,0,0));
            m_functions.push_back(f);
            return *this;
        }

        if (m_dirs.size() == 1) {
            //gram schmidt orthogonalization
            defaulttype::Vector3 gramm_schmidt = defaulttype::Vector3(0,1,0) - m_dirs[0] * dot(m_dirs[0],defaulttype::Vector3(0,1,0));

            //Change with othogonalization around Z if the initial direction is aligned with Y
            if (gramm_schmidt.norm() < std::numeric_limits<double>::epsilon()) gramm_schmidt = defaulttype::Vector3(0,0,1) - m_dirs[0] * dot(m_dirs[0],defaulttype::Vector3(0,0,1));

            m_dirs.push_back(gramm_schmidt.normalized());
            m_functions.push_back(f);
            return *this;
        }

        if (m_dirs.size() == 2) {
            m_dirs.push_back(cross(m_dirs[0],m_dirs[1]).normalized());
            m_functions.push_back(f);
            return *this;
        }

        return *this;
    }

    void computeViolations(unsigned  cid, collisionAlgorithm::PairDetection d, defaulttype::BaseVector * delta) const {
        for (unsigned i=0;i<m_dirs.size();i++) {
            double v = m_functions[i](d, m_dirs[i]);
            delta->set(cid + i, v);
        }
    }

    const defaulttype::Vector3 operator[](int i) const {
        return m_dirs[i];
    }

    ConstraintNormal& operator=(const ConstraintNormal& CN)
    {
        if(&CN == this)
            return *this;
        this->m_dirs = CN.m_dirs;
        this->m_functions = CN.m_functions;

        return *this;

    }


    void UpdateConstraintViolationWithProximityPosition(unsigned  cid, const collisionAlgorithm::PairDetection & detection, defaulttype::Vec3 prox_from, bool getF, defaulttype::Vec3 prox_dest, bool getD, defaulttype::BaseVector * delta) const {
        defaulttype::Vector3 PFree = detection.first->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vector3 QFree = detection.second->getPosition(core::VecCoordId::freePosition());
        defaulttype::Vec3 freeMotion = detection.first->getPosition(core::VecCoordId::freePosition()) - detection.first->getPosition(core::VecCoordId::position());
        if(getF) PFree = prox_from;
        if(getD) QFree = prox_dest;
        for (unsigned i=0;i<m_dirs.size();i++) {
            double v = dot(PFree - QFree, m_dirs[i]);
            delta->set(cid*m_dirs.size() + i, v);
        }
    }

    void printDirections(){
        for(int i=0; i<m_dirs.size(); i++){
            std::cout<<"direction = "<<m_dirs[i]<<std::endl;
        }
    }

protected:
    //pai of directions (vec3) and function to compute the violation of a par proximity
    helper::vector<defaulttype::Vector3> m_dirs;
    helper::vector<ViolationFunction> m_functions;

};

}

}
