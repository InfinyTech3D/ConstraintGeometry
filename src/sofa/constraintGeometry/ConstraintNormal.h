#pragma once

#include <sofa/type/vector.h>
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

    typedef std::function<double(const collisionAlgorithm::PairDetection & , const type::Vector3 &)> ViolationFunction;

    static double defaultViolationFunction(const collisionAlgorithm::PairDetection & detection, const type::Vector3 & normal) {
        const type::Vector3 & PFree = detection.first->getPosition(core::VecCoordId::freePosition());
        const type::Vector3 & QFree = detection.second->getPosition(core::VecCoordId::freePosition());
        return dot(PFree - QFree, normal);
    }

    ConstraintNormal() {}

    ConstraintNormal(const ConstraintNormal & cn) {
        m_dirs = cn.m_dirs;
        m_functions = cn.m_functions;
    }

    ConstraintNormal(const type::Vector3 & N, ViolationFunction f = std::bind(&ConstraintNormal::defaultViolationFunction, std::placeholders::_1, std::placeholders::_2)) {
        m_dirs.push_back(N.normalized());
        m_functions.push_back(f);
    }

    ConstraintNormal & add(const type::Vector3 & N, ViolationFunction f = std::bind(&ConstraintNormal::defaultViolationFunction, std::placeholders::_1, std::placeholders::_2)) {
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
            m_dirs.push_back(type::Vector3(1,0,0));
            m_functions.push_back(f);
            return *this;
        }

        if (m_dirs.size() == 1) {
            //gram schmidt orthogonalization
            type::Vector3 gramm_schmidt = type::Vector3(0,1,0) - m_dirs[0] * dot(m_dirs[0],type::Vector3(0,1,0));

            //Change with othogonalization around Z if the initial direction is aligned with Y
            if (gramm_schmidt.norm() < std::numeric_limits<double>::epsilon()) gramm_schmidt = type::Vector3(0,0,1) - m_dirs[0] * dot(m_dirs[0],type::Vector3(0,0,1));

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

    void computeViolations(unsigned  cid, collisionAlgorithm::PairDetection d, linearalgebra::BaseVector * delta) const {
        for (unsigned i=0;i<m_dirs.size();i++) {
            double v = m_functions[i](d, m_dirs[i]);
            delta->set(cid + i, v);
        }
    }

    const type::Vector3 operator[](int i) const {
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

protected:
    //pai of directions (vec3) and function to compute the violation of a par proximity
    sofa::type::vector<type::Vector3> m_dirs;
    sofa::type::vector<ViolationFunction> m_functions;

};

}

}
