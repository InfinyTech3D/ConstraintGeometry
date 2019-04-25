#pragma once

#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa {

namespace constraintGeometry {

class InternalConstraint;

/*!
 * \brief The ConstraintNormal class
 */
class ConstraintNormal {
    friend class InternalConstraint;

public:
    ConstraintNormal() {}

    ConstraintNormal(const helper::vector<defaulttype::Vector3> & vec) {
        for (unsigned i=0;i<vec.size();i++)
            m_dirs.push_back(vec[i]);
    }

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

    /// returns ConstraintNormal vector's (m_dirs) size
    unsigned size() const {
        return m_dirs.size();
    }

    /*!
     * \brief createFrame creates a frame from a vector3
     *  using dot & cross product
     * \param N1 : vector3
     * \return ConstraintNormal
     */
    static ConstraintNormal createFrame(defaulttype::Vector3 N1 = defaulttype::Vector3()) {
        if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
        defaulttype::Vector3 N2 = cross(
            N1,
            (fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ?
                defaulttype::Vector3(0,0,1) :
                defaulttype::Vector3(0,1,0)
        );
        defaulttype::Vector3 N3 = cross(N1,N2);
        return ConstraintNormal(N1,N2,N3);
    }

protected:
    helper::vector<defaulttype::Vector3> m_dirs;
};

}

}
