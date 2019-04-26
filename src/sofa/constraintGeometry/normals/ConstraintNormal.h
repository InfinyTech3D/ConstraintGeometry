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

    ConstraintNormal(const helper::vector<defaulttype::Vector3> & vec) {
        for (unsigned i=0;i<vec.size();i++)
            m_dirs.push_back(vec[i].normalized());
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
    static ConstraintNormal createFrame(defaulttype::Vector3 N1 = defaulttype::Vector3(1,0,0)) {
        ConstraintNormal CN(N1);
        CN.addFrame();
        return CN;
    }


    void addFrame() {
        m_dirs.resize(3,defaulttype::Vector3(0,0,0));

        if (m_dirs[0].norm()<std::numeric_limits<double>::epsilon()) m_dirs[0] = defaulttype::Vector3(1,0,0);
        m_dirs[0].normalize();

        //gram schmidt if the normal was provided
        m_dirs[1] -= m_dirs[0] * dot(m_dirs[0],m_dirs[1]);
        if (m_dirs[1].norm() < std::numeric_limits<double>::epsilon()) {
            m_dirs[1] = cross(m_dirs[0],defaulttype::Vector3(0,1,0));
            if (m_dirs[1].norm()<std::numeric_limits<double>::epsilon())
                m_dirs[1] = cross(m_dirs[0],defaulttype::Vector3(0,0,1));
        }
        m_dirs[1].normalize();

        m_dirs[2] = cross(m_dirs[0],m_dirs[1]);
        m_dirs[2].normalize();
    }

protected:
    helper::vector<defaulttype::Vector3> m_dirs;

};

}

}
