#pragma once

#include <sofa/constraintGeometry/BaseNormalHandler.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa::constraintGeometry {

template<class DataTypes>
class VectorPointNormalHandler : public BaseNormalHandler {
public:

    SOFA_CLASS(SOFA_TEMPLATE(VectorPointNormalHandler,DataTypes), BaseNormalHandler);

    Data<type::vector<type::Vector3>> d_normals;
	core::objectmodel::DataCallback c_callback;

    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<VectorPointGeometry<DataTypes>,PointGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    VectorPointGeometry()
    : d_normals(initData(&d_normals, "normals", "Vector of normals"))
    , l_geometry(initLink("geometry","Link to Geometry")){}


    void init()
	{
        for (unsigned i=0; i<d_normals.getValue().size(); i++)
        {
            l_geometry->getTopoProxIdx(i)->setNormal(d_normals.getValue()[i]);
        }

		c_callback.addInputs({&d_normals});
        c_callback.addCallback(std::bind(&VectorPointGeometry::makeDirty,this));
	}

	void makeDirty()
	{
		m_dirty = true;
	}

    void prepareDetection() override
	{
		if(m_dirty)
			init();
		m_dirty = false;
	}

	bool m_dirty;

};


}
