///******************************************************************************
//*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
//*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
//*                                                                             *
//* This library is free software; you can redistribute it and/or modify it     *
//* under the terms of the GNU Lesser General Public License as published by    *
//* the Free Software Foundation; either version 2.1 of the License, or (at     *
//* your option) any later version.                                             *
//*                                                                             *
//* This library is distributed in the hope that it will be useful, but WITHOUT *
//* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
//* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
//* for more details.                                                           *
//*                                                                             *
//* You should have received a copy of the GNU Lesser General Public License    *
//* along with this library; if not, write to the Free Software Foundation,     *
//* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
//*******************************************************************************
//*                               SOFA :: Modules                               *
//*                                                                             *
//* Authors: The SOFA Team and external contributors (see Authors.txt)          *
//*                                                                             *
//* Contact information: contact@sofa-framework.org                             *
//******************************************************************************/
//#ifndef SOFA_COMPONENT_AABBGeometry_H
//#define SOFA_COMPONENT_AABBGeometry_H

//#include "algorithm/CollisionDetectionAlgorithm.h"
//#include <sofa/core/behavior/ForceField.h>
//#include <sofa/core/behavior/MechanicalState.h>
//#include <sofa/core/objectmodel/Data.h>
//#include <sofa/defaulttype/VecTypes.h>
//#include "ConstraintProximity.h"

//namespace sofa {

//namespace core {

//namespace behavior {

//class AABBGeometry : public BaseGeometry, public BroadPhase
//{
//public:
//    SOFA_CLASS(AABBGeometry , BaseGeometry);

//    typedef sofa::defaulttype::Vec3dTypes DataTypes;
//    typedef DataTypes::VecCoord VecCoord;

//    Data<defaulttype::Vec3i> d_nbox;
//    Data<std::string> d_geometry;

//    class ElementAABBIterator : public ElementIterator {
//    public:
//        ElementAABBIterator(BaseGeometry * geo,std::set<int> set) {
//            m_selectedElements = set;
//            it = m_selectedElements.begin();
//            m_geo = geo;
//        }

//        bool next() {
//            it++;
//            return it != m_selectedElements.end();
//        }

//        ConstraintElementPtr getElement() {
//            return m_geo->getElement(*it);
//        }

//    private:
//        std::set<int> m_selectedElements;
//        std::set<int>::iterator it;
//        BaseGeometry * m_geo;
//    };

//    AABBGeometry();

//    void draw(const core::visual::VisualParams */*vparams*/);

//    void init();

//    void prepareDetection();

//    virtual ElementIteratorPtr getBroadPhaseIterator(ConstraintElementPtr elmt);

//    virtual int getNbElements() const ;

//    virtual ConstraintElementPtr getElement(unsigned eid) const;

//protected:
//    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
//    helper::vector<helper::vector<helper::vector<helper::vector<unsigned> > > >  m_indexedElement;
//    BaseGeometry * m_geo;

//    void fillTriangleSet(int d,defaulttype::Vec3i cbox,std::vector<int> & selectTriangles);

//    void getCloseElements(const ConstraintElementPtr pinfo, std::set<int> & selectElements);

//    void getCloseElements(defaulttype::Vec3i cbox, std::vector<int> & selectElements);
//};






//} // namespace forcefield

//} // namespace component

//} // namespace sofa


//#endif // NeedleLinearDescription_H
