/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYALGORITHMS_H
#define SOFA_COMPONENT_TOPOLOGY_TETRAHEDRONSETTOPOLOGYALGORITHMS_H

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/Vec.h>
#include <SofaBaseTopology/TriangleSetTopologyAlgorithms.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TetrahedronSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaBaseTopology/TetrahedronSetGeometryAlgorithms.h>
#include <algorithm>
#include <functional>

namespace sofa
{

namespace component
{

namespace topology
{

/**
* A class that performs topology algorithms on an TetrahedronSet.
*/
template<class DataTypes>
class TetrahedronCutting : public core::objectmodel::BaseObject {

    public:
        SOFA_CLASS(SOFA_TEMPLATE(TetrahedronCutting,DataTypes),core::objectmodel::BaseObject);

        typedef typename DataTypes::Coord Coord;
        typedef typename DataTypes::VecCoord VecCoord;
        typedef typename Coord::value_type Real;
        typedef sofa::core::topology::BaseMeshTopology::SeqEdges SeqEdges;
        typedef sofa::core::topology::BaseMeshTopology::TetraID TetraID;
        typedef sofa::core::topology::BaseMeshTopology::Tetra Tetra;
        typedef sofa::core::topology::BaseMeshTopology::EdgeID EdgeID;
        typedef sofa::core::topology::BaseMeshTopology::Edge Edge;

        typedef sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron TrianglesInTetrahedron;
        typedef sofa::core::topology::BaseMeshTopology::EdgesInTetrahedron EdgesInTetrahedron;
        typedef sofa::core::topology::BaseMeshTopology::TetrahedraAroundEdge TetrahedraAroundEdge;
        typedef sofa::helper::vector<Tetra> TetraList;

        Data <double> f_rangeCoeff;
        Data <unsigned> f_resetCut;
        Data <bool> f_followCut;
        Data <bool> f_drawActiveEdge;
        Data <bool> f_drawAddedPoints;

        void subDivideTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2);
        void subDivideRestTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2);
        void carveTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2);
        void carveRestTetrahedronsWithPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2);

        virtual void init();
        void draw(const core::visual::VisualParams* vparams);

        void deleteTemporaryPoints();
        void deletePermanentPoints();
        void deletePermanentUnduplicatePoints();

        bool isPermanantAddedPoint(unsigned p);

    private:
        typedef struct {
            unsigned eid;
            unsigned e0;
            unsigned e1;
            double alpha;
        } Intersection;

        typedef struct s_TetraInfo {
            unsigned renumberedId[4];
            unsigned configuration;
            unsigned tid;

            unsigned ancTetra[4];// initial tetraid

            unsigned status0[4];// list of point uncut
            unsigned status1[4];// list of point cut 1 time
            unsigned status2[4];// list of point cut 2 times
            unsigned status3[4];// list of point cut 3 times
            unsigned status4[4];// list of duplicate points

            unsigned id0;// list of point uncut
            unsigned id1;// list of point cut 1 time
            unsigned id2;// list of point cut 2 times
            unsigned id3;// list of point cut 3 times
            unsigned id4;// list of duplicate points

            s_TetraInfo() {
                id0 = id1 = id2 = id3 = id4 = 0;
            }
        } TetraInfo;

        typedef struct {
            unsigned pid;
            unsigned a0;
            unsigned a1;
            int opp;
        } AddedPoint;

        typedef struct {            
            unsigned a0;
            unsigned a1;
            double alpha;
        } TemporaryPoint;

        sofa::component::topology::TetrahedronSetGeometryAlgorithms<DataTypes>* tetrahedronGeo;
        sofa::component::topology::TetrahedronSetTopologyContainer* m_container;
        sofa::component::topology::TetrahedronSetTopologyModifier* tetrahedronModifier;

        helper::vector<Intersection> intersectedEdge;        
        helper::vector<TemporaryPoint> temporaryPoints;
        helper::vector<AddedPoint> addedPoints;

        TetraList toBeAddedTetra;//To be added components
        sofa::helper::vector<TetraID> toBeAddedTetraIndex;//To be added components
        sofa::helper::vector<TetraID> toBeRemovedTetraIndex;//To be removed components
        sofa::helper::vector< sofa::helper::vector<unsigned> > toBeAddedPointAncestors; // ancestors to points added
        sofa::helper::vector< sofa::helper::vector<double> > toBeAddedPointcoefs; // coeffs of points added

        bool isPointInPlane(Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2, Coord& edgeDirec0, Coord& edgeDirec1, Coord& edgeDirec2, Coord& edgeDirec3, Coord& intersectedPoint);
        void CollisionDetectionBtwEdgesPlane(bool rest,Coord & normal,Coord& pre1, Coord& pre2, Coord& curr1, Coord& curr2);

        template<class Element>
        bool addSetVector(Element t,helper::vector<Element> & vector);

        template<class Element>
        bool contains(Element t,const helper::vector<Element> & vector);

        void propagateModifications();
        bool mergeSplittedPoints(const helper::vector<unsigned> & localBelow,const helper::vector<unsigned> & localAbove,helper::vector<unsigned> & globalBelow,helper::vector<unsigned> & globalAbove);
        void computeTetraAncesor(TetraID tid, TetraInfo & res);
        void computeTetraConfiguration(TetraInfo & res);
        void getTetraInfo(TetraID tid, TetraInfo & res);
        void getTetraInfoWithoutTemporaryCut(TetraID tid, TetraInfo & res,const helper::vector<unsigned> & skipID);

        void rec_serach(unsigned pid,helper::vector<unsigned> & pointSearch,helper::vector<unsigned> & linkedPoints);
        void createTemporaryPoint(unsigned a0,unsigned a1,double alpha);
        void createPermanantPoint(unsigned pid,unsigned a0,unsigned a1);
        unsigned createPointOnMesh(unsigned e0,unsigned e1,double alpha);
        bool isPermanantAddedPoint(unsigned p,unsigned & i);
        bool isPermanantDuplicatePoint(unsigned p);
        bool isTemporaryDuplicatedPoint(unsigned pid,unsigned & tmpID);
        bool isTemporaryDuplicatedPoint(unsigned pid);
        bool isPermanantBreakedEdge(unsigned e0,unsigned e1);
        bool isTemporaryBreakedEdge(unsigned e0,unsigned e1,unsigned & tmpID);
        bool isTemporaryBreakedEdge(unsigned e0,unsigned e1);
        void getTemporaryArnoudCut(unsigned e0,unsigned e1,helper::vector<unsigned> & tmpAroundID);
        void getTemporaryArnoudDuplicate(unsigned pid,helper::vector<unsigned> & tmpAroundID);
        void deleteTemporaryPoints(const helper::vector<unsigned> & tmpVecID);

        void deleteBelowTetra(const AddedPoint & add,const helper::vector<unsigned> & globalBelow);
        void breakTemporaryEdge(unsigned e0,unsigned e1,double alpha);
        void duplicateTemporaryPoint(unsigned pid);
        bool canDuplicateNodeInTetra(unsigned p,const TetraInfo & ti);
        bool canDuplicatePoint(unsigned p);
        bool canSplitEdgeInTetra(unsigned e0,unsigned e1,const TetraInfo & tid);
        bool isbreakableEdge(unsigned eid,unsigned e0,unsigned e1);
        void subdivideCollisionDetection();
        void carveCollisionDetection();

        void findAndSubdivide(TetraList & sub,TetraList & subdividedTetra,const AddedPoint & add);
        void cutTetra1(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0);
        void cutTetra2(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0,const AddedPoint & add1);
        void cutTetra3(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0,const AddedPoint & add1,const AddedPoint & add2);
        void cutTetra4(const Tetra & tetra,TetraList & subdividedTetra,const AddedPoint & add0,const AddedPoint & add1,const AddedPoint & add2,const AddedPoint & add3);
        void breakPermanantEdge(unsigned addedSize);
        void recTemporarySerach(unsigned pid,helper::vector<unsigned> & pointSearch,helper::vector<unsigned> & linkedPoints);
        void getTemporaryPointAround(const TemporaryPoint & tmp,helper::vector<unsigned> & pointAround);
        void splitTemporaryPointAround(unsigned addID,helper::vector<bool> & visited,helper::vector<unsigned> & snapTemporaryID,helper::vector<unsigned> & globalBelow,helper::vector<unsigned> & globalAbove);
        void snapTemporaryNodes();


        unsigned getPidIndex(unsigned id,const helper::vector<unsigned> & snapID);
        unsigned getOppIndex(unsigned id,const helper::vector<unsigned> & snapID);
        void rebuildTetra(const helper::vector<unsigned> & snapID,const helper::vector<unsigned> & globalSetPoint);
        void splitPermanentPointAround(unsigned addID,helper::vector<bool> & visited,helper::vector<unsigned> & snapPermanentID,helper::vector<unsigned> & globalBelow,helper::vector<unsigned> & globalAbove);
        void snapPermanentNodes();

        void resetCuts();
        int nbStep;

    protected:
        TetrahedronCutting();
        virtual ~TetrahedronCutting() {}
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_TOPOLOGY_TetrahedronCutting_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BASE_TOPOLOGY_API TetrahedronCutting<defaulttype::Vec3dTypes>;
#endif

#ifndef SOFA_DOUBLE
extern template class SOFA_BASE_TOPOLOGY_API TetrahedronCutting<defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace topology

} // namespace component

} // namespace sofa

#endif
