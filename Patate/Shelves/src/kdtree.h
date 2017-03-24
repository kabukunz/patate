/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.

 \authors Gael Guennebaud, Nicolas Mellado

*/


#ifndef _SHELVES_KDTREE_
#define _SHELVES_KDTREE_

#include "bbox.h"

#include "Eigen/Core"

#include <limits>
#include <iostream>
#include <numeric>  //iota

// max depth of the tree
#define KD_MAX_DEPTH 32

// number of neighbors
#define KD_POINT_PER_CELL 64


namespace Shelves{

/*!
  <h3>Generation</h3>
  You can create the KdTree in two way :
   - Simple pass construction : easy to use, but use a full vertices memory
   copy :
   \code
    std::vector < KdTree<float>::VectorType > data(60);
    int i = 0;

    for ( ; i != 30; i++ )
        data.push_back(KdTree<float>::VectorType::Random());

    data.push_back(KdTree<float>::VectorType(0.5,0.5,0.5));   // we must find this vertex
    i++;

    for ( ; i != 60; i++ )
        data.push_back(KdTree<float>::VectorType::Random());

    KdTree<float> t ( data ); // Memory copy
   \endcode

   - Per-vertex pass construction : more code, but avoid copy, and so better
   performances are expected :
    \code
    KdTree<float> t ( 60 );
    int i = 0;

    for ( ; i != 30; i++ )
        t.set( i ,KdTree<float>::VectorType::Random() );

    t.set(i, KdTree<float>::VectorType(0.5,0.5,0.5));   // we must find this vertex
    i++;

    for ( ; i != 60; i++ )
        t.set( i ,KdTree<float>::VectorType::Random() );

    t.finalize();  // the real creation of the KdTree
    \endcode
    This version is expected to be more efficient.


  <h3>Closest-Point query</h3>
    It is important to note that in the case of multiple neighbors request,
    the result isn't sorted (see HeapMaxPriorityQueue for more explanations).
    So if you want to get the closest point, you must perform a single request.

    You must specify the size of the request using setMaxNofNeighbors.

    \code
    t.setMaxNofNeighbors(1);
    t.doQueryK( vec3(0,0,0) );
    vec3 result           = t.getNeighbor( 0 ).p;
    unsigned int resultId = t.getNeighborId( 0 );

    cout << resultId << endl;
    cout << result(0) << " " << result(1) << " " << result(2) << endl;
    cout << t.getNeighborSquaredDistance( 0 ) << endl;
    \endcode
    \ingroup groupGeometry
  */
template<typename _Scalar, typename _Index = int >
class KdTree
{
public:
    struct KdNode
    {
        union {
            struct {
                float splitValue;
                unsigned int firstChildId:24;
                unsigned int dim:2;
                unsigned int leaf:1;
            };
            struct {
                unsigned int start;
                unsigned short size;
            };
        };
    };

    typedef _Scalar Scalar;
    typedef _Index  Index;

    static constexpr Index invalidIndex() { return -1; }

    typedef Eigen::Matrix<Scalar,3,1> VectorType;
    typedef AABB3D<Scalar> AxisAlignedBoxType;

    typedef std::vector<KdNode>      NodeList;
    typedef std::vector<VectorType>  PointList;
    typedef std::vector<Index>       IndexList;

    inline const NodeList&   _getNodes   (void) { return mNodes;   }
    inline const PointList&  _getPoints  (void) { return mPoints;  }
    inline const PointList&  _getIndices (void) { return mIndices;  }


public:
    //! Create the Kd-Tree using memory copy.
    KdTree(const PointList& points,
           unsigned int nofPointsPerCell = KD_POINT_PER_CELL,
           unsigned int maxDepth = KD_MAX_DEPTH );

    //! Create a void KdTree
    KdTree( unsigned int size = 0,
            unsigned int nofPointsPerCell = KD_POINT_PER_CELL,
            unsigned int maxDepth = KD_MAX_DEPTH );

    //! Add a new vertex in the KdTree
    template <class VectorDerived>
    inline void add( const VectorDerived &p ){
         // this is ok since the memory has been reserved at construction time
        mPoints.push_back(p);
        mIndices.push_back(mIndices.size());
        mAABB.extendTo(p);
    }

    inline void add(Scalar *position){
        add(Eigen::Map< Eigen::Matrix<Scalar, 3, 1> >(position));
    }

    //! Finalize the creation of the KdTree
    inline
    void finalize( );

    inline const AxisAlignedBoxType& aabb() const  {return mAABB; }

    ~KdTree();

    inline void doQueryK(const VectorType& p);

    /*!
     * \brief Performs distance query and return vector coordinates
     */
    template<typename Container = std::vector<VectorType> >
    inline void
    doQueryDist(const VectorType& queryPoint,
                Scalar sqdist,
                Container& result){
        _doQueryDistIndicesWithFunctor(queryPoint,
                                      sqdist,
                                      [&result,this](unsigned int i){
            result.push_back(typename Container::value_type(mPoints[i]));
        });
    }

    /*!
     * \brief Performs distance query and return indices
     */
    template<typename IndexContainer = std::vector<Index> >
    inline void
    doQueryDistIndices(const VectorType& queryPoint,
                       float sqdist,
                       IndexContainer& result){
        _doQueryDistIndicesWithFunctor(queryPoint,
                                      sqdist,
                                      [&result,this](unsigned int i){
            result.push_back(typename IndexContainer::value_type(mIndices[i]));
        });
    }

    /*!
     * \brief Finds the closest element index within the range [0:sqrt(sqdist)]
     * \param currentId Index of the querypoint if it belongs to the tree
     */
    inline Index
    doQueryRestrictedClosestIndex(const VectorType& queryPoint,
                                  Scalar sqdist,
                                  int currentId = -1);

     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    //! element of the stack
    struct QueryNode
    {
        inline QueryNode() {}
        inline QueryNode(unsigned int id) : nodeId(id) {}
        //! id of the next node
        unsigned int nodeId;
        //! squared distance to the next node
        Scalar sq;
    };

    /*!
      Used to build the tree: split the subset [start..end[ according to dim
      and splitValue, and returns the index of the first element of the second
      subset.
      */
    inline
    unsigned int split(int start, int end, unsigned int dim, Scalar splitValue);

    void createTree(unsigned int nodeId,
                    unsigned int start,
                    unsigned int end,
                    unsigned int level,
                    unsigned int targetCellsize,
                    unsigned int targetMaxDepth);


    /*!
     * \brief Performs distance query and pass the internal id to a functor
     */
    template<typename Functor >
    inline void
    _doQueryDistIndicesWithFunctor(const VectorType& queryPoint,
                                   float sqdist,
                                   Functor f);
protected:

    PointList  mPoints;
    IndexList  mIndices;
    AxisAlignedBoxType mAABB;
    NodeList   mNodes;
    QueryNode mNodeStack[64];

    unsigned int _nofPointsPerCell;
    unsigned int _maxDepth;
};





/*!
  \see KdTree(unsigned int size, unsigned int nofPointsPerCell, unsigned int maxDepth)
  */
template<typename Scalar, typename Index>
KdTree<Scalar, Index>::KdTree(const PointList& points,
                       unsigned int nofPointsPerCell,
                       unsigned int maxDepth)
    : mPoints(points),
      mIndices(points.size()),
      mAABB(points.cbegin(), points.cend()),
      _nofPointsPerCell(nofPointsPerCell),
      _maxDepth(maxDepth)
{
    std::iota (mIndices.begin(), mIndices.end(), 0); // Fill with 0, 1, ..., 99.
    finalize();
}

/*!
  Second way to create the KdTree, in two time. You must call finalize()
  before requesting for closest points.

  \see finalize()
  */
template<typename Scalar, typename Index>
KdTree<Scalar, Index>::KdTree(unsigned int size,
                       unsigned int nofPointsPerCell,
                       unsigned int maxDepth)
    : _nofPointsPerCell(nofPointsPerCell),
      _maxDepth(maxDepth)
{
    mPoints.reserve(size);
    mIndices.reserve(size);
}

template<typename Scalar, typename Index>
void
KdTree<Scalar, Index>::finalize()
{
    mNodes.clear();
    mNodes.reserve(4*mPoints.size()/_nofPointsPerCell);
    mNodes.push_back(KdNode());
    mNodes.back().leaf = 0;
    std::cout << "create tree" << std::endl;
    createTree(0, 0, mPoints.size(), 1, _nofPointsPerCell, _maxDepth);
    std::cout << "create tree ... DONE (" << mPoints.size() << " points)" << std::endl;
}

template<typename Scalar, typename Index>
KdTree<Scalar, Index>::~KdTree()
{
}


/*!

  This algorithm uses the simple distance to the split plane to prune nodes.
  A more elaborated approach consists to track the closest corner of the cell
  relatively to the current query point. This strategy allows to save about 5%
  of the leaves. However, in practice the slight overhead due to this tracking
  reduces the overall performance.

  This algorithm also use a simple stack while a priority queue using the squared
  distances to the cells as a priority values allows to save about 10% of the leaves.
  But, again, priority queue insertions and deletions are quite involved, and therefore
  a simple stack is by far much faster.

  The optionnal parameter currentId is used when the query point is
  stored in the tree, and must thus be avoided during the query
*/
template<typename Scalar, typename Index>
Index
KdTree<Scalar, Index>::doQueryRestrictedClosestIndex(
        const VectorType& queryPoint,
        Scalar sqdist,
        int currentId)
{

    Index  cl_id   = invalidIndex();
    Scalar cl_dist = sqdist;

    mNodeStack[0].nodeId = 0;
    mNodeStack[0].sq = 0.f;
    unsigned int count = 1;

    //int nbLoop = 0;
    while (count)
    {
        //nbLoop++;
        QueryNode& qnode = mNodeStack[count-1];
        KdNode   & node  = mNodes[qnode.nodeId];

        if (qnode.sq < cl_dist)
        {
            if (node.leaf)
            {
                --count; // pop
                const int end = node.start+node.size;
                for (int i=node.start ; i<end ; ++i){
                    const Scalar sqdist = (queryPoint - mPoints[i]).squaredNorm();
                    if (sqdist <= cl_dist && mIndices[i] != currentId){
                        cl_dist = sqdist;
                        cl_id   = mIndices[i];
                    }
                }
            }
            else
            {
                // replace the stack top by the farthest and push the closest
                const Scalar new_off = queryPoint[node.dim] - node.splitValue;

                //std::cout << "new_off = " << new_off << std::endl;

                if (new_off < 0.)
                {
                    mNodeStack[count].nodeId  = node.firstChildId; // stack top the farthest
                    qnode.nodeId = node.firstChildId+1;            // push the closest
                }
                else
                {
                    mNodeStack[count].nodeId  = node.firstChildId+1;
                    qnode.nodeId = node.firstChildId;
                }
                mNodeStack[count].sq = qnode.sq;
                qnode.sq = new_off*new_off;
                ++count;
            }
        }
        else
        {
            // pop
            --count;
        }
    }
    return cl_id;
}

/*!
  \see doQueryRestrictedClosest For more information about the algorithm.

  This function is an alternative to doQueryK(const VectorType& queryPoint)
  that allow to perform the query by requesting a maximum distance instead of
  neighborhood size.
 */
template<typename Scalar, typename Index>
template<typename Functor >
void
KdTree<Scalar, Index>::_doQueryDistIndicesWithFunctor(
        const VectorType& queryPoint,
        float sqdist,
        Functor f)
{
    mNodeStack[0].nodeId = 0;
    mNodeStack[0].sq = 0.f;
    unsigned int count = 1;

    while (count)
    {
        QueryNode& qnode = mNodeStack[count-1];
        KdNode   & node  = mNodes[qnode.nodeId];

        if (qnode.sq < sqdist)
        {
            if (node.leaf)
            {
                --count; // pop
                unsigned int end = node.start+node.size;
                for (unsigned int i=node.start ; i<end ; ++i)
                    if ( (queryPoint - mPoints[i]).squaredNorm() < sqdist){
                        f(i);
                    }
            }
            else
            {
                // replace the stack top by the farthest and push the closest
                Scalar new_off = queryPoint[node.dim] - node.splitValue;
                if (new_off < 0.)
                {
                    mNodeStack[count].nodeId  = node.firstChildId;
                    qnode.nodeId = node.firstChildId+1;
                }
                else
                {
                    mNodeStack[count].nodeId  = node.firstChildId+1;
                    qnode.nodeId = node.firstChildId;
                }
                mNodeStack[count].sq = qnode.sq;
                qnode.sq = new_off*new_off;
                ++count;
            }
        }
        else
        {
            // pop
            --count;
        }
    }
}

template<typename Scalar, typename Index>
unsigned int KdTree<Scalar, Index>::split(int start, int end, unsigned int dim, Scalar splitValue)
{
    int l(start), r(end-1);
    for ( ; l<r ; ++l, --r)
    {
        while (l < end && mPoints[l][dim] < splitValue)
            l++;
        while (r >= start && mPoints[r][dim] >= splitValue)
            r--;
        if (l > r)
            break;
        std::swap(mPoints[l],mPoints[r]);
        std::swap(mIndices[l],mIndices[r]);
    }
    return (mPoints[l][dim] < splitValue ? l+1 : l);
}

/*!

    Recursively builds the kdtree


   The heuristic is the following:
    - if the number of points in the node is lower than targetCellsize then make a leaf
    - else compute the AABB of the points of the node and split it at the middle of
      the largest AABB dimension.

   This strategy might look not optimal because it does not explicitly prune empty space,
   unlike more advanced SAH-like techniques used for RT. On the other hand it leads to a shorter tree,
   faster to traverse and our experience shown that in the special case of kNN queries,
   this strategy is indeed more efficient (and much faster to build). Moreover, for volume data
   (e.g., fluid simulation) pruning the empty space is useless.

   Actually, storing at each node the exact AABB (we therefore have a binary BVH) allows
   to prune only about 10% of the leaves, but the overhead of this pruning (ball/ABBB intersection)
   is more expensive than the gain it provides and the memory consumption is x4 higher !
*/
template<typename Scalar, typename Index>
void KdTree<Scalar, Index>::createTree(unsigned int nodeId, unsigned int start, unsigned int end, unsigned int level, unsigned int targetCellSize, unsigned int targetMaxDepth)
{

    KdNode& node = mNodes[nodeId];
    AxisAlignedBoxType aabb;
    //aabb.Set(mPoints[start]);
    for (unsigned int i=start ; i<end ; ++i)
        aabb.extendTo(mPoints[i]);

    VectorType diag =  Scalar(0.5) * (aabb.max()- aabb.min());
    typename VectorType::Index dim;

#ifdef DEBUG

//    std::cout << "createTree("
//              << nodeId << ", "
//              << start << ", "
//              << end << ", "
//              << level << ")"
//              << std::endl;

    if (std::isnan(diag.maxCoeff(&dim))){
        std::cerr << "NaN values discovered in the tree, abort" << std::endl;
        return;
    }
#else
    diag.maxCoeff(&dim);
#endif


#undef DEBUG
    node.dim = dim;
    node.splitValue = aabb.center()(dim);

    unsigned int midId = split(start, end, dim, node.splitValue);

    node.firstChildId = mNodes.size();

    {
        KdNode n;
        n.size = 0;
        mNodes.push_back(n);
        mNodes.push_back(n);
    }
    //mNodes << Node() << Node();
    //mNodes.resize(mNodes.size()+2);

    {
        // left child
        unsigned int childId = mNodes[nodeId].firstChildId;
        KdNode& child = mNodes[childId];
        if (midId-start <= targetCellSize || level>=targetMaxDepth)
        {
            child.leaf = 1;
            child.start = start;
            child.size = midId-start;
        }
        else
        {
            child.leaf = 0;
            createTree(childId, start, midId, level+1, targetCellSize, targetMaxDepth);
        }
    }

    {
        // right child
        unsigned int childId = mNodes[nodeId].firstChildId+1;
        KdNode& child = mNodes[childId];
        if (end-midId <= targetCellSize || level>=targetMaxDepth)
        {
            child.leaf = 1;
            child.start = midId;
            child.size = end-midId;
        }
        else
        {
            child.leaf = 0;
            createTree(childId, midId, end, level+1, targetCellSize, targetMaxDepth);
        }
    }
}
} //namespace Shelves


#endif // KDTREE_H
