/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.

 \authors Gael Guennebaud, Nicolas Mellado

*/


#ifndef _SHELVES_BOUNDEDHEAPPRIORITYQUEUE_
#define _SHELVES_BOUNDEDHEAPPRIORITYQUEUE_

#include <algorithm>


namespace Shelves{

template <typename Index, typename Weight>
struct BoundedHeapPriorityElement
{
    Weight weight;
    Index index;
};

/*!
    \brief Base class for bounded-size priority queue

    Be carrefull that the resulting queue isn't sorted, but all the elements
    that are present at the end of the insertion process are all the
    smallest/greatest of the set (ie. the SIZE smallest elements in the following
    code).

    \snippet Shelves/boundedHeapPriorityQueue.cpp Using BoundedHeapMinPriorityQueue

*/

template <typename Index,
          typename Weight,
          typename Comp > class BoundedHeapPriorityQueueBase
{

public:
    using Element = BoundedHeapPriorityElement<Index, Weight>;

    BoundedHeapPriorityQueueBase(void)
    {
        mElements = 0;
        mMaxSize  = 0;
        mCount    = 0;
    }

    inline void reserve(int maxSize)
    {
        if (mMaxSize!=maxSize)
        {
            mMaxSize = maxSize;
            delete[] mElements;
            mElements = new Element[mMaxSize];
            mpOffsetedElements = (mElements-1);
        }
        reset();
    }

    /*!
     * \brief reset Reset the queue
     * \warning Does not free memory
     */
    inline void reset() { mCount = 0; }

    inline bool isFull() const { return mCount == mMaxSize; }

    /*!
     * returns number of elements inserted in queue
     */
    inline int size() const { return mCount; }

    constexpr Element* begin() { return mElements; }
    constexpr Element* end()   { return mElements+mCount; }

    inline void insert(Index index, Weight weight)
    {
        Element e;
        e.index  = index;
        e.weight = weight;
        insert(e);
    }

    inline void insert(Element e)
    {
        Comp comp;
        if (mCount==mMaxSize)
        {
            if (comp (e, mElements[0]))
            {
                register int j, k;
                j = 1;
                k = 2;
                while (k <= mMaxSize)
                {
                    Element* z = &(mpOffsetedElements[k]);
                    if ((k < mMaxSize) && comp(*z, mpOffsetedElements[k+1]))
                        z = &(mpOffsetedElements[++k]);

                    if(! comp (e, *z))
                        break;
                    mpOffsetedElements[j] = *z;
                    j = k;
                    k = 2 * j;
                }
                mpOffsetedElements[j] = e;
            }
        }
        else
        {
            int i, j;
            i = ++mCount;
            while (i >= 2)
            {
                j = i >> 1;
                Element& y = mpOffsetedElements[j];
                if( ! comp (y, e))
                    break;
                mpOffsetedElements[i] = y;
                i = j;
            }
            mpOffsetedElements[i] = e;
        }
    }

protected:

    int mCount;
    int mMaxSize;
    Element* mElements;
    Element* mpOffsetedElements;
}; // class HeapMaxPriorityQueue

template <typename Index,  typename Weight>
struct BoundedHeapMaxPriorityComp {
    using Element = BoundedHeapPriorityElement<Index, Weight>;
    inline bool operator()(const Element& lhs, const Element& rhs) {
        return lhs.weight > rhs.weight;
    }
};

template <typename Index,  typename Weight>
struct BoundedHeapMinPriorityComp {
    using Element = BoundedHeapPriorityElement<Index, Weight>;
    inline bool operator()(const Element& lhs, const Element& rhs) {
        return lhs.weight < rhs.weight;
    }
};

//! \brief Priority Queue storing the smallest elements
template <typename Index,  typename Weight>
using BoundedHeapMinPriorityQueue = BoundedHeapPriorityQueueBase<Index, Weight, BoundedHeapMinPriorityComp<Index, Weight>>;

//! \brief Priority Queue storing the greatest elements
template <typename Index,  typename Weight>
using BoundedHeapMaxPriorityQueue = BoundedHeapPriorityQueueBase<Index, Weight, BoundedHeapMaxPriorityComp<Index, Weight>>;

} // namespace Shelves

#endif
