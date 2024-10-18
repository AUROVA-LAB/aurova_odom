#ifndef KDTREEDESC_H
#define KDTREEDESC_H

#include <vector>
#include <nanoflann.hpp>
#include "STDesc.h"

struct STDescCloud {
    std::vector<STDesc> descriptors;

    inline size_t kdtree_get_point_count() const { return descriptors.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return descriptors[idx].toVector()[dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }

};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, STDescCloud>,
    STDescCloud,
    18 /* Dimensiones (3 side_length + 3 angle + 3 center + 3*3 vertices) */
> STDescKDTree;

STDesc findClosestDescriptor(const STDesc& query, STDescKDTree& index, STDescCloud& cloud);

#endif // KDTREEDESC_H
