#include "include/KDTree_STD.h"

STDesc findClosestDescriptor(const STDesc& query, STDescKDTree& index, STDescCloud& cloud) {

    std::vector<double> queryVec = query.toVector();

    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);

    index.findNeighbors(resultSet, queryVec.data(), nanoflann::SearchParameters(10));

    return cloud.descriptors[ret_index];
}
