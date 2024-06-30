#ifndef NEAREST_NEIGHBOR_SEARCH_H
#define NEAREST_NEIGHBOR_SEARCH_H

#include <iostream>
#include <vector>
#include <memory>
#include "nanoflann.hpp"

using namespace std;
using namespace nanoflann;

class NearestNeighborSearch {
public:
    struct PointCloud {
        std::vector<std::vector<double>> pts;

        inline size_t kdtree_get_point_count() const { return pts.size(); }

        inline double kdtree_distance(const double *p1, const size_t idx_p2, size_t size) const {
            const double d0 = p1[0] - pts[idx_p2][0];
            const double d1 = p1[1] - pts[idx_p2][1];
            return d0 * d0 + d1 * d1;
        }

        inline double kdtree_get_pt(const size_t idx, int dim) const {
            return pts[idx][dim];
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
    };

    static shared_ptr<NearestNeighborSearch> create(const vector<pair<double, double>>& points) {
        auto nnSearch = make_shared<NearestNeighborSearch>();
        for (const auto& point : points) {
            nnSearch->addPoint(point.first, point.second);
        }
        nnSearch->buildIndex();
        return nnSearch;
    }

    NearestNeighborSearch() : index(2, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)) {}

    void addPoint(double x, double y) {
        cloud.pts.push_back({x, y});
    }

    void buildIndex() {
        index.buildIndex();
    }

    vector<pair<size_t, vector<double>>> findNeighbors(double x, double y, double radius) {
        double query_pt[2] = {x, y};
        vector<nanoflann::ResultItem<unsigned int, double>> ret_matches;
        index.radiusSearch(&query_pt[0], radius * radius, ret_matches);
        vector<pair<size_t, vector<double>>> results;
        results.reserve(ret_matches.size());
        for (const auto& match : ret_matches) {
            results.push_back({match.first, {cloud.kdtree_get_pt(match.first, 0), cloud.kdtree_get_pt(match.first, 1)}});
        }
        return results;
    }

private:
    PointCloud cloud;
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, PointCloud>,
        PointCloud,
        2 /* dimension */,
        unsigned int /* index type */
    > my_kd_tree_t;
    my_kd_tree_t index;
};

#endif // NEAREST_NEIGHBOR_SEARCH_H

