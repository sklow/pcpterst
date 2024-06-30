#ifndef THREE_DIMENSIONAL_POINT_HANDLER_H
#define THREE_DIMENSIONAL_POINT_HANDLER_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "NearestNeighborSearch.h"

using namespace std;
using namespace Eigen;

class ThreeDimensionalPointHandler {
public:
    static shared_ptr<ThreeDimensionalPointHandler> create(const vector<pair<double, double>>& points) {
        auto searcher = NearestNeighborSearch::create(points);
        auto handler = make_shared<ThreeDimensionalPointHandler>(searcher);
        return handler;
    }

    ThreeDimensionalPointHandler(shared_ptr<NearestNeighborSearch> nnSearch) : nnSearch(nnSearch) {}

    double performSearchAndFit(double x, double y, double radius, const vector<double>& extra_data) {
        auto neighbors = nnSearch->findNeighbors(x, y, radius);

        // 近傍点の個数を求める
        size_t neighbor_count = neighbors.size();
        // cout << neighbor_count << endl;

        // Create 3D points using the extra data
        vector<Vector3d> points3D;
        for (const auto& neighbor : neighbors) {
            size_t idx = neighbor.first;
            double z = extra_data[idx];
            points3D.emplace_back(neighbor.second[0], neighbor.second[1], z);
        }

        // Perform least squares fitting using Eigen
        auto coeffs = fitQuadratic(points3D);

        // Assign the fitted quadratic value to the target point
        double fitted_value = evaluateQuadratic(coeffs, x, y);
        return fitted_value;
    }

private:
    shared_ptr<NearestNeighborSearch> nnSearch;

    vector<double> fitQuadratic(const vector<Vector3d>& points) {
        int n = points.size();
        MatrixXd A(n, 6);
        VectorXd b(n);
        for (int i = 0; i < n; ++i) {
            double x = points[i][0];
            double y = points[i][1];
            double z = points[i][2];
            A(i, 0) = x * x;
            A(i, 1) = y * y;
            A(i, 2) = x * y;
            A(i, 3) = x;
            A(i, 4) = y;
            A(i, 5) = 1.0;
            b(i) = z;
        }

        VectorXd coeffs = A.colPivHouseholderQr().solve(b);
        return vector<double>(coeffs.data(), coeffs.data() + coeffs.size());
    }

    double evaluateQuadratic(const vector<double>& coeffs, double x, double y) {
        return coeffs[0] * x * x + coeffs[1] * y * y + coeffs[2] * x * y + coeffs[3] * x + coeffs[4] * y + coeffs[5];
    }

};

#endif // THREE_DIMENSIONAL_POINT_HANDLER_H
