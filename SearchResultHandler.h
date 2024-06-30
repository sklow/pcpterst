#ifndef SEARCH_RESULT_HANDLER_H
#define SEARCH_RESULT_HANDLER_H

#include <vector>
#include <memory>
#include "NearestNeighborSearch.h"

using namespace std;

class SearchResultHandler {
public:
    static shared_ptr<SearchResultHandler> create(const vector<pair<double, double>>& points) {
        auto searcher = NearestNeighborSearch::create(points);
        auto handler = make_shared<SearchResultHandler>(searcher);
        return handler;
    }

    SearchResultHandler(shared_ptr<NearestNeighborSearch> nnSearch) : nnSearch(nnSearch) {}

    vector<pair<size_t, vector<double>>> performSearch(double x, double y, double radius) {
        return nnSearch->findNeighbors(x, y, radius);
    }

private:
    shared_ptr<NearestNeighborSearch> nnSearch;
};

#endif // SEARCH_RESULT_HANDLER_H
