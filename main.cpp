#include "ThreeDimensionalPointHandler.h"
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

#include <chrono>

using namespace std;
using namespace chrono;

void saveToCSV(const vector<pair<double, double>>& points, const vector<double>& extra_data, const string& filename) {
    ofstream file(filename);

    if (file.is_open()) {
        file << "x,y,z\n";
        for (size_t i = 0; i < points.size(); ++i) {
            file << points[i].first << "," << points[i].second << "," << extra_data[i] << "\n";
        }
        file.close();
    } else {
        cerr << "Unable to open file: " << filename << endl;
    }
}

void saveResultsToCSV(const vector<tuple<double, double, double>>& results, const string& filename) {
    ofstream file(filename);

    if (file.is_open()) {
        file << "x,y,fitted_value\n";
        for (const auto& result : results) {
            file << get<0>(result) << "," << get<1>(result) << "," << get<2>(result) << "\n";
        }
        file.close();
    } else {
        cerr << "Unable to open file: " << filename << endl;
    }
}

int main() {
    srand(static_cast<unsigned int>(time(nullptr)));

    vector<pair<double, double>> points;
    vector<double> extra_data;

    for (size_t i = 0; i < 60000; ++i) {
        // 60000点のポイントを生成
        // ここではターゲットの近傍点が50個程度になるように密度を調整する
        double x = static_cast<double>(rand() % 1000) / 10.0; // 0から100までの範囲にスケール
        double y = static_cast<double>(rand() % 1000) / 10.0; // 0から100までの範囲にスケール
        double z = x * x + y * y; // 二次曲面の方程式 z = x^2 + y^2
        points.emplace_back(x, y);
        extra_data.push_back(z);
    }

    // CSVファイルに保存
    saveToCSV(points, extra_data, "points.csv");

    auto handler = ThreeDimensionalPointHandler::create(points);

    int count = 2048*300;
    double radius = 1.5; // 近傍点が50個程度になるような半径を設定
    vector<tuple<double, double, double>> results;
    results.reserve(count);

    // 処理時間の測定開始
    auto start = high_resolution_clock::now();

    for (size_t i = 0; i < count; ++i) {
        double x = static_cast<double>(rand() % 1000) / 10.0;
        double y = static_cast<double>(rand() % 1000) / 10.0;
        double result = handler->performSearchAndFit(x, y, radius, extra_data);
        results.emplace_back(x, y, result);
    }

    // 処理時間の測定終了
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end - start);

    cout << "Processing time: " << duration.count() << " ms" << endl;
    saveResultsToCSV(results, "results.csv");

    return 0;
}
