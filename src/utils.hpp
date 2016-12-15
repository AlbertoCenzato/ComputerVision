#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

void print(Point2f& point) {
    cout << "(" << point.x << "," << point.y << ")";
    return;
}

void print(Point3f& point) {
    cout << "(" << point.x << "," << point.y << "," << point.z << ")";
    return;
}

void print(vector<Point3f>& vec) {
    for(int i = 0; i < vec.size(); ++i) {
        print(vec[i]);
        cout << " , ";
    }
    cout << endl;

    return;
}

void print(vector<Point2f>& vec) {
    for(int i = 0; i < vec.size(); ++i) {
        print(vec[i]);
        cout << " , ";
    }
    cout << endl;

    return;
}

void print(vector<vector<Point3f> >& vec) {
    for(int i = 0; i < vec.size(); ++i) {
        print(vec[i]);
        cout << " , ";
    }
    cout << endl;

    return;
}

void print(vector<vector<Point2f> >& vec) {
    for(int i = 0; i < vec.size(); ++i) {
        print(vec[i]);
        cout << " , ";
    }
    cout << endl;

    return;
}

#endif // UTILS_HPP
