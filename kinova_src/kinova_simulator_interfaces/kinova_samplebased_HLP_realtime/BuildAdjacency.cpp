#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <omp.h>
#include <vector>
#include <chrono>

using namespace std;
using namespace std::chrono;

#define NUMBER_OF_NODES 2140000
#define EDGE_THRESHOLD 0.1154*6 // This is norm(pi / 72 * ones(7,1))

Eigen::VectorXd wrapToPi(const Eigen::VectorXd& angles) {
    Eigen::VectorXd wrapped_angles = angles;
    for (int i = 0; i < 7; i += 2) {
        double angle = angles(i);
        while (angle < -M_PI) {
            angle += 2*M_PI;
        }
        while (angle > M_PI) {
            angle -= 2*M_PI;
        }
        wrapped_angles(i) = angle;
    }
    return wrapped_angles;
}

int main() {
    ifstream input_file("uniformNodes.csv");
    std::vector<Eigen::VectorXd> nodes;
    nodes.reserve(NUMBER_OF_NODES);
    if (input_file.is_open()) {
        for (int i = 0; i < NUMBER_OF_NODES; i++) {
            Eigen::VectorXd vec(7);
            for (int j = 0; j < 7; j++) {
                input_file >> vec(j);
            }
            nodes[i] = vec;
        }
    }
    input_file.close();

    auto start_time = high_resolution_clock::now();

    ofstream output_file("adj_matrix_uniform_mult5.csv"); 

    #pragma omp parallel for
    for (int i = 1; i < NUMBER_OF_NODES; i++) {
        for (int j = 0; j < i; j++) {
            Eigen::VectorXd diffVec = wrapToPi(nodes[i] - nodes[j]);
            double edgeLength = diffVec.norm();
            if (edgeLength < EDGE_THRESHOLD) {
                #pragma omp critical 
                {
                    output_file << i << " " << j << " " << edgeLength << "\n";
                }
            }
        }
    }
    
    output_file.close();

    auto stop_time = high_resolution_clock::now();
    auto elapsed_time = duration_cast<milliseconds>(stop_time - start_time);

    cout << "Elapsed time: " << elapsed_time.count() << " ms" << endl;

    return 0;
}