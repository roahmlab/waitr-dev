#include "FastCollisionChecking.h"
#include "BufferPath.h"

const std::string inputfilename1 = pathname + "obstacles.csv";
const std::string inputfilename2 = pathname + "joint_positions_uniform.csv"; // "joint_positions_uniform.csv"; // NOTE: comes from precompute joints positions matlab script
const std::string inputfilename3 = pathname + "adj_matrix_uniform_mult5.csv"; // adj_matrix_uniform_mult5.csv
const std::string outputfilename1 = pathname + "node_feasibility.csv";
const std::string outputfilename2 = pathname + "link_c.csv";
const std::string outputfilename3 = pathname + "collision_free_adj_matrix.csv";

#define NUM_EDGES 112826403
#define COLLISION_THRESHOLD -0.075
#define EDGE_THRESHOLD 0.1154*4.0

int main() {
/*
Section I:
    Parse input
    There is no check and warning, so be careful!
*/

    // const int num_obstacles = 10;
    // const double obstacles[num_obstacles * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3] = {-0.28239,  -0.33281, 0.88069, 0.069825, 0, 0, 0,  0.09508, 0, 0, 0, 0.016624,
    //                                                                             -0.19033,  0.035391,  1.3032,  0.11024, 0, 0, 0, 0.025188, 0, 0, 0, 0.014342,
    //                                                                             0.67593, -0.085841, 0.43572,  0.17408, 0, 0, 0,  0.07951, 0, 0, 0,  0.18012,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             -0.28239,  -0.33281, 0.88069, 0.069825, 0, 0, 0,  0.09508, 0, 0, 0, 0.016624,
    //                                                                             -0.19033,  0.035391,  1.3032,  0.11024, 0, 0, 0, 0.025188, 0, 0, 0, 0.014342,
    //                                                                             0.67593, -0.085841, 0.43572,  0.17408, 0, 0, 0,  0.07951, 0, 0, 0,  0.18012,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981};

    SimplifiedObstacles O;
    const int num_obstacles = 10;
    double obstacles[MAX_OBSTACLE_NUM * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3] = {0.0};

    std::ifstream inputstream1(inputfilename1);
  
    // inputstream1 >> num_obstacles;
    if (num_obstacles > MAX_OBSTACLE_NUM || num_obstacles < 0) {
        throw;
    }
    if (num_obstacles > 0) {
        for (int i = 0; i < num_obstacles * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3; i++) {
            inputstream1 >> obstacles[i];
        }
    }

    inputstream1.close();

    O.initialize(obstacles, num_obstacles); 

    Eigen::Vector3d link_sliced_center[NUM_NODES_AT_ONE_TIME * NUM_JOINTS];
    Eigen::Matrix<double, 3, LINK_FRS_GENERATOR_NUM> link_independent_generators[NUM_NODES_AT_ONE_TIME * NUM_JOINTS];
    double* link_c = new double[NUM_JOINTS * NUM_NODES_AT_ONE_TIME * num_obstacles];
    bool* node_feasibilities = new bool[NUM_NODES];
    std::ifstream inputstream2(inputfilename2);
    // std::ofstream outputstream1(outputfilename1);
    // std::ofstream outputstream2(outputfilename2);

    auto start1 = std::chrono::high_resolution_clock::now();

    // reading joint positions
    for (int k = 0; k < NUM_NODES / NUM_NODES_AT_ONE_TIME; k++) {
        for (int i = 0; i < NUM_NODES_AT_ONE_TIME; i++) {
            Eigen::Vector3d pos1;
            inputstream2 >> pos1(0) >> pos1(1) >> pos1(2);
            Eigen::Vector3d pos2;
            for (int j = 0; j < NUM_JOINTS; j++) {
                inputstream2 >> pos2(0) >> pos2(1) >> pos2(2);

                link_sliced_center[i * NUM_JOINTS + j] = 0.5 * (pos1 + pos2);
                link_independent_generators[i * NUM_JOINTS + j] = 0.5 * (pos1 - pos2);

                pos1 = pos2;
            }
        }

        /*
        Section II: Buffer obstacles and initialize collision checking hyperplanes
        */
        try {
            O.initializeHyperPlane(link_independent_generators);
        }
        catch (int errorCode) {
            WARNING_PRINT("        CUDA Collision checker: Error initializing collision checking hyperplanes! Check previous error message!");
            return -1;
        }

        /*
        Section III:
            Collision checking
        */

        try {
            O.linkFRSConstraints(link_sliced_center, link_c);
        }
        catch (int errorCode) {
            WARNING_PRINT("        CUDA Collision checker: Error peforming collision checking! Check previous error message!");
            return -1;
        }
        
        /*
        Section IV:
            Prepare output
        */
        // for (int i = 0; i < NUM_NODES_AT_ONE_TIME * num_obstacles; i++) {
        //     for (int j = 0; j < NUM_JOINTS; j++) {
        //         outputstream2 << link_c[j * NUM_NODES_AT_ONE_TIME * num_obstacles + i] << ' ';
        //     }
        //     outputstream2 << '\n';
        // }
        for (int i = 0; i < NUM_NODES_AT_ONE_TIME; i++) {
            bool node_feasibility = true;
            for (int j = 0; j < NUM_JOINTS; j++) {
                for (int h = 0; h < num_obstacles; h++) {
                    if (link_c[(j * NUM_NODES_AT_ONE_TIME + i) * num_obstacles + h] > COLLISION_THRESHOLD) {
                        node_feasibility = false;
                        break;
                    }
                }
            }
            // outputstream1 << node_feasibility << endl;
            node_feasibilities[k * NUM_NODES_AT_ONE_TIME + i] = node_feasibility;
        }
    }

    auto stop1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
    cout << "        CUDA Collision checker: Time taken by peforming collision checking: " << duration1.count() << " milliseconds" << endl;

    auto start2 = std::chrono::high_resolution_clock::now();

    // building adjacency matrix
    std::ifstream inputstream3(inputfilename3);
    std::ofstream outputstream3(outputfilename3);

    for (int i = 0; i < NUM_EDGES; i++) {
        int node_a = 0, node_b = 0;
        double edge_distance = 0;
        inputstream3 >> node_a >> node_b >> edge_distance;

        if (node_feasibilities[node_a] && node_feasibilities[node_b] && edge_distance < EDGE_THRESHOLD) {
            outputstream3 << node_a << ' ' << node_b << ' ' << edge_distance << '\n';
        }
    }

    auto stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - start2);
    cout << "        CUDA Collision checker: Time taken by exporting all edges: " << duration2.count() << " milliseconds" << endl;

    inputstream2.close();
    inputstream3.close();
    // outputstream1.close();
    // outputstream2.close();
    outputstream3.close();
    delete[] link_c;
    delete[] node_feasibilities;
    
    return 0;
}