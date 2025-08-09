// Copyright 2025 Soheil E.nia


/*
    Since The robot move from child node to parent node I use steer the same way! steer(child,parent) but the time_parent > time_child
    also for combining the trajectoris in backward case i had to cobine them from the end of the execution trajectory till the start!
*/



// ////////////////////////////////////////VALID WITH RRTX data///////////////////////
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath> // For M_PI
// #include <tuple> // For std::get (if needed for fineGrain output)

// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"

// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
//     return full_state.head(D_spatial_dim);
// }


// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("thruster_steer_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     std::cout << "--- ThrusterSteerStateSpace Test ---\n";

//     // --- Configuration for 5D (2D position/velocity/time) ---
//     // Uncomment this block to test 5D
//     const int DIM = 5; // [x, y, vx, vy, t]
//     const double MAX_ACCEL = 5.0; // m/s^2 per dimension
//     Eigen::VectorXd robot_state(DIM);
//     robot_state << .8650457720455063, .6127650224165739,    // x, y
//                    .14352950978627876, .9942593962683071,    // vx, vy (start from rest)
//                    0.0;        // t_start
//     Eigen::VectorXd goal_state(DIM);
//     goal_state   << .522633156914757, .03581210314633027,   // x, y
//                    .012700721672822324, .413182577890049,   // vx, vy (end with some velocity)
//                    10.0;        // t_end (target arrival time)
//     int D_SPATIAL_DIM = 2;

//     int D_spatial = (DIM - 1) / 2; // e.g., 3 for 3D position/velocity


//     Eigen::VectorXd robot_pos = robot_state.head(D_spatial);
//     Eigen::VectorXd robot_vel = robot_state.segment(D_spatial, D_spatial);
//     double robot_time = robot_state[DIM - 1];
//     std::cout<<robot_pos<<" "<< robot_vel<<" "<< robot_time<<"\n";
//     Eigen::VectorXd goal_pos = goal_state.head(D_spatial);
//     Eigen::VectorXd goal_vel = goal_state.segment(D_spatial, D_spatial);
//     double goal_time = goal_state[DIM - 1];
//     std::cout<<goal_pos<<" "<<goal_vel<<" "<<goal_time<<"\n";


//     Eigen::VectorXd A(D_spatial);
//     A << .7978204924930904,.5206223536819761;
//     std::shared_ptr<ThrusterSteerStateSpace> thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);

//     ThrusterSteerStateSpace::NDSteeringResult nd_result = thruster_ss->steeringND(robot_pos, goal_pos, robot_vel, goal_vel, robot_time, goal_time, A);
    
//     std::cout<<"------------X------------- \n";
//     std::cout<<nd_result.X<<"\n";
//     std::cout<<"------------V------------- \n";
//     std::cout<<nd_result.V<<"\n";
//     std::cout<<"------------A------------- \n";
//     std::cout<<nd_result.A<<"\n";
//     std::cout<<"------------T------------- \n";
//     std::cout<<nd_result.Time<<"\n";
    

//     double discretization_step = 0.01; // meters (or unit of position distance for spatial dimensions)
//     auto [fine_Time, fine_A, fine_V, fine_X] = thruster_ss->fineGrain(nd_result.Time, nd_result.A, nd_result.V, nd_result.X, discretization_step);
//     std::cout<<"------------X------------- \n";
//     std::cout<<fine_X<<"\n";
//     std::cout<<"------------V------------- \n";
//     std::cout<<fine_V<<"\n";
//     std::cout<<"------------A------------- \n";
//     std::cout<<fine_A<<"\n";
//     std::cout<<"------------T------------- \n";
//     std::cout<<fine_Time<<"\n";
//     ////////////////////////////////////////solving////////////////////////////////////
//     std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%% \n";
//     std::cout << "Attempting to steer from: "; printVec(robot_state); std::cout << "\n";
//     std::cout << "To: "; printVec(goal_state); std::cout << "\n";

//     auto start_time_steer = std::chrono::high_resolution_clock::now();
//     Trajectory traj = thruster_ss->steer(robot_state,goal_state);
//     auto end_time_steer = std::chrono::high_resolution_clock::now();
//     auto duration_steer = std::chrono::duration_cast<std::chrono::microseconds>(end_time_steer - start_time_steer);
//     std::cout << "Time taken for the steer function: " << duration_steer.count() << " microseconds\n";


//     if (traj.is_valid) {
//         std::cout<<traj.execution_data.X<<" \n ----- \n  "<<traj.execution_data.Time<<"\n";
//         std::cout << "SUCCESS: Found a valid trajectory with cost (time elapsed): " << traj.cost << "s\n";
        
//         // Convert the list of waypoints into a list of line segments for visualization
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> path_segments;
//         if (traj.path_points.size() > 1) {
//             for (size_t i = 0; i < traj.path_points.size() - 1; ++i) {
//                 // Extract only the spatial (x,y,z) part for visualization
//                 path_segments.emplace_back(getSpatialPosition(traj.path_points[i], D_SPATIAL_DIM),
//                                            getSpatialPosition(traj.path_points[i+1], D_SPATIAL_DIM));
//             }
//         }
        
//         std::cout << "Visualizing trajectory with " << path_segments.size() << " segments in RViz...\n";
        
//         rclcpp::WallRate loop_rate(1); // Publish once per second
//         while(rclcpp::ok()) {
//             visualizer->visualizeEdges(path_segments, "map", "0.0,1.0,0.0", "thruster_path"); // Green line
//             rclcpp::spin_some(node);
//             loop_rate.sleep();
//         }

//     } else {
//         std::cout << "FAILURE: Could not find a valid trajectory between the states.\n";
//     }

//     rclcpp::shutdown();
//     return 0;
// }






// //////////////////////////////////////simple 2 state for 5D and 7D///////////////////////
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath> // For M_PI
// #include <tuple> // For std::get (if needed for fineGrain output)

// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"

// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
//     return full_state.head(D_spatial_dim);
// }


// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("thruster_steer_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     std::cout << "--- ThrusterSteerStateSpace Test ---\n";

//     // Uncomment this block to test 5D
//     const int DIM = 5; // [x, y, vx, vy, t]
//     const double MAX_ACCEL = 5.0; // m/s^2 per dimension
//     Eigen::VectorXd start_state(DIM);
//     start_state << 0.0, 0.0,    // x, y
//                    0.0, 0.0,    // vx, vy (start from rest)
//                    5.0;        // t_start
//     Eigen::VectorXd end_state(DIM);
//     end_state   << 20.0, 10.0,   // x, y
//                    2.0, -1.0,   // vx, vy (end with some velocity)
//                    0.0;        // t_end (target arrival time)
//     int D_SPATIAL_DIM = 2;


//     // // --- Configuration for 7D (3D position/velocity/time) ---
//     // // Uncomment this block to test 7D
//     // const int DIM = 7; // [x, y, z, vx, vy, vz, t]
//     // const double MAX_ACCEL = 3.0; // m/s^2 per dimension
//     // Eigen::VectorXd start_state(DIM);
//     // start_state << 0.0, 0.0, 0.0,     // x, y, z
//     //                0.0, 0.0, 0.0,     // vx, vy, vz (start from rest)
//     //                10.0;               // t_start
//     // Eigen::VectorXd end_state(DIM);
//     // end_state   << 15.0, 10.0, 5.0,   // x, y, z
//     //                1.0, -1.0, 0.5,    // vx, vy, vz (end with some velocity)
//     //                0.0;              // t_end (target arrival time)
//     // int D_SPATIAL_DIM = 3;


//     std::shared_ptr<StateSpace> thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);

//     std::cout << "Attempting to steer from: "; printVec(end_state); std::cout << "\n";
//     std::cout << "To: "; printVec(start_state); std::cout << "\n";

//     auto start_time_steer = std::chrono::high_resolution_clock::now();
//     Trajectory traj = thruster_ss->steer(end_state, start_state);
//     auto end_time_steer = std::chrono::high_resolution_clock::now();
//     auto duration_steer = std::chrono::duration_cast<std::chrono::microseconds>(end_time_steer - start_time_steer);
//     std::cout << "Time taken for the steer function: " << duration_steer.count() << " microseconds\n";


//     if (traj.is_valid) {
//         std::cout<<traj.execution_data.X<<" \n ----- \n  "<<traj.execution_data.Time<<"\n";
//         std::cout << "SUCCESS: Found a valid trajectory with cost (time elapsed): " << traj.cost << "s\n";
        
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> path_segments;
//         if (traj.path_points.size() > 1) {
//             for (size_t i = 0; i < traj.path_points.size() - 1; ++i) {
//                 path_segments.emplace_back(getSpatialPosition(traj.path_points[i], D_SPATIAL_DIM),
//                                            getSpatialPosition(traj.path_points[i+1], D_SPATIAL_DIM));
//             }
//         }
        
//         std::cout << "Visualizing trajectory with " << path_segments.size() << " segments in RViz...\n";
        
//         rclcpp::WallRate loop_rate(1);
//         while(rclcpp::ok()) {
//             visualizer->visualizeEdges(path_segments, "map", "0.0,1.0,0.0", "thruster_path"); // Green line
//             rclcpp::spin_some(node);
//             loop_rate.sleep();
//         }

//     } else {
//         std::cout << "FAILURE: Could not find a valid trajectory between the states.\n";
//     }

//     rclcpp::shutdown();
//     return 0;
// }



// ////////////////////////////// 7D case with 4 points///////////////////////////////////////////

// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath>
// #include <tuple>
// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"


// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
//     return full_state.head(D_spatial_dim);
// }


// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("thruster_steer_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     std::cout << "--- Multi-Segment ThrusterSteer Test (A->B->C->D Backward Planning) ---\n";

//     const int DIM = 7; // [x, y, z, vx, vy, vz, t]
//     const double MAX_ACCEL = 3.0; // m/s^2 per dimension
//     int D_SPATIAL_DIM = 3;



//     Eigen::VectorXd state_A(DIM); // Robot's Initial Start, Planner's Goal
//     state_A << 0.0, 0.0, 0.0,    // x, y, z
//                0.0, 0.0, 0.0,    // vx, vy, vz (start from rest)
//                0.0;              // t_start = 0

//     Eigen::VectorXd state_B(DIM); // Intermediate Waypoint
//     state_B << 15.0, 10.0, 5.0,  // x, y, z (move some distance)
//                1.0, 1.0, 0.5,    // vx, vy, vz (start moving slowly)
//                20.0;             // t_B = 20 (delta_t A->B = 20s)

//     Eigen::VectorXd state_C(DIM); // Intermediate Waypoint
//     state_C << 30.0, 25.0, 10.0, // x, y, z (further displace)
//                2.0, 2.0, 1.0,    // vx, vy, vz (speed up)
//                40.0;             // t_C = 40 (delta_t B->C = 20s)

//     Eigen::VectorXd state_D(DIM); // Ultimate Goal for Robot, Planner's Start/Root
//     state_D << 50.0, 35.0, 12.0, // x, y, z (final displacement)
//                0.0, 0.0, 0.0,    // vx, vy, vz (arrive at rest)
//                60.0;             // t_D = 60 (delta_t C->D = 20s)


//     std::shared_ptr<StateSpace> thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);

//     std::vector<Eigen::VectorXd> segments_to_plan;
//     segments_to_plan.push_back(state_D); 
//     segments_to_plan.push_back(state_C); 
//     segments_to_plan.push_back(state_B); 
//     segments_to_plan.push_back(state_A); 

//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> all_path_segments_viz; 
//     std::vector<Eigen::VectorXd> waypoint_nodes_viz;

//     double total_cost = 0.0;

//     waypoint_nodes_viz.push_back(getSpatialPosition(state_A, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(state_B, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(state_C, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(state_D, D_SPATIAL_DIM));


//     for (size_t i = 0; i < segments_to_plan.size() - 1; ++i) {
//         Eigen::VectorXd from_state = segments_to_plan[i+1];    
//         Eigen::VectorXd to_state = segments_to_plan[i];    

//         std::cout << "\n--- Planning segment " << i+1 << " --- (from time " << from_state[DIM-1] << " to time " << to_state[DIM-1] << ")\n";
//         std::cout << "  From: "; printVec(from_state); std::cout << "\n";
//         std::cout << "  To:   "; printVec(to_state);   std::cout << "\n";

//         auto start_time_steer = std::chrono::high_resolution_clock::now();
//         Trajectory traj = thruster_ss->steer(from_state, to_state); 
//         auto end_time_steer = std::chrono::high_resolution_clock::now();
//         auto duration_steer = std::chrono::duration_cast<std::chrono::microseconds>(end_time_steer - start_time_steer);
//         std::cout << "  Time taken for steer: " << duration_steer.count() << " microseconds\n";

//         if (traj.is_valid) {
//             std::cout << "  SUCCESS: Segment cost (time elapsed): " << traj.cost << "s\n";
//             if (!traj.path_points.empty()) {
//                 std::cout << "    DEBUG TRAJ: First point:  "; printVec(traj.path_points[0]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Second point: "; printVec(traj.path_points[1]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Last point:   "; printVec(traj.path_points.back()); std::cout << "\n";
//             } else {
//                 std::cout << "    DEBUG TRAJ: Trajectory is empty!\n";
//             }
//             total_cost += traj.cost;
//             if (traj.path_points.size() > 1) {
//                 for (size_t j = 0; j < traj.path_points.size() - 1; ++j) {
//                     all_path_segments_viz.emplace_back(getSpatialPosition(traj.path_points[j], D_SPATIAL_DIM),
//                                                        getSpatialPosition(traj.path_points[j+1], D_SPATIAL_DIM));
//                 }
//             }
//         } else {
//             std::cout << "  FAILURE: Could not find a valid trajectory for this segment.\n";
//             std::cout << "  This segment's path will not be visualized.\n";
//         }
//     }

//     if (!all_path_segments_viz.empty()) {
//         std::cout << "Total planned path segments for visualization: " << all_path_segments_viz.size() << "\n";
//         std::cout << "Total accumulated cost (time elapsed): " << total_cost << "s\n";
//         std::cout << "Visualizing combined trajectory in RViz...\n";
        
//         rclcpp::WallRate loop_rate(1); 
//         while(rclcpp::ok()) {
//             visualizer->visualizeEdges(all_path_segments_viz, "map", "0.0,1.0,0.0", "full_thruster_path"); 
//             visualizer->visualizeNodes(waypoint_nodes_viz, "map", {1.0f, 0.0f, 0.0f}, "waypoint_nodes"); // Visualize waypoints as red nodes
//             rclcpp::spin_some(node);
//             loop_rate.sleep();
//         }
//     } else {
//         std::cout << "No valid segments were found for visualization.\n";
//     }

//     rclcpp::shutdown();
//     return 0;
// }





// /////////////////////////////////// 5D case with 10 points////////////////////////////////
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath>
// #include <tuple>

// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"
// #include "motion_planning/utils/ros2_manager_thruster.hpp"
// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
//     return full_state.head(D_spatial_dim);
// }


// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("thruster_steer_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);
    
//     std::cout << "--- 5D Multi-Segment ThrusterSteer Test (10 Points) ---\n";

//     const int DIM = 5; // [x, y, vx, vy, t]
//     const double MAX_ACCEL = 2.0; // m/s^2 per dimension (Adjusted for 10-point path)
//     int D_SPATIAL_DIM = 2; // For [x, y]



//     Eigen::VectorXd P0(DIM); // Start
//     P0 << 0.0, 0.0, 0.0, 0.0, 0.0; // (0,0), rest, t=0

//     Eigen::VectorXd P1(DIM); 
//     P1 << 5.0, 2.0, 0.5, 0.2, 10.0; // Moving, t=10

//     Eigen::VectorXd P2(DIM);
//     P2 << 12.0, 8.0, 1.0, 0.5, 20.0; // Speeding up, t=20

//     Eigen::VectorXd P3(DIM);
//     P3 << 20.0, 15.0, 1.5, 1.0, 30.0; // Accelerating, t=30

//     Eigen::VectorXd P4(DIM);
//     P4 << 30.0, 22.0, 1.0, 1.5, 40.0; // Changing direction slightly, t=40

//     Eigen::VectorXd P5(DIM);
//     P5 << 42.0, 28.0, 0.5, 2.0, 50.0; // Decelerating in X, accelerating in Y, t=50

//     Eigen::VectorXd P6(DIM);
//     P6 << 50.0, 35.0, 0.0, 1.0, 60.0; // Stopping in X, slow Y, t=60

//     Eigen::VectorXd P7(DIM);
//     P7 << 55.0, 42.0, 1.0, 0.5, 70.0; // Accelerating again, t=70

//     Eigen::VectorXd P8(DIM);
//     P8 << 60.0, 48.0, 0.5, -0.5, 80.0; // Changing Y velocity to negative, t=80

//     Eigen::VectorXd P9(DIM); // End
//     P9 << 65.0, 50.0, 0.0, 0.0, 90.0; // Final position, rest, t=90


//     std::shared_ptr<StateSpace> thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);

//     std::vector<Eigen::VectorXd> segments_to_plan;
//     segments_to_plan.push_back(P9); 
//     segments_to_plan.push_back(P8); 
//     segments_to_plan.push_back(P7); 
//     segments_to_plan.push_back(P6); 
//     segments_to_plan.push_back(P5); 
//     segments_to_plan.push_back(P4); 
//     segments_to_plan.push_back(P3); 
//     segments_to_plan.push_back(P2); 
//     segments_to_plan.push_back(P1); 
//     segments_to_plan.push_back(P0); 

//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> all_path_segments_viz; 
//     std::vector<Eigen::VectorXd> waypoint_nodes_viz;

//     double total_cost = 0.0;

//     waypoint_nodes_viz.push_back(getSpatialPosition(P0, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P1, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P2, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P3, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P4, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P5, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P6, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P7, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P8, D_SPATIAL_DIM));
//     waypoint_nodes_viz.push_back(getSpatialPosition(P9, D_SPATIAL_DIM));


//     for (size_t i = 0; i < segments_to_plan.size() - 1; ++i) {
//         Eigen::VectorXd from_state = segments_to_plan[i+1];     // child node
//         Eigen::VectorXd to_state = segments_to_plan[i];     // parent node 

//         std::cout << "\n--- Planning segment " << i+1 << " --- (from time " << from_state[DIM-1] << " to time " << to_state[DIM-1] << ")\n";
//         std::cout << "  From: "; printVec(from_state); std::cout << "\n";
//         std::cout << "  To:   "; printVec(to_state);   std::cout << "\n";

//         auto start_time_steer = std::chrono::high_resolution_clock::now();
//         Trajectory traj = thruster_ss->steer(from_state, to_state); 
//         auto end_time_steer = std::chrono::high_resolution_clock::now();
//         auto duration_steer = std::chrono::duration_cast<std::chrono::microseconds>(end_time_steer - start_time_steer);
//         std::cout << "  Time taken for steer: " << duration_steer.count() << " microseconds\n";

//         if (traj.is_valid) {
//             std::cout << "  SUCCESS: Segment cost (time elapsed): " << traj.cost << "s\n";
//             if (!traj.path_points.empty()) {
//                 std::cout << "    DEBUG TRAJ: First point:  "; printVec(traj.path_points[0]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Second point: "; printVec(traj.path_points[1]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Last point:   "; printVec(traj.path_points.back()); std::cout << "\n";
//             } else {
//                 std::cout << "    DEBUG TRAJ: Trajectory is empty!\n";
//             }
//             total_cost += traj.cost;
//             if (traj.path_points.size() > 1) {
//                 for (size_t j = 0; j < traj.path_points.size() - 1; ++j) {
//                     all_path_segments_viz.emplace_back(getSpatialPosition(traj.path_points[j], D_SPATIAL_DIM),
//                                                        getSpatialPosition(traj.path_points[j+1], D_SPATIAL_DIM));
//                 }
//             }
//         } else {
//             std::cout << "  FAILURE: Could not find a valid trajectory for this segment.\n";
//             std::cout << "  This segment's path will not be visualized.\n";
//         }
//     }

//     if (!all_path_segments_viz.empty()) {
//         std::cout << "Total planned path segments for visualization: " << all_path_segments_viz.size() << "\n";
//         std::cout << "Total accumulated cost (time elapsed): " << total_cost << "s\n";
//         std::cout << "Visualizing combined trajectory in RViz...\n";
        
//         rclcpp::WallRate loop_rate(1); 
//         while(rclcpp::ok()) {
//             visualizer->visualizeEdges(all_path_segments_viz, "map", "0.0,1.0,0.0", "full_thruster_path"); 
//             visualizer->visualizeNodes(waypoint_nodes_viz, "map", {1.0f, 0.0f, 0.0f}, "waypoint_nodes"); 
//             rclcpp::spin_some(node);
//             loop_rate.sleep();
//         }
//     } else {
//         std::cout << "No valid segments were found for visualization.\n";
//     }

//     rclcpp::shutdown();
//     return 0;
// }



// ///////////////////////////////////////////////////ONLY TESTING 1D Steering to validify!/////////////////////////////////////////////////////////////////////

// #include <iostream>
// #include <vector>
// #include <string>
// #include <cmath>
// #include <iomanip>
// #include <memory>
// #include <limits>
// #include <Eigen/Dense>

// #include "motion_planning/state_space/thruster_statespace.hpp"

// struct TestCase1D {
//     std::string name;
//     double x_start, x_end;
//     double v_start, v_end;
//     double t_start, t_end;
//     double a_max;
//     bool expect_success;
// };

// bool verify_trajectory(const TestCase1D& test, const Eigen::VectorXd& result) {
//     std::cout << "  [Verification] Running physics simulation with results...\n";
//     const double epsilon = 1e-4;

//     double t1 = result[0];
//     double t2 = result[1];
//     double a1 = result[2];
//     double a2 = result[3];

//     double dt1 = t1 - test.t_start;
//     double dt2 = t2 - t1;
//     double dt3 = test.t_end - t2;

//     if (dt1 < -epsilon || dt2 < -epsilon || dt3 < -epsilon) {
//         std::cout << "  [Verification FAIL] Negative time duration calculated.\n";
//         return false;
//     }

//     double x_at_t1 = test.x_start + test.v_start * dt1 + 0.5 * a1 * dt1 * dt1;
//     double v_at_t1 = test.v_start + a1 * dt1;
//     std::cout << "    - After phase 1 (t=" << t1 << "): x=" << x_at_t1 << ", v=" << v_at_t1 << "\n";

//     double x_at_t2 = x_at_t1 + v_at_t1 * dt2;
//     double v_at_t2 = v_at_t1;
//     std::cout << "    - After phase 2 (t=" << t2 << "): x=" << x_at_t2 << ", v=" << v_at_t2 << "\n";

//     double x_final = x_at_t2 + v_at_t2 * dt3 + 0.5 * a2 * dt3 * dt3;
//     double v_final = v_at_t2 + a2 * dt3;
//     std::cout << "    - After phase 3 (t=" << test.t_end << "): x=" << x_final << ", v=" << v_final << "\n";

//     bool pos_ok = std::abs(x_final - test.x_end) < epsilon;
//     bool vel_ok = std::abs(v_final - test.v_end) < epsilon;

//     std::cout << "    - Expected final state:          x=" << test.x_end << ", v=" << test.v_end << "\n";

//     if (pos_ok && vel_ok) {
//         std::cout << "  [Verification PASS] Final state matches expected state.\n";
//         return true;
//     } else {
//         std::cout << "  [Verification FAIL] Final state mismatch!\n";
//         std::cout << "    - Position error: " << std::abs(x_final - test.x_end) << "\n";
//         std::cout << "    - Velocity error: " << std::abs(v_final - test.v_end) << "\n";
//         return false;
//     }
// }


// int main() {

//     auto thruster_ss = std::make_shared<ThrusterSteerStateSpace>(7, 5.0);

//     std::vector<TestCase1D> tests = {
//         {
//             "Simple Accel-Coast-Decel (Rest to Rest)",
//             0.0, 50.0,  // x_start, x_end
//             0.0, 0.0,   // v_start, v_end
//             0.0, 15.0,  // t_start, t_end
//             3.0,        // a_max
//             true
//         },
//         {
//             "Start from Rest, End with Velocity",
//             0.0, 80.0,
//             0.0, 5.0,
//             0.0, 12.0,
//             2.0,
//             true
//         },
//         {
//             "BUGGY CASE: Positive to Negative Velocity",
//             0.0, 5.0,   // x_start, x_end
//             2.0, -2.0,  // v_start, v_end (This triggers Case B)
//             0.0, 10.0,  // t_start, t_end
//             1.0,        // a_max
//             true
//         },
//         {
//             "Flipped Start: Negative to Positive Velocity",
//             10.0, 5.0,
//             -2.0, 1.0,  // v_start is negative, should trigger the flip_flag
//             0.0, 8.0,
//             1.5,
//             true
//         },
//         {
//             "Infeasible: Not Enough Time",
//             0.0, 100.0,
//             0.0, 0.0,
//             0.0, 5.0,   // Only 5 seconds to travel 100m
//             3.0,
//             false
//         },
//         {
//             "Infeasible: Distance Too Far",
//             0.0, 200.0,
//             0.0, 0.0,
//             0.0, 15.0,  // 15 seconds is enough time for a shorter distance
//             3.0,
//             false
//         }
//     };

//     int passed_count = 0;
//     for (const auto& test : tests) {
//         std::cout << "\n==========================================================\n";
//         std::cout << "--- Running Test: " << test.name << " ---\n";
//         std::cout << "  Inputs: x=[" << test.x_start << "->" << test.x_end << "], v=[" << test.v_start << "->" << test.v_end
//                   << "], t=[" << test.t_start << "->" << test.t_end << "], a_max=" << test.a_max << "\n";
//         std::cout << "----------------------------------------------------------\n";

//         Eigen::VectorXd result = thruster_ss->steering1D(
//             test.x_start, test.x_end, test.v_start, test.v_end, test.t_start, test.t_end, test.a_max
//         );

//         bool has_nan = result.hasNaN();
//         bool success = !has_nan;
//         bool overall_pass = true;

//         if (success) {
//             std::cout << "  [Result] Success! Trajectory found.\n";
//             std::cout << std::fixed << std::setprecision(4);
//             std::cout << "    - t1=" << result[0] << ", t2=" << result[1]
//                       << ", a1=" << result[2] << ", a2=" << result[3] << ", v_coast=" << result[4] << "\n";

//             if (!test.expect_success) {
//                 std::cout << "  [Status: FAIL] A trajectory was found, but none was expected.\n";
//                 overall_pass = false;
//             } else {
//                 // Verify the trajectory only if success was expected and achieved
//                 if (verify_trajectory(test, result)) {
//                     std::cout << "  [Status: PASS]\n";
//                 } else {
//                     std::cout << "  [Status: FAIL] Verification of physics failed.\n";
//                     overall_pass = false;
//                 }
//             }
//         } else { // Failure case
//             std::cout << "  [Result] Failure. No trajectory found (returned NaN).\n";
//             if (test.expect_success) {
//                 std::cout << "  [Status: FAIL] A trajectory was expected, but none was found.\n";
//                 overall_pass = false;
//             } else {
//                 std::cout << "  [Status: PASS] Failure was correctly expected.\n";
//             }
//         }

//         if (overall_pass) {
//             passed_count++;
//         }
//     }

//     std::cout << "Test suite finished. " << passed_count << " out of " << tests.size() << " tests passed.\n";

//     return 0;
// }


// // /////////////////////////////////////////////5D forward movement!/////////////////////////////////////////////////////////////////////////////
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath>
// #include <tuple>
// #include <algorithm>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/executors.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"
// #include "motion_planning/ds/edge_info.hpp"
// #include "motion_planning/utils/params.hpp"
// #include "motion_planning/utils/ros2_manager_thruster.hpp"



// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// ExecutionTrajectory combineExecutionTrajectories(const std::vector<ExecutionTrajectory>& segments) {
//     if (segments.empty()) {
//         return ExecutionTrajectory{};
//     }
//     int total_points = 0;
//     for (const auto& seg : segments) {
//         if (seg.Time.size() > 0) {
//             total_points += seg.Time.size() -1;
//         }
//     }
//     total_points +=1;
//     if (total_points == 0) return ExecutionTrajectory{};
    
//     int D_spatial = segments[0].X.cols();
//     ExecutionTrajectory final_traj;
//     final_traj.X = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.V = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.A = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.Time = Eigen::VectorXd(total_points);

//     int current_row = 0;
//     for (size_t i = 0; i < segments.size(); ++i) {
//         const auto& seg = segments[i];
//         if (seg.Time.size() == 0) continue;
//         int points_to_copy = (i == segments.size() - 1) ? seg.Time.size() : seg.Time.size() - 1;

//         if (points_to_copy > 0) {
//             final_traj.Time.segment(current_row, points_to_copy) = seg.Time.head(points_to_copy);
//             final_traj.X.block(current_row, 0, points_to_copy, D_spatial) = seg.X.topRows(points_to_copy);
//             final_traj.V.block(current_row, 0, points_to_copy, D_spatial) = seg.V.topRows(points_to_copy);
//             final_traj.A.block(current_row, 0, points_to_copy, D_spatial) = seg.A.topRows(points_to_copy);
//             current_row += points_to_copy;
//         }
//     }
//     final_traj.is_valid = true;
//     if (total_points > 0) {
//        final_traj.total_cost = final_traj.Time.tail(1)[0] - final_traj.Time.head(1)[0];
//     }
//     return final_traj;
// }


// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
    
//     std::cout << "--- 5D Multi-Segment ThrusterSteer Test with FORWARD Simulation ---\n";

//     const int DIM = 5;
//     const double MAX_ACCEL = 2.0;

//     // --- Define Waypoints in Forward Time Order ---
//     std::vector<Eigen::VectorXd> waypoints;
//     waypoints.push_back((Eigen::VectorXd(DIM) << 0.0, 0.0, 0.0, 0.0, 0.0).finished());    // P0
//     waypoints.push_back((Eigen::VectorXd(DIM) << 5.0, 2.0, 0.5, 0.2, 10.0).finished());   // P1
//     waypoints.push_back((Eigen::VectorXd(DIM) << 12.0, 8.0, 1.0, 0.5, 20.0).finished());  // P2
//     waypoints.push_back((Eigen::VectorXd(DIM) << 20.0, 15.0, 1.5, 1.0, 30.0).finished()); // P3
//     waypoints.push_back((Eigen::VectorXd(DIM) << 30.0, 22.0, 1.0, 1.5, 40.0).finished()); // P4
//     waypoints.push_back((Eigen::VectorXd(DIM) << 42.0, 28.0, 0.5, 2.0, 50.0).finished()); // P5
//     waypoints.push_back((Eigen::VectorXd(DIM) << 50.0, 35.0, 0.0, 1.0, 60.0).finished()); // P6
//     waypoints.push_back((Eigen::VectorXd(DIM) << 55.0, 42.0, 1.0, 0.5, 70.0).finished()); // P7
//     waypoints.push_back((Eigen::VectorXd(DIM) << 60.0, 48.0, 0.5, 0.5, 80.0).finished()); // P8
//     waypoints.push_back((Eigen::VectorXd(DIM) << 65.0, 50.0, 0.0, 0.0, 90.0).finished()); // P9

//     auto thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);
//     std::vector<ExecutionTrajectory> execution_segments;

//     for (size_t i = 0; i < waypoints.size() - 1; ++i) {
//         Eigen::VectorXd from_state = waypoints[i];
//         Eigen::VectorXd to_state = waypoints[i+1];

//         std::cout << "\n--- Planning segment " << i << " (" << to_state[DIM-1] << "s to " << from_state[DIM-1] << "s) ---\n";
//         Trajectory traj = thruster_ss->steer(from_state, to_state);

//         if (traj.is_valid) {
//             std::cout << "  SUCCESS: Segment cost: " << traj.cost << "s\n";
//             execution_segments.push_back(traj.execution_data);
//         } else {
//             RCLCPP_ERROR(rclcpp::get_logger("main"), "FAILURE: Could not find a trajectory for segment %zu.", i);
//         }
//     }
    
//     std::cout << "\n--- Combining trajectories for execution ---\n";
//     ExecutionTrajectory full_execution_path = combineExecutionTrajectories(execution_segments);

//     if (!full_execution_path.is_valid || full_execution_path.Time.size() == 0) {
//         RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create a valid combined execution path. Exiting.");
//         rclcpp::shutdown();
//         return -1;
//     }

//     auto driver_node = std::make_shared<rclcpp::Node>("thruster_test_driver");
//     auto visualizer = std::make_shared<RVizVisualization>(driver_node);

//     Params params;
//     params.setParam("thruster_state_dimension", DIM);
//     params.setParam("simulation_time_step", 0.05);
//     params.setParam("use_sim_time", false);

//     auto ros2_manager = std::make_shared<ROS2Manager>(visualizer, params);
//     ros2_manager->setInitialState(waypoints[0]); // START AT P0
//     ros2_manager->setPlannedThrusterTrajectory(full_execution_path);

//     visualizer->visualizeNodes(waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoint_nodes");

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(driver_node);
//     executor.add_node(ros2_manager);
    
//     std::cout << "Starting ROS2Manager for FORWARD robot simulation (P0->P9)...\n";
//     executor.spin();

//     rclcpp::shutdown();
//     return 0;
// }


// // /////////////////////////////////////////////5D backward movement with backward iterator (NOT IDEAL)! /////////////////////////////////////////////////////////////////////////////
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath>
// #include <tuple>
// #include <algorithm>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/executors.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"
// #include "motion_planning/ds/edge_info.hpp"
// #include "motion_planning/utils/params.hpp" 
// #include "motion_planning/utils/ros2_manager_thruster.hpp"



// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
//     return full_state.head(D_spatial_dim);
// }

// ExecutionTrajectory combineExecutionTrajectories(const std::vector<ExecutionTrajectory>& segments) {
//     if (segments.empty()) {
//         return ExecutionTrajectory{};
//     }
//     int total_points = 0;
//     for (const auto& seg : segments) {
//         if (seg.Time.size() > 0) {
//             total_points += seg.Time.size() -1;
//         }
//     }
//     total_points +=1;
//     if (total_points == 0) return ExecutionTrajectory{};
    
//     int D_spatial = segments[0].X.cols();
//     ExecutionTrajectory final_traj;
//     final_traj.X = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.V = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.A = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.Time = Eigen::VectorXd(total_points);

//     int current_row = 0;
//     for (size_t i = 0; i < segments.size(); ++i) {
//         const auto& seg = segments[i];
//         if (seg.Time.size() == 0) continue;
//         int points_to_copy = (i == segments.size() - 1) ? seg.Time.size() : seg.Time.size() - 1;

//         if (points_to_copy > 0) {
//             final_traj.Time.segment(current_row, points_to_copy) = seg.Time.head(points_to_copy);
//             final_traj.X.block(current_row, 0, points_to_copy, D_spatial) = seg.X.topRows(points_to_copy);
//             final_traj.V.block(current_row, 0, points_to_copy, D_spatial) = seg.V.topRows(points_to_copy);
//             final_traj.A.block(current_row, 0, points_to_copy, D_spatial) = seg.A.topRows(points_to_copy);
//             current_row += points_to_copy;
//         }
//     }
//     final_traj.is_valid = true;
//     if (total_points > 0) {
//        final_traj.total_cost = final_traj.Time.tail(1)[0] - final_traj.Time.head(1)[0];
//     }
//     return final_traj;
// }

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
    
//     std::cout << "--- 5D Thruster Test: Simulating CORRECT Backward-in-Time Path Execution (P9 -> P0) ---\n";

//     const int DIM = 5;
//     const double MAX_ACCEL = 2.0;
//     int D_SPATIAL_DIM = 2;

//     // // --- Define Original Waypoints (P0 to P9) ---
//     // // This represents the result of a planner growing a tree from a goal at t=0
//     // std::vector<Eigen::VectorXd> original_waypoints;
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 0.0, 0.0, 0.0, 0.0, 0.0).finished());    // P0
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 5.0, 2.0, 0.5, 0.2, 10.0).finished());   // P1
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 12.0, 8.0, 1.0, 0.5, 20.0).finished());  // P2
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 20.0, 15.0, 1.5, 1.0, 30.0).finished()); // P3
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 30.0, 22.0, 1.0, 1.5, 40.0).finished()); // P4
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 42.0, 28.0, 0.5, 2.0, 50.0).finished()); // P5
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 50.0, 35.0, 0.0, 1.0, 60.0).finished()); // P6
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 55.0, 42.0, 1.0, 0.5, 70.0).finished()); // P7
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 60.0, 48.0, 0.5, 0.5, 80.0).finished()); // P8
//     // original_waypoints.push_back((Eigen::VectorXd(DIM) << 65.0, 50.0, 0.0, 0.0, 90.0).finished()); // P9

//     std::vector<Eigen::VectorXd> original_waypoints;
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 0.0, 0.0, 0.0, 0.0, 0.0).finished());    // P0
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 5.0, 2.0, -0.5, -0.2, 10.0).finished());   // P1
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 12.0, 8.0, -1.0, -0.5, 20.0).finished());  // P2
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 20.0, 15.0, -1.5, -1.0, 30.0).finished()); // P3
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 30.0, 22.0, -1.0, -1.5, 40.0).finished()); // P4
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 42.0, 28.0, -0.5, -2.0, 50.0).finished()); // P5
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 50.0, 35.0, -0.0, -1.0, 60.0).finished()); // P6
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 55.0, 42.0, -1.0, -0.5, 70.0).finished()); // P7
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 60.0, 48.0, -0.5, -0.5, 80.0).finished()); // P8
//     original_waypoints.push_back((Eigen::VectorXd(DIM) << 65.0, 50.0, 0.0, 0.0, 90.0).finished()); // P9
//     auto thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);
//     std::vector<ExecutionTrajectory> execution_segments;
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> all_path_segments_viz; 
//     double total_cost = 0.0;
//     for (int i = original_waypoints.size() - 1; i > 0; --i) {
//         Eigen::VectorXd start_node = original_waypoints[i];
//         Eigen::VectorXd end_node = original_waypoints[i-1];

//         double segment_duration = start_node[DIM-1] - end_node[DIM-1];
        
//         Eigen::VectorXd to_state = start_node;
//         to_state[DIM-1] = 0.0;

//         Eigen::VectorXd from_state = end_node;
//         from_state[DIM-1] = segment_duration;
        
//         std::cout << "\n--- Planning execution segment from P" << i << " to P" << i-1 << " ---\n";
//         Trajectory traj = thruster_ss->steer(from_state, to_state);


//         if (traj.is_valid) {
//             std::cout << "  SUCCESS: Found " << segment_duration << "s trajectory.\n";
//             execution_segments.push_back(traj.execution_data);
//             if (!traj.path_points.empty()) {
//                 std::cout << "    DEBUG TRAJ: First point:  "; printVec(traj.path_points[0]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Second point: "; printVec(traj.path_points[1]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Last point:   "; printVec(traj.path_points.back()); std::cout << "\n";
//             } else {
//                 std::cout << "    DEBUG TRAJ: Trajectory is empty!\n";
//             }
//             total_cost += traj.cost;
//             if (traj.path_points.size() > 1) {
//                 for (size_t j = 0; j < traj.path_points.size() - 1; ++j) {
//                     all_path_segments_viz.emplace_back(getSpatialPosition(traj.path_points[j], D_SPATIAL_DIM),
//                                                        getSpatialPosition(traj.path_points[j+1], D_SPATIAL_DIM));
//                 }
//             }
//         } else {
//             RCLCPP_ERROR(rclcpp::get_logger("main"), "FAILURE: Could not find trajectory for segment P%d -> P%d.", i, i-1);
//         }
//     }
    
//     std::cout << "\n--- Stitching trajectories with a continuous timeline (t=0 at P9) ---\n";
    
//     double time_offset = 0.0;
//     for (auto& seg : execution_segments) {
//         if (seg.Time.size() > 0) {
//             seg.Time.array() += time_offset;
//             time_offset = seg.Time.tail(1)[0];
//         }
//     }

//     ExecutionTrajectory full_execution_path = combineExecutionTrajectories(execution_segments);

//     if (!full_execution_path.is_valid || full_execution_path.Time.size() == 0) {
//         RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create a valid combined execution path. Exiting.");
//         rclcpp::shutdown();
//         return -1;
//     }

//     auto driver_node = std::make_shared<rclcpp::Node>("thruster_test_driver");
//     auto visualizer = std::make_shared<RVizVisualization>(driver_node);

//     Params params;
//     params.setParam("thruster_state_dimension", DIM);
//     params.setParam("simulation_time_step", 0.05);
//     params.setParam("use_sim_time", false);

//     auto ros2_manager = std::make_shared<ROS2Manager>(visualizer, params);
    
//     Eigen::VectorXd initial_robot_state = original_waypoints.back();
//     initial_robot_state[DIM-1] = 0.0;
//     ros2_manager->setInitialState(initial_robot_state); 
//     ros2_manager->setPlannedThrusterTrajectory(full_execution_path);



//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(driver_node);
//     executor.add_node(ros2_manager);
//     // executor.spin();

//     rclcpp::WallRate loop_rate(60); // Publish once per second
//     while(rclcpp::ok()) {
//         visualizer->visualizeEdges(all_path_segments_viz, "map", "0.0,1.0,0.0", "thruster_path"); // Green line
//         visualizer->visualizeNodes(original_waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoint_nodes");
//         executor.spin_some();
//         loop_rate.sleep();
//     }

//     rclcpp::shutdown();
//     return 0;
// }

// //////////////////////////////////////////////////////5D 3 states  backward///////////////////////////////////////////////////////////



// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath> // For M_PI
// #include <tuple> // For std::get (if needed for fineGrain output)
// #include "motion_planning/utils/params.hpp" // Using the provided Params header
// #include "motion_planning/utils/ros2_manager_thruster.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"

// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
//     return full_state.head(D_spatial_dim);
// }

// ExecutionTrajectory combineExecutionTrajectories(const std::vector<ExecutionTrajectory>& segments) {
//     if (segments.empty()) {
//         return ExecutionTrajectory{};
//     }
//     int total_points = 0;
//     for (const auto& seg : segments) {
//         if (seg.Time.size() > 0) {
//             total_points += seg.Time.size() -1;
//         }
//     }
//     total_points +=1;
//     if (total_points == 0) return ExecutionTrajectory{};
    
//     int D_spatial = segments[0].X.cols();
//     ExecutionTrajectory final_traj;
//     final_traj.X = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.V = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.A = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.Time = Eigen::VectorXd(total_points);

//     int current_row = 0;
//     for (size_t i = 0; i < segments.size(); ++i) {
//         const auto& seg = segments[i];
//         if (seg.Time.size() == 0) continue;
//         int points_to_copy = (i == segments.size() - 1) ? seg.Time.size() : seg.Time.size() - 1;

//         if (points_to_copy > 0) {
//             final_traj.Time.segment(current_row, points_to_copy) = seg.Time.head(points_to_copy);
//             final_traj.X.block(current_row, 0, points_to_copy, D_spatial) = seg.X.topRows(points_to_copy);
//             final_traj.V.block(current_row, 0, points_to_copy, D_spatial) = seg.V.topRows(points_to_copy);
//             final_traj.A.block(current_row, 0, points_to_copy, D_spatial) = seg.A.topRows(points_to_copy);
//             current_row += points_to_copy;
//         }
//     }
//     final_traj.is_valid = true;
//     if (total_points > 0) {
//        final_traj.total_cost = final_traj.Time.tail(1)[0] - final_traj.Time.head(1)[0];
//     }
//     return final_traj;
// }




// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("thruster_steer_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);

//     std::cout << "--- ThrusterSteerStateSpace Test ---\n";

//     std::vector<Eigen::VectorXd> waypoints;

//     const int DIM = 5; // [x, y, vx, vy, t]
//     const double MAX_ACCEL = 5.0; // m/s^2 per dimension
//     Eigen::VectorXd start_state(DIM);
//     start_state << 0.0, 0.0,    // x, y
//                    0.0, 0.0,    // vx, vy 
//                    20.0;        // t_start
//     Eigen::VectorXd end_state(DIM);
//     end_state   << 20.0, 10.0,   // x, y
//                    3.0, -4.0,   // vx, vy 
//                    10.0;        // t_end 

//     Eigen::VectorXd end_state2(DIM);
//     end_state2   << 10.0, -20.0,   // x, y
//                    3.0, -4.0,   // vx, vy 
//                    0.0;        // t_end 
//     int D_SPATIAL_DIM = 2;

//     waypoints.push_back(start_state);
//     waypoints.push_back(end_state);
//     waypoints.push_back(end_state2);

//     // --- Configuration for 7D (3D position/velocity/time) ---
//     // // Uncomment this block to test 7D
//     // const int DIM = 7; // [x, y, z, vx, vy, vz, t]
//     // const double MAX_ACCEL = 3.0; // m/s^2 per dimension
//     // Eigen::VectorXd start_state(DIM);
//     // start_state << 0.0, 0.0, 0.0,     // x, y, z
//     //                0.0, 0.0, 0.0,     // vx, vy, vz (start from rest)
//     //                10.0;               // t_start
//     // Eigen::VectorXd end_state(DIM);
//     // end_state   << 15.0, 10.0, 5.0,   // x, y, z
//     //                1.0, -1.0, 0.5,    // vx, vy, vz (end with some velocity)
//     //                0.0;              // t_end (target arrival time)
//     // int D_SPATIAL_DIM = 3;


//     // --- Create the Thruster State Space ---
//     std::shared_ptr<StateSpace> thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);

//     std::cout << "Attempting to steer from: "; printVec(start_state); std::cout << "\n";
//     std::cout << "To: "; printVec(end_state); std::cout << "\n";

//     std::vector<ExecutionTrajectory> execution_segments;

//     // --- 3. Call the steer function ---
//     auto start_time_steer = std::chrono::high_resolution_clock::now();
//     Trajectory traj = thruster_ss->steer(end_state, start_state); //EVEN THOUGH WE STEER FROM START TO END BUT WE ASSUME ROBOT IS GONNA START FROM THE END_STATE AND GO TO START_STATE
//     Trajectory traj2 = thruster_ss->steer(end_state2, end_state);
//     auto end_time_steer = std::chrono::high_resolution_clock::now();
//     auto duration_steer = std::chrono::duration_cast<std::chrono::microseconds>(end_time_steer - start_time_steer);
//     std::cout << "Time taken for the steer function: " << duration_steer.count() << " microseconds\n";
//     // The trick is to push_back backwards!
//     execution_segments.push_back(traj2.execution_data);
//     execution_segments.push_back(traj.execution_data);
//     //////////////////////////////
//     auto combined = combineExecutionTrajectories(execution_segments);
//     ///////////////////////////////
//     Params params;
//     params.setParam("thruster_state_dimension", DIM);
//     params.setParam("simulation_time_step", 0.05);
//     params.setParam("use_sim_time", false);

//     auto ros2_manager = std::make_shared<ROS2Manager>(visualizer, params);
//     ros2_manager->setInitialState(end_state2); // START AT P0
//     // ros2_manager->setPlannedThrusterTrajectory(execution_segments[0]);
//     ros2_manager->setPlannedThrusterTrajectory(combined);
//     //////////////////////////////
//     if (traj.is_valid) {
//         std::cout << "SUCCESS: Found a valid trajectory with cost (time elapsed): " << traj.cost << "s\n";
        
//         std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> path_segments;
//         if (traj.path_points.size() > 1) {
//             for (size_t i = 0; i < traj.path_points.size() - 1; ++i) {
//                 path_segments.emplace_back(getSpatialPosition(traj.path_points[i], D_SPATIAL_DIM),
//                                            getSpatialPosition(traj.path_points[i+1], D_SPATIAL_DIM));
//             }
//             for (size_t i = 0; i < traj2.path_points.size() - 1; ++i) {
//                 path_segments.emplace_back(getSpatialPosition(traj2.path_points[i], D_SPATIAL_DIM),
//                                            getSpatialPosition(traj2.path_points[i+1], D_SPATIAL_DIM));
//             }
//         }

//         rclcpp::executors::MultiThreadedExecutor executor;
//         executor.add_node(node);
//         executor.add_node(ros2_manager);
//         // executor.spin();

//         rclcpp::WallRate loop_rate(60); // Publish once per second
//         while(rclcpp::ok()) {
//             visualizer->visualizeEdges(path_segments, "map", "0.0,1.0,0.0", "thruster_path"); // Green line
//             visualizer->visualizeNodes(waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoint_nodes");
//             executor.spin_some();
//             loop_rate.sleep();
//         }

//     } else {
//         std::cout << "FAILURE: Could not find a valid trajectory between the states.\n";
//         std::cout << "This could be due to: \n";
//         std::cout << " - Insufficient or excessive time allocated (t_end - t_start).\n";
//         std::cout << " - Unreachable target velocity/position given max acceleration.\n";
//         std::cout << " - Numerical precision issues in the 1D solver.\n";
//     }

//     rclcpp::shutdown();
//     return 0;
// }






///////////////////////////////////////////////////
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <algorithm> 
#include <iomanip>   

#include "motion_planning/utils/params.hpp"
#include "motion_planning/utils/ros2_manager_thruster.hpp"
#include "rclcpp/rclcpp.hpp"
#include "motion_planning/utils/rviz_visualization.hpp"
#include "motion_planning/state_space/thruster_statespace.hpp"

Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
    return full_state.head(D_spatial_dim);
}

ExecutionTrajectory combineExecutionTrajectories(const std::vector<ExecutionTrajectory>& segments) {
    ExecutionTrajectory final_traj;
    final_traj.is_valid = false;
    if (segments.empty()) {
        return final_traj;
    }

    std::vector<double> time_vec;
    std::vector<Eigen::RowVectorXd> x_vec, v_vec;

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (seg.Time.size() == 0) continue;
        long start_j = (i == 0) ? 0 : 1;

        for (long j = start_j; j < seg.Time.size(); ++j) {
            time_vec.push_back(seg.Time(j));
            x_vec.push_back(seg.X.row(j));
            v_vec.push_back(seg.V.row(j));
        }
    }


    if (time_vec.empty()) {
        return final_traj;
    }

    // --- Populate the final trajectory object ---
    int D_spatial = x_vec[0].size();
    long num_points = time_vec.size();
    final_traj.Time.resize(num_points);
    final_traj.X.resize(num_points, D_spatial);
    final_traj.V.resize(num_points, D_spatial);

    for (long i = 0; i < num_points; ++i) {
        final_traj.Time(i) = time_vec[i];
        final_traj.X.row(i) = x_vec[i];
        final_traj.V.row(i) = v_vec[i];
    }
    
    final_traj.is_valid = true;
    return final_traj;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("thruster_steer_test_node");
    auto visualizer = std::make_shared<RVizVisualization>(node);

    std::cout << "--- ThrusterSteerStateSpace Test (Goal at Origin) ---\n";

    std::vector<Eigen::VectorXd> waypoints;
    const int DIM = 5; // [px, py, vx, vy, t]
    const double MAX_ACCEL = 5.0;
    int D_SPATIAL_DIM = 2;

    waypoints.push_back((Eigen::VectorXd(DIM) << 65.0, 50.0, 0.0, 0.0, 90.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 60.0, 48.0, -0.5, -0.5, 80.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 55.0, 42.0, -1.0, -0.5, 70.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 50.0, 35.0, 0.0, -1.0, 60.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 42.0, 28.0, -0.5, -2.0, 50.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 30.0, 22.0, -1.0, -1.5, 40.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 20.0, 15.0, -1.5, -1.0, 30.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 12.0, 8.0, -1.0, -0.5, 20.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 5.0, 2.0, -2.0, 1.0, 10.0).finished());
    waypoints.push_back((Eigen::VectorXd(DIM) << 0.0, 0.0, 0.0, 0.0, 0.0).finished());

    auto thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> all_path_segments_viz; 
    std::vector<ExecutionTrajectory> execution_segments;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Eigen::VectorXd from_state = waypoints[i];
        Eigen::VectorXd to_state = waypoints[i+1];

        std::cout << "\n\n====================================================\n";
        std::cout << "--- Steering Segment " << i << " ---\n";
        std::cout << "From: " << from_state.transpose() << "\n";
        std::cout << "To:   " << to_state.transpose() << "\n";
        std::cout << "====================================================\n";

        Trajectory traj = thruster_ss->steer(from_state, to_state);
        if (traj.is_valid) {
            // DEBUG PRINT: Show the raw output from the steer function for this segment
            std::cout << "--- Steer Output for Segment " << i << " (Valid) ---\n";
            std::cout << std::fixed << std::setprecision(4);
            const auto& data = traj.execution_data;
            for (long j = 0; j < data.Time.size(); ++j) {
                std::cout << "  Point[" << j << "]: P=[" << data.X.row(j)
                          << "], V=[" << data.V.row(j)
                          << "], T=[" << data.Time(j) << "]\n";
            }
            std::cout << "--- End of Steer Output for Segment " << i << " ---\n";

            execution_segments.push_back(traj.execution_data);
            if (traj.path_points.size() > 1) {
                for (size_t j = 0; j < traj.path_points.size() - 1; ++j) {
                    all_path_segments_viz.emplace_back(getSpatialPosition(traj.path_points[j], D_SPATIAL_DIM),
                                                       getSpatialPosition(traj.path_points[j+1], D_SPATIAL_DIM));
                }
            }
        } else {
            std::cout << "--- Steer Output for Segment " << i << " (INVALID) ---\n";
        }
    }

    auto combined = combineExecutionTrajectories(execution_segments);
    
    std::vector<Eigen::VectorXd> final_path_for_manager;
    if (combined.is_valid) {
        final_path_for_manager.reserve(combined.Time.size());
        for (long i = 0; i < combined.Time.size(); ++i) {
            Eigen::VectorXd state(DIM);
            
            state.head(D_SPATIAL_DIM) = combined.X.row(i);
            state.segment(D_SPATIAL_DIM, D_SPATIAL_DIM) = combined.V.row(i);
            state(DIM - 1) = combined.Time(i);

            final_path_for_manager.push_back(state);
        }
    }

    std::cout << "\n\n*********************************************\n";
    std::cout << "--- Final Path Data to be Sent to Manager ---\n";
    std::cout << "*********************************************\n";
    std::cout << std::fixed << std::setprecision(4); // For clean output
    for (size_t i = 0; i < final_path_for_manager.size(); ++i) {
        const auto& state = final_path_for_manager[i];
        Eigen::Vector2d pos = state.head<2>();
        Eigen::Vector2d vel = state.segment<2>(D_SPATIAL_DIM);
        double time = state(DIM - 1);
        std::cout << "State[" << i << "]: P=[" << pos.transpose()
                  << "], V=[" << vel.transpose()
                  << "], T=[" << time << "]\n";
    }
    std::cout << "--- End of Path Data ---\n\n";

    Params params;
    params.setParam("thruster_state_dimension", DIM);
    params.setParam("simulation_time_step", -0.05);
    params.setParam("use_sim_time", false);

    auto ros2_manager = std::make_shared<ROS2Manager>(visualizer, params);

    ros2_manager->setInitialState(waypoints.front()); 
    
    ros2_manager->setPlannedThrusterTrajectory(final_path_for_manager);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(ros2_manager);

    rclcpp::WallRate loop_rate(60);
    while(rclcpp::ok()) {
        visualizer->visualizeEdges(all_path_segments_viz, "map", "0.0,1.0,0.0", "thruster_path");
        visualizer->visualizeNodes(waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoint_nodes");
        executor.spin_some();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


// //////////////////////////////////////////////////////7D Many states backwards ///////////////////////////////////////////////////////////

// #include <iostream>
// #include <memory>
// #include <vector>
// #include <Eigen/Dense>
// #include <cmath> 
// #include <tuple> 
// #include "motion_planning/utils/params.hpp" 
// #include "motion_planning/utils/ros2_manager_thruster.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "motion_planning/utils/rviz_visualization.hpp"
// #include "motion_planning/state_space/thruster_statespace.hpp"

// static void printVec(const Eigen::VectorXd& v) {
//     std::cout << "[ ";
//     for (int i = 0; i < v.size(); ++i) {
//         std::cout << v[i] << (i + 1 < v.size() ? ", " : " ");
//     }
//     std::cout << "]";
// }

// Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& full_state, int D_spatial_dim) {
//     return full_state.head(D_spatial_dim);
// }

// ExecutionTrajectory combineExecutionTrajectories(const std::vector<ExecutionTrajectory>& segments) {
//     if (segments.empty()) {
//         return ExecutionTrajectory{};
//     }
//     int total_points = 0;
//     for (const auto& seg : segments) {
//         if (seg.Time.size() > 0) {
//             total_points += seg.Time.size() -1;
//         }
//     }
//     total_points +=1;
//     if (total_points == 0) return ExecutionTrajectory{};
    
//     int D_spatial = segments[0].X.cols();
//     ExecutionTrajectory final_traj;
//     final_traj.X = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.V = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.A = Eigen::MatrixXd(total_points, D_spatial);
//     final_traj.Time = Eigen::VectorXd(total_points);

//     int current_row = 0;
//     for (int i = segments.size(); i >= 0; --i) {
//         const auto& seg = segments[i];
//         if (seg.Time.size() == 0) continue;
//         int points_to_copy = (i == segments.size() - 1) ? seg.Time.size() : seg.Time.size() - 1;

//         if (points_to_copy > 0) {
//             final_traj.Time.segment(current_row, points_to_copy) = seg.Time.head(points_to_copy);
//             final_traj.X.block(current_row, 0, points_to_copy, D_spatial) = seg.X.topRows(points_to_copy);
//             final_traj.V.block(current_row, 0, points_to_copy, D_spatial) = seg.V.topRows(points_to_copy);
//             final_traj.A.block(current_row, 0, points_to_copy, D_spatial) = seg.A.topRows(points_to_copy);
//             current_row += points_to_copy;
//         }
//     }
//     final_traj.is_valid = true;
//     if (total_points > 0) {
//        final_traj.total_cost = final_traj.Time.tail(1)[0] - final_traj.Time.head(1)[0];
//     }
//     return final_traj;
// }




// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("thruster_steer_test_node");
//     auto visualizer = std::make_shared<RVizVisualization>(node);

//     std::cout << "--- ThrusterSteerStateSpace Test ---\n";

//     std::vector<Eigen::VectorXd> waypoints;

//     const int DIM = 7; // [x, y,z, vx, vy, vz, t]
//     const double MAX_ACCEL = 5.0; // m/s^2 per dimension

//     int D_SPATIAL_DIM = 3;
//     //--------------------------------------------x----y----z---vx---vy---vz--t----//
//     waypoints.push_back((Eigen::VectorXd(DIM) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0).finished());    // P0
//     waypoints.push_back((Eigen::VectorXd(DIM) << 5.0, 2.0, 2.0, 2.0, -1.0,-1.0, 80.0).finished());   // P1
//     waypoints.push_back((Eigen::VectorXd(DIM) << 12.0, 8.0, 8.0, 1.0, 0.5, 0.5, 70.0).finished());  // P2
//     waypoints.push_back((Eigen::VectorXd(DIM) << 20.0, 15.0, 15.0, 1.5, 1.0,1.0, 60.0).finished()); // P3
//     waypoints.push_back((Eigen::VectorXd(DIM) << 30.0, 22.0, 22.0, 1.0, 1.5, 1.5, 50.0).finished()); // P4
//     waypoints.push_back((Eigen::VectorXd(DIM) << 42.0, 28.0, 28.0, 0.5, 2.0, 2.0, 40.0).finished()); // P5
//     waypoints.push_back((Eigen::VectorXd(DIM) << 50.0, 35.0, 35.0, 0.0, 1.0, 1.0, 30.0).finished()); // P6
//     waypoints.push_back((Eigen::VectorXd(DIM) << 55.0, 42.0, 42.0, 1.0, 0.5, 0.5, 20.0).finished()); // P7
//     waypoints.push_back((Eigen::VectorXd(DIM) << 60.0, 48.0, 48.0, 0.5, 0.5, 0.5, 10.0).finished()); // P8
//     waypoints.push_back((Eigen::VectorXd(DIM) << 65.0, 50.0, 50.0, 0.0, 0.0, 0.0, 0.0).finished()); // P




//     std::shared_ptr<StateSpace> thruster_ss = std::make_shared<ThrusterSteerStateSpace>(DIM, MAX_ACCEL);
//     std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> all_path_segments_viz; 
//     double total_cost = 0.0;

//     std::vector<ExecutionTrajectory> execution_segments;
//     for (size_t i = 0; i < waypoints.size() - 1; ++i) {
//         Eigen::VectorXd from_state = waypoints[i+1]; 
//         Eigen::VectorXd to_state = waypoints[i];

//         std::cout << "\n--- Planning segment " << i << " (" << to_state[DIM-1] << "s to " << from_state[DIM-1] << "s) ---\n";
//         Trajectory traj = thruster_ss->steer(from_state, to_state);

//         if (traj.is_valid) {
//             std::cout << "  SUCCESS: Segment cost (time elapsed): " << traj.cost << "s\n";
//             execution_segments.push_back(traj.execution_data);
//             // Debug prints for trajectory points (optional, can remove for cleaner output)
//             if (!traj.path_points.empty()) {
//                 std::cout << "    DEBUG TRAJ: First point:  "; printVec(traj.path_points[0]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Second point: "; printVec(traj.path_points[1]); std::cout << "\n";
//                 std::cout << "    DEBUG TRAJ: Last point:   "; printVec(traj.path_points.back()); std::cout << "\n";
//             } else {
//                 std::cout << "    DEBUG TRAJ: Trajectory is empty!\n";
//             }
//             total_cost += traj.cost;
//             if (traj.path_points.size() > 1) {
//                 for (size_t j = 0; j < traj.path_points.size() - 1; ++j) {
//                     all_path_segments_viz.emplace_back(getSpatialPosition(traj.path_points[j], D_SPATIAL_DIM),
//                                                        getSpatialPosition(traj.path_points[j+1], D_SPATIAL_DIM));
//                 }
//             }
//         } else {
//             std::cout << "  FAILURE: Could not find a valid trajectory for this segment.\n";
//             std::cout << "  This segment's path will not be visualized.\n";
//         }
//     }


//     // std::reverse(execution_segments,execution_segments.begin(),execution_segments.end()); // instead of this i reversed combined it in combineExection function below!

//     auto combined = combineExecutionTrajectories(execution_segments);
//     Params params;
//     params.setParam("thruster_state_dimension", DIM);
//     params.setParam("simulation_time_step", 0.05);
//     params.setParam("use_sim_time", false);

//     auto ros2_manager = std::make_shared<ROS2Manager>(visualizer, params);
//     ros2_manager->setInitialState(waypoints[9]); // START AT P0
//     ros2_manager->setPlannedThrusterTrajectory(combined);
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.add_node(ros2_manager);
//     // executor.spin();

//     rclcpp::WallRate loop_rate(60); 
//     while(rclcpp::ok()) {
//         visualizer->visualizeEdges(all_path_segments_viz, "map", "0.0,1.0,0.0", "thruster_path"); // Green line
//         visualizer->visualizeNodes(waypoints, "map", {1.0f, 0.0f, 0.0f}, "waypoint_nodes");
//         executor.spin_some();
//         loop_rate.sleep();
//     }


//     rclcpp::shutdown();
//     return 0;
// }





