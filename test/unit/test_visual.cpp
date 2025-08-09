// Copyright 2025 Soheil E.nia

#include "motion_planning/utils/rviz_visualization.hpp"
#include "rclcpp/rclcpp.hpp"

class FMTNode : public rclcpp::Node {
public:
    FMTNode() : Node("fmtx_visualizer") {
    }

    void initializeVisualizer() {
        visualizer_ = std::make_shared<RVizVisualization>(shared_from_this());
    }

    void triggerVisualization() {
        std::vector<Eigen::VectorXd> nodes;
        
        // Create two random 2D nodes
        Eigen::VectorXd node1 = Eigen::VectorXd::Random(2);
        Eigen::VectorXd node2 = Eigen::VectorXd::Random(2);

        // Add the nodes to the nodes vector
        nodes.push_back(node1);
        nodes.push_back(node2);
        
        // Visualize nodes
        visualizer_->visualizeNodes(nodes);

        // Create an edge between the two nodes
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
        edges.push_back(std::make_pair(node1, node2));
        
        // Visualize edges (lines between the nodes)
        visualizer_->visualizeEdges(edges);
    }

private:
    std::shared_ptr<Visualization> visualizer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create the node as a shared pointer
    auto node = std::make_shared<FMTNode>();

    // Now initialize the visualizer
    node->initializeVisualizer();
    rclcpp::Rate rate(1);  // Control the loop rate
    
    while (rclcpp::ok()) {
        node->triggerVisualization();
        rclcpp::spin_some(node);  // Handle incoming messages if any
        rate.sleep();  // Sleep to maintain the loop rate
    }


    // // Spin the node
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}