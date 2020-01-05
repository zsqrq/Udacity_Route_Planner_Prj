#include "route_planner.h"
#include <algorithm>

using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    // construct m_model with input var : model
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    // Construct start and end node by using m_model's member function : FindClosetNode
    RoutePlanner::start_node = &RoutePlanner::m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &RoutePlanner::m_Model.FindClosestNode(end_x, end_y);

}


// 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calulate the distance between the end node and the input node
    return node->distance(*RoutePlanner::end_node);
}


//  4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Find the neighbors of the input current node
    current_node->FindNeighbors();
    //
    for (RouteModel::Node *neighbor: current_node->neighbors) {
        //current node is the parent node of its neighbors
        neighbor->parent = current_node;
        // calculate the hvalue and the gvalue of the neighbor
        neighbor->h_value = RoutePlanner::CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        // push the negihbors into the openlist
        RoutePlanner::open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }

}


// 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value, by asending order
    sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end(), [](const auto &_1st, const auto &_2nd){
        return (_1st->h_value + _1st->g_value) < (_2nd->h_value + _2nd->g_value);
    });
    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node* nextnode = RoutePlanner::open_list.front();

    RoutePlanner::open_list.erase(RoutePlanner::open_list.begin());
    // return the minimum fvalue node pointer
    return nextnode;
}


// 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Implement your solution here.
    while (current_node->parent != nullptr){
        distance += current_node->distance(*(current_node->parent));
        path_found.emplace(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }
    path_found.emplace(path_found.begin(), *current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    RoutePlanner::start_node->visited = true;
    RoutePlanner::open_list.emplace_back(RoutePlanner::start_node);

    while (!RoutePlanner::open_list.empty()){
        current_node = RoutePlanner::NextNode();

        if (current_node->distance(*RoutePlanner::end_node) == 0){
            RoutePlanner::m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
            return;
        }
        RoutePlanner::AddNeighbors(current_node);
    }
}