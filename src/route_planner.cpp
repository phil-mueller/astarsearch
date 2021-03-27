#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Define start and end nodes
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculate h value with the distance function
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Add neighbors to open list and calculate their g and h values
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // 1.Populate the neighbors vector for the current node
    current_node->FindNeighbors();
    // 2.Loop over neighbors
    for(RouteModel::Node *neighbor : current_node->neighbors){
        // Set parent
        neighbor->parent = current_node;
        // Set h value
        neighbor->h_value = CalculateHValue(neighbor);
        // Set g value
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        // Add to open list
        open_list.emplace_back(neighbor);
        // Set node's visited attribute to true
        neighbor->visited = true;
    }
}


// Sort open list and get next node depending on the lowest sum of f = h + g
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open list
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b){return (a->g_value+a->h_value > b->g_value+b->h_value);});
    // Create pointer to next node for output
    RouteModel::Node *next = open_list.back();
    // Remove next node from open list
    open_list.pop_back();
    // Return pointer to next node
    return next;
}


// Construct the final path starting from the last node
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Loop back over parents
    while(current_node!=start_node){
        // Add current node to path
        path_found.emplace_back(*current_node);
        // Add to distance variable
        distance += current_node->distance(*current_node->parent);
        // Set parent to next current node
        current_node = current_node->parent;
    }
    // Manually add the start node after breaking from the while loop
    path_found.emplace_back(*start_node);

    // Reverse order to get correct path direction
    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// A* algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Set current node to the starting point, set properties and add to open list
    current_node = start_node;
    current_node->g_value = 0;
    current_node->h_value = CalculateHValue(current_node);
    current_node->visited = true;
    open_list.emplace_back(current_node);

    // While loop
    while(current_node!=end_node){
        current_node = NextNode();
        AddNeighbors(current_node);
    }
    m_Model.path = ConstructFinalPath(current_node);

}