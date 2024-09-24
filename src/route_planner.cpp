#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates by m_Model.findClosestNode method.
    // Then, store the nodes we find in the RoutePlanner's start_node and end_node attributes.
	start_node = &m_Model.FindClosestNode(start_x, start_y);
	end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// CalculateHValue method to find heuristic value from the current node to end_node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return end_node -> distance(*node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	// Use the FindNeighbors method of the current_node to populate its neighbors.
	current_node -> FindNeighbors();
	for (auto& neighbor : current_node -> neighbors){
		// Update the neighbor's parent, h_value and g_value
		neighbor -> parent = current_node;
		neighbor -> h_value = RoutePlanner::CalculateHValue(neighbor);
		neighbor -> g_value = current_node -> g_value + neighbor -> distance(*current_node);

		// Mark neighbor's visited variable, so that we don't visit twice again.
		neighbor -> visited = true;
		// Add the current's neighbor
		open_list.emplace_back(neighbor);
	}
}


// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

// Set up next node we will process sequentially.
RouteModel::Node *RoutePlanner::NextNode() {
	// Sort the open_list by descending order of sum for h value and g value using lambda expression.
	sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* A, const RouteModel::Node* B) {
		return A -> h_value + A -> g_value > B -> h_value + B -> g_value;
	});

	// Create a pointer to the node in the vector with the lowest sum.
	RouteModel::Node* lowest = open_list.back();
	open_list.pop_back();
	return lowest;
}

// Backtrack the path from the destination node to origin node.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	
	// Set up the destination node to track
	path_found.emplace_back(*current_node);
	// Track all the routes by means of current_node's parent until we find the origin node.
	while (current_node != start_node){
		distance += current_node -> distance(*(current_node -> parent));
		
		// Move forward to the next parent's node.
		current_node = current_node -> parent;
		path_found.push_back(*current_node);
	}
	
	// Reverse the path to rectify from beginning start_node to end_node.
	reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// Overall AStarSearch's pipeline
void RoutePlanner::AStarSearch() {
    // Time Complexity: O(n * n * log(n))
	// Space Complexity: O(n) where n is the number of nodes.
	
	// Set up the start_node before searching
	RouteModel::Node *current_node = nullptr;
	start_node -> visited = true;
	open_list.push_back(start_node);

	while (open_list.size() > 0 ){
		// Sort the open_list and then get the next node with the lowest sum of scores.
		current_node = NextNode();

		// Expand its adjacent neighbors of the current to open_list
		AddNeighbors(current_node);
		
		// If we reached end_node(destination), then save the path in the m_Model.path attribute and stop loop.
		// The path would be displayed on the map tile.
		if (current_node -> x == end_node -> x && current_node -> y == end_node -> y){
			m_Model.path = ConstructFinalPath(current_node);
			break;
		}
	}
}
