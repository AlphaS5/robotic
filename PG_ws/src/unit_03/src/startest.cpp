list<Node*> compute_path_A_star(Node* start, Node* goal)
{
    priority_queue< nodeDistance, vector< nodeDistance >, CompareDist> pq;
    vector<Node*> came_from;
    vector<float> cost_so_far;

    nodeDistance first;
    first.node = start;
    first.distance = 0.0f;
    pq.push(first);

    came_from.resize(20 * 20, nullptr);

    cost_so_far.resize(20 * 20, 9999.0f); 
    cost_so_far[start->x + 20 * start->y] = 0.0f;

    //Compute where we came from for every location that’s visited
    int i = 0;
    while (!pq.empty())
    {
        nodeDistance temp = pq.top();
        pq.pop();
        Node* current_node = temp.node;
        current_node = get_node(current_node->x, current_node->y);

        if (current_node == goal) break;

        for (auto n : current_node->neighbours)
        {
            float new_cost = cost_so_far[current_node->x + 20 * current_node->y] + n->cost;

            if (new_cost < cost_so_far[n->x + 20 * n->y])
            {
                cost_so_far[n->x + 20 * n->y] = new_cost;
                came_from[n->x + 20 * n->y] = current_node;

                nodeDistance newNode;
                newNode.node = n;
                newNode.distance = new_cost + heuristic(n, goal);
                pq.push(newNode);

            }
        }
    }



    #pragma region Create the path (start -> goal)

        Node* current = new Node();
        current = goal;
        list<Node*> path;
        path.push_back(current);
        while (current != start)
        {
            current = came_from[current->x + 20 * current->y];

            path.push_back(current);
        }
        path.reverse();

    #pragma endregion

    return path;
}
