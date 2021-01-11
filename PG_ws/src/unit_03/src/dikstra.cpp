#include <iostream>
#include <vector>
#include <string>
#include <list>

#include <limits> // for numeric_limits

#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>
#include "dikstra.h"

using namespace std;

/*typedef int vertex_t;
typedef double weight_t;

const weight_t max_weight = numeric_limits<double>::infinity();

struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

typedef vector<vector<neighbor> > adjacency_list_t;

*/
void DijkstraComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          vector<weight_t> &min_distance,
                          vector<vertex_t> &previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    set<pair<weight_t, vertex_t> > vertex_queue;
    vertex_queue.insert(make_pair(min_distance[source], source));

    while (!vertex_queue.empty())
    {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
	const vector<neighbor> &neighbors = adjacency_list[u];
        for (vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
	    if (distance_through_u < min_distance[v]) {
	        vertex_queue.erase(make_pair(min_distance[v], v));

	        min_distance[v] = distance_through_u;
	        previous[v] = u;
	        vertex_queue.insert(make_pair(min_distance[v], v));

	    }

        }
    }
}


list<vertex_t> DijkstraGetShortestPathTo(
    vertex_t vertex, const vector<vertex_t> &previous)
{
    list<vertex_t> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}


list<vertex_t> dikstra_main(int start_knoten, int ziel_knoten){

    adjacency_list_t adjacency_list(42);
    adjacency_list[0].push_back(neighbor(1, 1));
    adjacency_list[1].push_back(neighbor(2, 1));
    adjacency_list[2].push_back(neighbor(3, 1));
    adjacency_list[3].push_back(neighbor(4, 1));
    adjacency_list[4].push_back(neighbor(5, 1));
    adjacency_list[5].push_back(neighbor(6, 1));
    adjacency_list[6].push_back(neighbor(7, 1));
    adjacency_list[7].push_back(neighbor(8, 1));
    adjacency_list[8].push_back(neighbor(9, 1));
    adjacency_list[9].push_back(neighbor(10, 1));
    adjacency_list[10].push_back(neighbor(11, 1));
    adjacency_list[11].push_back(neighbor(12, 1));
    adjacency_list[12].push_back(neighbor(13, 1));
    adjacency_list[13].push_back(neighbor(11, 1));
    adjacency_list[11].push_back(neighbor(13, 1));
    adjacency_list[13].push_back(neighbor(14, 1));
    adjacency_list[14].push_back(neighbor(15, 1));
    adjacency_list[15].push_back(neighbor(16, 1));
    adjacency_list[16].push_back(neighbor(8, 1));
    adjacency_list[8].push_back(neighbor(16, 1));
    adjacency_list[16].push_back(neighbor(17, 1));
    adjacency_list[17].push_back(neighbor(7, 1));
    adjacency_list[7].push_back(neighbor(17, 1));
    adjacency_list[17].push_back(neighbor(18, 1));
    adjacency_list[18].push_back(neighbor(17, 1));
    adjacency_list[17].push_back(neighbor(6, 1));
    adjacency_list[6].push_back(neighbor(17, 1));
    adjacency_list[7].push_back(neighbor(18, 1));
    adjacency_list[18].push_back(neighbor(19, 1));
    adjacency_list[19].push_back(neighbor(20, 1));
    adjacency_list[20].push_back(neighbor(5, 1));
    adjacency_list[5].push_back(neighbor(20, 1));
    adjacency_list[20].push_back(neighbor(21, 1));
    adjacency_list[21].push_back(neighbor(4, 1));
    adjacency_list[4].push_back(neighbor(21, 1));
    adjacency_list[21].push_back(neighbor(22, 1));
    adjacency_list[22].push_back(neighbor(3, 1));
    adjacency_list[3].push_back(neighbor(22, 1));
    adjacency_list[22].push_back(neighbor(23, 1));
    adjacency_list[23].push_back(neighbor(2, 1));
    adjacency_list[2].push_back(neighbor(23, 1));
    adjacency_list[23].push_back(neighbor(24, 1));
    adjacency_list[24].push_back(neighbor(25, 1));
    adjacency_list[25].push_back(neighbor(26, 1));
    adjacency_list[26].push_back(neighbor(27, 1));
    adjacency_list[27].push_back(neighbor(28, 1));
    adjacency_list[28].push_back(neighbor(29, 1));
    adjacency_list[29].push_back(neighbor(25, 1));
    adjacency_list[25].push_back(neighbor(29, 1));
    adjacency_list[29].push_back(neighbor(30, 1));
    adjacency_list[30].push_back(neighbor(37, 1));
    adjacency_list[37].push_back(neighbor(30, 1));
    adjacency_list[30].push_back(neighbor(31, 1));
    adjacency_list[31].push_back(neighbor(32, 1));
    adjacency_list[32].push_back(neighbor(33, 1));
    adjacency_list[33].push_back(neighbor(34, 1));
    adjacency_list[34].push_back(neighbor(35, 1));
    adjacency_list[35].push_back(neighbor(36, 1));
    adjacency_list[36].push_back(neighbor(35, 1));
    adjacency_list[36].push_back(neighbor(37, 1));
    adjacency_list[37].push_back(neighbor(36, 1));
    adjacency_list[36].push_back(neighbor(38, 1));
    adjacency_list[38].push_back(neighbor(33, 1));
    adjacency_list[33].push_back(neighbor(38, 1));
    adjacency_list[38].push_back(neighbor(39, 1));
    adjacency_list[39].push_back(neighbor(32, 1));
    adjacency_list[32].push_back(neighbor(39, 1));
    adjacency_list[39].push_back(neighbor(40, 1));
    adjacency_list[40].push_back(neighbor(41, 1));
    adjacency_list[41].push_back(neighbor(30, 1));
    adjacency_list[30].push_back(neighbor(41, 1));
    adjacency_list[41].push_back(neighbor(2, 1));


    vector<weight_t> min_distance;
    vector<vertex_t> previous;
    DijkstraComputePaths(start_knoten, adjacency_list, min_distance, previous); //instead of 0 Start
    //std::cout << "Distance from 0 to 5: " << min_distance[5] << std::endl;
    list<vertex_t> path = DijkstraGetShortestPathTo(ziel_knoten, previous); //instead of 5 Ziel
    //std::cout << "Path : ";
    //copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    //std::cout << std::endl;
    return path;
}
