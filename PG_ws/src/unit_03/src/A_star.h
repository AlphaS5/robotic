struct GridLocation {
  int x, y;
};

inline float heuristic(GridLocation a, GridLocation b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}


struct nodeDistance
{
    Node* node;
    float distance;
};

class CompareDist
{
public:
    bool operator()(nodeDistance& n1, nodeDistance& n2)
    {
        if (n1.distance > n2.distance)
            return true;
        else
            return false;
    }
};

list<Node*> Graph;

struct Node
{
    int x, y;
    float cost;
    Node* parent;
    list<Node*> neighbours;
};
