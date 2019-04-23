#include <iostream>
#include <vector>
#include <limits>

using namespace std;

struct Node {
    Node(int x, int y) : x(x), y(y) {}

    int x;
    int y;

};

class Map {
public:
    Map(int width, int height, const vector<int> &map_) : width(width), height(height), map_(map_) {}

    int getWidth() const {
        return width;
    }

    int getHeight() const {
        return height;
    }

    const vector<int> &getMap() const {
        return map_;
    }

    int getIndex(const Node &n) const {
        return getIndex(n.x, n.y);
    }
    int getIndex(int x, int y) const {
        return x * getWidth() + y;
    }

    int getValue(int index) const {
        return map_[index];
    }

private:
    int width;
    int height;
    vector<int> map_;
};

class Dijkstra {
public:

    int findShortestPath(const Map &map, const Node &s, const Node &e, vector<Node> &path) {
        vector<int> costmap;
        costmap.resize(map.getMap().size(), std::numeric_limits<int>::max());

        int index = map.getIndex(e);
        costmap[index] = map.getValue(index);
        //小根堆处理，方便找到最小cost

    }


};


int main(int argc, char *argv[]) {

    vector<int> cost{1, 2, 3, 4,
                     4, 2, 3, 4,
                     3, 1, 4, 5,
                     2, 5, 6, 7};
    Map map(4, 4, cost);
    Dijkstra dijkstra;
    vector<Node> path;
    int pathCost = dijkstra.findShortestPath(map, Node(0, 1), Node(3, 2), path);
    cout << pathCost << endl;
    return 0;
}