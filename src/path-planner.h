#include <vector>
#include <iostream>
#include <algorithm>
#include <queue>
#include <unordered_set>

struct coord
{
    int x;
    int y;

    coord(int xVal, int yVal) : x(xVal), y(yVal) {} // constructor

    bool operator==(const coord &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const coord &other) const
    {
        return x != other.x || y != other.y;
    }

    bool operator<(const coord &other) const
    {
        if (x == other.x)
        {
            return y < other.y;
        }
        return x < other.x;
    }
};

struct node
{
    coord pos;
    int f;
    int g;
    int h;
    coord parent;

    node(coord posVal, int fVal, int gVal, int hVal, coord parentVal) : pos(posVal), f(fVal), g(gVal), h(hVal), parent(parentVal) {} // constructor

    bool operator==(const node &other) const
    {
        return pos.x == other.pos.x && pos.y == other.pos.y;
    }
};

int get_heading(coord position);

int turn_amt(int current, int next);

int taxicab_dist(coord start, coord end);

coord closest_node(coord current, std::vector<coord> nodes);

bool isValid(coord pos, std::vector<coord> obstacles);

class Compare
{
public:
    bool operator()(const node &lhs, const node &rhs) const
    {
        return lhs.f > rhs.f;
    }
};

struct hash_node
{
    size_t operator()(const node &n) const
    {
        return std::hash<int>()(n.pos.x) ^ std::hash<int>()(n.pos.y);
    }
};

std::vector<coord> a_star(coord start, coord end, std::vector<coord> obstacles);

std::vector<int> move_sequence(std::vector<coord> path);

std::vector<int> complete_sequence(coord start, coord end, std::vector<coord> gates, std::vector<coord> obstacles);

std::vector<int> count_moves(std::vector<int> moves);
