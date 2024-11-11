#include <vector>
#include <iostream>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <stdint.h>

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

    // comparison for next permutation function
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

coord current_pos(0, 0);

// return int degree of heading where 0 is up and 270 is left
int get_heading(coord position)
{
    if (position.x == 0)
    {
        return 90;
    }
    else if (position.x == 8)
    {
        return 270;
    }
    else if (position.y == 0)
    {
        return 0;
    }
    else if (position.y == 10)
    {
        return 180;
    }
    else
    {
        return 0; // in case it fails or something
    }
}

// how much to turn based on heading, where heading is 0-360
int turn_amt(int current, int next)
{
    int cw = (next - current + 360) % 360;
    int ccw = (current - next + 360) % 360;

    if (cw < ccw)
    {
        return cw;
    }
    else
    {
        return -ccw;
    }
}

// manhattan/taxicab geometry distance
int taxicab_dist(coord start, coord end)
{
    return (abs(start.x - end.x) + abs(start.y - end.y));
}

// gives coordinate of closest node from current position, taxicab distance
coord closest_node(coord current, std::vector<coord> nodes)
{
    coord closest = nodes[0];

    for (coord node : nodes)
    {
        // if dist from new gate is less than
        if (taxicab_dist(node, current) < taxicab_dist(closest, current))
        {
            closest = node;
        }
    }

    return closest;
}

// check if a given position is valid
bool isValid(coord pos, std::vector<coord> obstacles)
{
    if (pos.x <= 0 || pos.x >= 8 || pos.y <= 0 || pos.y >= 10)
    {
        return false;
    }
    else if (pos.x % 2 == 0 && pos.y % 2 == 0)
    {
        return false;
    }
    else
    {
        for (coord obstacle : obstacles)
        {
            if (pos == obstacle)
            {
                return false;
            }
        }
    }

    return true;
}

// comparison function for priority queue
class Compare
{
public:
    bool operator()(const node &lhs, const node &rhs) const
    {
        return lhs.f > rhs.f;
    }
};

// custom hash to compare nodes
struct hash_node
{
    size_t operator()(const node &n) const
    {
        return std::hash<int>()(n.pos.x) ^ std::hash<int>()(n.pos.y);
    }
};

// Uses A* algorithm to find shortest path
std::vector<coord> a_star(coord start, coord end, std::vector<coord> obstacles)
{
    std::vector<coord> path;

    std::priority_queue<node, std::vector<node>, Compare> open;
    std::vector<node> closed;

    std::unordered_set<node, hash_node> in_open;
    std::unordered_set<node, hash_node> in_closed;

    node start_node(start, taxicab_dist(start, end), 0, taxicab_dist(start, end), start);
    open.push(start_node);

    while (!open.empty())
    {

        // find node with lowest f value
        node current = open.top();

        // move node from open to close list
        open.pop();
        closed.push_back(current);

        in_closed.insert(current); // add to in closed list

        // found path
        if (current.pos == end)
        {
            node current_node = current;

            while (current_node.pos != start)
            {
                path.push_back(current_node.pos);

                for (node i : closed)
                {
                    if (i.pos == current_node.parent)
                    {
                        current_node = i;
                        break;
                    }
                }
            }

            path.push_back(start);

            std::reverse(path.begin(), path.end());
            break;
        }

        std::vector<node> neighbors = {node(coord(current.pos.x + 1, current.pos.y), 0, 0, 0, current.pos),
                                       node(coord(current.pos.x - 1, current.pos.y), 0, 0, 0, current.pos),
                                       node(coord(current.pos.x, current.pos.y + 1), 0, 0, 0, current.pos),
                                       node(coord(current.pos.x, current.pos.y - 1), 0, 0, 0, current.pos)};

        for (auto &neighbor : neighbors)
        {

            // check if neighbor is in closed list
            bool foundInClosed = false;
            if (in_closed.find(neighbor) != in_closed.end())
            {
                foundInClosed = true;
            }

            // check if neighbor is already in closed list or is not valid position
            if (foundInClosed || !isValid(neighbor.pos, obstacles))
            {
                continue;
            }

            // check if new path to neighbor is shorter or neighbor is not in open list
            if ((taxicab_dist(start, neighbor.pos) + taxicab_dist(neighbor.pos, end) < taxicab_dist(start, current.pos) + taxicab_dist(current.pos, end)) || in_open.find(neighbor) == in_open.end())
            {
                neighbor.f = taxicab_dist(start, neighbor.pos) + taxicab_dist(neighbor.pos, end);
                neighbor.g = taxicab_dist(start, neighbor.pos);
                neighbor.h = taxicab_dist(neighbor.pos, end);
                neighbor.parent = current.pos;

                // check if neighbor is not in open list
                if (in_open.find(neighbor) == in_open.end())
                {
                    open.push(neighbor);
                    in_open.insert(neighbor);
                }
            }
        }
    }

    current_pos = end;
    return path;
}

// returns vector with steps to do, 0 is go forward, 90 is turn right, -90 is turn left, converting coords to moves
std::vector<int> move_sequence(std::vector<coord> path)
{
    std::vector<int> sequence;

    int heading = get_heading(path[0]); // will only return heading at the start

    for (int i = 0; i < path.size() - 1; i++)
    {
        coord current = path[i];
        coord next = path[i + 1];

        // generate the moves
        if (next.x == current.x + 1)
        {
            if (heading == 90)
            {
                sequence.push_back(0); // only need to go forward
            }
            else
            {
                sequence.push_back(turn_amt(heading, 90)); // turn first
                sequence.push_back(0);                     // go forward
                heading = 90;
            }
        }
        else if (next.x == current.x - 1)
        {
            if (heading == 270)
            {
                sequence.push_back(0);
            }
            else
            {
                sequence.push_back(turn_amt(heading, 270));
                sequence.push_back(0);
                heading = 270;
            }
        }
        else if (next.y == current.y + 1)
        {
            if (heading == 0)
            {
                sequence.push_back(0);
            }
            else
            {
                sequence.push_back(turn_amt(heading, 0));
                sequence.push_back(0);
                heading = 0;
            }
        }
        else if (next.y == current.y - 1)
        {
            if (heading == 180)
            {
                sequence.push_back(0);
            }
            else
            {
                sequence.push_back(turn_amt(heading, 180));
                sequence.push_back(0);
                heading = 180;
            }
        }
    }

    return sequence;
}

// index 0, 1, 2 is for 0 deg, 90 deg, 180 deg (neg is the same as pos), and total weighted sum
std::vector<int> count_moves(std::vector<int> moves)
{
    std::vector<int> nums = {0, 0, 0, 0};

    // based on testing at full speed(in milliseconds)
    int deg_0_weight = 1145;
    int deg_90_weight = 805;
    int deg_180_weight = 1540;

    for (int i : moves)
    {
        int amt = abs(i);

        if (amt == 0)
        {
            nums[0]++;
            nums[3] += deg_0_weight;
        }
        else if (amt == 90)
        {
            nums[1]++;
            nums[3] += deg_90_weight;
        }
        else
        {
            nums[2]++;
            nums[3] += deg_180_weight;
        }
    }

    return nums;
}

// finds the shortest sequence based on all permutations of gates
std::vector<int> complete_sequence(coord start, coord end, std::vector<coord> gates, std::vector<coord> obstacles)
{
    std::vector<int> sequence;
    std::vector<coord> permutation = gates;
    std::sort(permutation.begin(), permutation.end());

    int moves = INT32_MAX; // time it takes

    current_pos = start;

    do
    {
        std::vector<coord> new_path;
        std::vector<coord> temp;

        for (int i = 0; i < permutation.size(); i++)
        {
            // go to the next gate
            temp = a_star(current_pos, permutation[i], obstacles);
            new_path.insert(new_path.end(), temp.begin(), temp.end());
        }
        // go to the end
        temp = a_star(current_pos, end, obstacles);
        new_path.insert(new_path.end(), temp.begin(), temp.end());

        std::vector<int> new_sequence = move_sequence(new_path);
        int new_moves = count_moves(new_sequence)[3];

        // find the shortest sequence
        if (new_moves < moves)
        {
            sequence = new_sequence;
            moves = new_moves;
        }

        current_pos = start; // reset pos for next permutation

    } while (std::next_permutation(permutation.begin(), permutation.end()));

    // go to the end
    return sequence;
}


// //test setup
// int main(){
//     coord start(1, 0);
//     coord end(7, 9);
//     coord g1(7, 1);
//     coord g2(7, 5);
//     coord g3(1, 5);
//     coord g4(3, 9);
//     coord o1(1, 8);
//     coord o2(2, 1);
//     coord o3(2, 7);
//     coord o4(3, 2);
//     coord o5(4, 1);
//     coord o6(4, 3);
//     coord o7(4, 5);
//     coord o8(5, 2);
//     coord o9(5, 8);
//     coord o10(6, 3);

//     std::vector<coord> gates = {g1, g2, g3, g4};
//     std::vector<coord> obstacles = {o1, o2, o3, o4, o5, o6, o7, o8, o9, o10};
//     std::vector<coord> no_obstacles = {};

//     std::vector<int> path = complete_sequence(start, end, gates, obstacles);

//     for (int i : path){
//         std::cout << i << "\n";
//     }
// }

