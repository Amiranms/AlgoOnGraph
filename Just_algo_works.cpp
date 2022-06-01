#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <array>
#include <utility>
#include <queue>
#include <tuple>
#include <cstdlib>
#include <vector>
#include <string>
#include <functional>
#include <map>
#include <algorithm>
#include <unordered_map>


//at the first time we will implement the graph structure

struct SimpleGraph{
    std::unordered_map<char,std::vector<char> > edges;

    std::vector<char> neighbors(char id){
        return edges[id];
    }
};

SimpleGraph example_graph{ { 
    {'A',{'B'}},
    {'B',{'C'}},
    {'C',{'B','D','F'}},
    {'D',{'C','E'}},
    {'E',{'F'}},
    {'F',{}}
}};

//поиск в ширину - начало
//we dont need it now 
/*
void breadth_first_search(SimpleGraph graph, char start) {
  std::queue<char> frontier;
  frontier.push(start);

  std::unordered_set<char> reached;
  reached.insert(start);

  while (!frontier.empty()) {
    char current = frontier.front();
    frontier.pop();

    std::cout << "  Visiting " << current << '\n';
    for (char next : graph.neighbors(current)) {
      if (reached.find(next) == reached.end()) {
        frontier.push(next);
        reached.insert(next);
      }
    }
  }
}
*/
//breadth-first search - end 

struct GridLocation {
    int x,y;
};

//по большому счёту создается дополнительная имплементацияи хэширующей функции, которая необходима для определения конкретного контейнера в который будем помещаться тот 
// или иной объект GridLocation в unordered_set
namespace std{
    /*implement has function so we can put GridLocation into an unordered_set */
    template <> struct hash<GridLocation>{
        typedef GridLocation argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const GridLocation& id) const noexcept {
            return std::hash<int>()(id.x^(id.y <<4));
        }

    };

}


//созданеи сеточной карты происходит здеся 

struct SquareGrid {
    static std::array<GridLocation,4> DIRS;

    int width, height;
    std::unordered_set<GridLocation> walls;

    SquareGrid(int width_,int height_): width(width_),height(height_) {}
    
    bool in_bounds(GridLocation id) const {
        return 0 <= id.x && id.x < width && 0 <= id.y && id.y <= height;
    }

    bool passable(GridLocation id) const {
        return walls.find(id) == walls.end();
    }

    std::vector<GridLocation> neighbors(GridLocation id) const {
        
        std::vector<GridLocation> results;

        for (GridLocation dir: DIRS){
            GridLocation next{id.x+dir.x,id.y+dir.y};
            if(in_bounds(next) && passable(next)){
                results.push_back(next);
            }
        }

        if ((id.x + id.y)% 2 ==0){
            //see "ugly paths" section for an explanation:
            std::reverse(results.begin(),results.end());
        }

        return results;
    }

};

std::array<GridLocation,4> SquareGrid::DIRS = {
    //east, west,north,south //
    GridLocation{1,0},GridLocation{-1,0},GridLocation{0,-1},
    GridLocation{0,1}

};

//Helpers for GridLocation

bool operator == (GridLocation a, GridLocation b){
    return a.x==b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b)
{
    return !(a==b);
}

bool operator < (GridLocation a, GridLocation b){
    return std::tie(a.x,a.y) < std::tie(b.x,b.y);
}

std::basic_iostream<char>::basic_ostream& operator <<(std::basic_iostream<char>::basic_ostream& out,
const GridLocation& loc){
    out<<'('<<loc.x<<','<<loc.y<<')';
    return out;
}

//This outputs a grid. WE can pass in a distances map if we want to
//print the distances, or we can pass in a point_to map if we want to print 
// arrows that point to the parent location, or we can pass in a path vector
//if we want to draw the path

template<class Graph>
void draw_grid(const Graph& graph,
                std::unordered_map<GridLocation, double>* distances=nullptr,
                std::unordered_map<GridLocation, GridLocation>* point_to = nullptr,
                std::vector<GridLocation>* path = nullptr,
                GridLocation* start = nullptr,
                GridLocation* goal = nullptr){
    const int field_width =3;
    std::cout << std::string(field_width * graph.width, '_')<<"\n";

    for( int y = 0; y!=graph.height; ++y){//traversing all points
        for (int x = 0 ; x != graph.width; ++x){
            GridLocation id {x,y};
            
            //we 'mark' it if it wall by 3 grids 
            if (graph.walls.find(id) != graph.walls.end()){
                std::cout << std::string(field_width, '#');
            } else if (start && id == *start){
                std::cout<< " A ";//if we in the start
                //if starting point was passed at all .....
            } else if (goal && id == * goal ) {
                std::cout<< " Z ";
            } else if (path != nullptr && find(path->begin(), path->end(), id) 
                                                != path->end()){

                    std::cout<< " @ ";            
            }else if (point_to != nullptr && point_to->count(id)) {
                GridLocation next = (*point_to)[id];
                if (next.x == x+1) { std::cout << " > " ;}
                else if (next.x == x - 1) { std::cout<< " < "; }
                else if (next.y == y + 1) { std::cout<< " v "; }
                else if (next.y == y - 1) { std::cout<< " ^ "; }
                else { std::cout << " * ";}
            }else if (distances !=nullptr && distances->count(id)){
                std::cout<< ' ' << std::left << std::setw(field_width-1) << (*distances)[id];
            }else {
                std::cout << " . ";
            }
        }
        std::cout<<"\n";
    }
    std::cout<<std::string(field_width * graph.width, '~')<<"\n";
}
//по большому счету помимо стандартной отрисовки карты теперь предоставляется
//возможность

void add_rect(SquareGrid& grid, int x1,int y1, int x2, int y2){
    for (int x = x1; x < x2; ++x){
        for (int y = y1; y < y2; ++y) {
            grid.walls.insert(GridLocation{x,y});
        }
    }
}

SquareGrid make_diagram1() {
    SquareGrid grid(30,15);
    add_rect(grid,3,3,5,12);//we just 
    add_rect(grid,13,4,15,15);
    add_rect(grid,21,0,23,7);
    add_rect(grid,23,5,26,7);
    return grid;
}


//now this is the time to implmenet the map with the obstacles 
//or grid with the weights..this may be called "forests"!!!!


struct GridWithWeights: SquareGrid {
    std::unordered_set<GridLocation> forests;
    GridWithWeights(int w, int h) : SquareGrid(w,h) {}
    //was used the great method, we used SquareGrid constructor for initialization
    //our new class object.Great job...
    double cost(GridLocation from_node, GridLocation to_node) const {
        return forests.find(to_node) != forests.end()? 5 : 1;
    }//if we in the forest then the cost of our movement increases five times 
};


GridWithWeights make_diagram4() {
    GridWithWeights grid(10,10);
    add_rect(grid,1,7,4,9);
    typedef GridLocation L;
    grid.forests = std::unordered_set<GridLocation> {
        L{3,4}, L{3,5}, L{4,1}, L{4,2},
        L{4,3}, L{4,4}, L{4,5}, L{4,6},
        L{4,7}, L{4,8}, L{5,1}, L{5,2},
        L{5,3}, L{5,4}, L{5,5}, L{5,6},
        L{5,7}, L{5,8}, L {6,2}, L {6,3},
        L {6,4}, L {6,5}, L{6,6}, L{6,7},
        L{7,3}, L{7,4}, L{7,5}
    };
    return grid;
}

//now just let's try breadth First Search again, keeping track of came_from:

template <typename Location, typename Graph>
std::unordered_map<Location, Location>
breadth_first_search(Graph graph, Location start, Location goal)
{
    std::queue<Location> frontier; //граница.
    frontier.push(start);

    std::unordered_map<Location,Location> came_from;
    came_from[start]=start;

    while(!frontier.empty()){
        Location current = frontier.front();
        frontier.pop();
        
        if (current == goal)
        {
            break;
        }

        for ( Location next : graph.neighbors(current)){
            if(came_from.find(next) == came_from.end())
            {
                frontier.push(next);
                came_from[next] = current;
            }
        }
    }
    return came_from; 

}

template <typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
                        std::greater<PQElement>> elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put( T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

//now it's time to implement dijkstra_search

template <typename Location, typename Graph>
void dijkstra_search(Graph graph,
                     Location start,
                     Location goal,
                     std::unordered_map<Location, Location> &came_from,
                     std::unordered_map<Location, double> &cost_so_far) 
{
    PriorityQueue<Location, double> frontier;
    frontier.put(start,0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while(!frontier.empty())
    {
        Location current = frontier.get();

        if(current == goal){
            break;
        }

        for (Location next: graph.neighbors(current)){
            double new_cost = cost_so_far[current] + graph.cost(current,next);
            if(cost_so_far.find(next) == cost_so_far.end()
                || new_cost < cost_so_far[next]){
                    cost_so_far[next] = new_cost;
                    came_from[next] = current;
                    frontier.put(next,new_cost);
             }
        }
    }
}

//visualisation of the path

template<typename Location>
std::vector<Location> reconstruct_path(
        Location start, Location goal,
        std::unordered_map<Location, Location> came_from
){
    std::vector<Location> path;
    Location current = goal;
    while (current!=start){
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); //nado li??
    std::reverse(path.begin(),path.end());
    return path;
}

inline double heuristic( GridLocation a, GridLocation b){
    return std::abs(a.x-b.x)+std::abs(a.y-b.y);
}

template<typename Location, typename Graph>
void a_star_search 
    (Graph graph,
    Location start,
    Location goal,
    std::unordered_map<Location,Location>& came_from,
    std::unordered_map<Location,double> & cost_so_far)
{
    PriorityQueue<Location,double> frontier;
    frontier.put(start,0);

    came_from[start]=start;
    cost_so_far[start]=0;

    while (!frontier.empty())
    {
        Location current = frontier.get();

        if (current == goal)
        {
            break;
        }

        for(Location next: graph.neighbors(current))
        {
            double new_cost = cost_so_far[current] + graph.cost(current,next);
            if(cost_so_far.find(next) == cost_so_far.end() ||
            new_cost < cost_so_far[next]){
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, goal);
                frontier.put(next,priority);
                came_from[next] = current;
            }
        }
    }
}

int main() {
  GridWithWeights grid = make_diagram4();
  GridLocation start{1, 4}, goal{8, 3};
  std::unordered_map<GridLocation, GridLocation> came_from;
  std::unordered_map<GridLocation, double> cost_so_far;
  a_star_search(grid, start, goal, came_from, cost_so_far);
  draw_grid(grid, nullptr, &came_from, nullptr, &start, &goal);
  std::cout << '\n';
  std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
  draw_grid(grid, nullptr, nullptr, &path, &start, &goal);
  std::cout << '\n';
  draw_grid(grid, &cost_so_far, nullptr, nullptr, &start, &goal);


/*
    SquareGrid grid = make_diagram1();
    GridLocation start{7, 8}, goal{17, 2};
    auto parents = breadth_first_search(grid, start, goal);
    draw_grid(grid, nullptr, &parents, nullptr, &start, &goal);
*/
} // useful to remeber that formating the string can be done by "ctrl+k, ctrl+f "
//shortcuts 
//u can use typedef std_class custom_name right in the function with no contradiction at all 

//WHY I AM WRITE THE CODE...
//HY I AM WRITE THE CODE... 
//WHY I AM WRITE THE CODE...
//why i'm write the code why i'm     
//why I am write the code 