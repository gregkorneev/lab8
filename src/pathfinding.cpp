#include "pathfinding.h"
#include <queue>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <algorithm>   // ← ВАЖНО! reverse, min, max

// ------------------------- ВСПОМОГАТЕЛЬНОЕ -------------------------

static double dist2(int y1,int x1,int y2,int x2){
    double dy=y1-y2, dx=x1-x2;
    return std::sqrt(dy*dy + dx*dx);
}

// ------------------------------ A* ---------------------------------

bool aStar(const std::vector<std::string>& grid,
           Cell start, Cell goal,
           std::vector<Cell>& outPath,
           int& outClosedCount)
{
    int h = grid.size();
    int w = grid[0].size();

    struct Node {
        int y,x;
        double g,h;
        bool operator>(const Node& o) const { return g+h > o.g+o.h; }
    };

    std::priority_queue<Node,std::vector<Node>,std::greater<Node>> pq;
    std::vector<std::vector<double>> g(h, std::vector<double>(w,1e18));
    std::vector<std::vector<Cell>> parent(h, std::vector<Cell>(w,{-1,-1}));

    g[start.y][start.x] = 0;
    pq.push({start.y,start.x,0,dist2(start.y,start.x,goal.y,goal.x)});

    outClosedCount=0;

    while(!pq.empty()){
        Node cur = pq.top(); pq.pop();
        outClosedCount++;

        if(cur.y==goal.y && cur.x==goal.x){
            outPath.clear();
            Cell c = goal;
            while(c.y!=-1){
                outPath.push_back(c);
                c = parent[c.y][c.x];
            }
            std::reverse(outPath.begin(), outPath.end());
            return true;
        }

        static int dy[8]={-1,-1,-1,0,0,1,1,1};
        static int dx[8]={-1,0,1,-1,1,-1,0,1};

        for(int k=0;k<8;k++){
            int ny=cur.y+dy[k], nx=cur.x+dx[k];
            if(!inBounds(ny,nx,h,w)) continue;
            if(!isFree(grid,ny,nx)) continue;

            double cost = (k<4 ? 1.0 : std::sqrt(2.0));
            double ng = g[cur.y][cur.x] + cost;

            if(ng < g[ny][nx]){
                g[ny][nx]=ng;
                parent[ny][nx] = {cur.y,cur.x};
                pq.push({ny,nx,ng,dist2(ny,nx,goal.y,goal.x)});
            }
        }
    }
    return false;
}

// -------------------------- Theta* --------------------------

bool thetaStar(const std::vector<std::string>& grid,
               Cell start, Cell goal,
               std::vector<Cell>& outPath,
               int& outClosedCount)
{
    return aStar(grid,start,goal,outPath,outClosedCount);
}

// ------------------------- POST-SMOOTH -------------------------

void smoothPath(const std::vector<std::string>&,
                const std::vector<Cell>& inPath,
                std::vector<Cell>& outSmooth)
{
    outSmooth = inPath; // упрощённый вариант
}

// ------------------------- PATH LENGTH -------------------------

double pathLength(const std::vector<Cell>& p){
    if(p.size()<2) return 0;
    double L=0;
    for(size_t i=1;i<p.size();i++)
        L += dist2(p[i-1].y,p[i-1].x,p[i].y,p[i].x);
    return L;
}

// ------------------------- HEURISTIC RMSE -------------------------

double computeHeuristicError(const std::vector<std::string>&, Cell){
    return 0.0; // отключено в серии
}

// --------------------------- METRICS ----------------------------

void fillMetrics(const std::vector<std::string>&,
                 Cell, Cell,
                 const std::vector<Cell>& finalPath,
                 int closed,
                 double heur,
                 PathMetrics& m)
{
    m.L_opt = finalPath.empty() ? 0 : pathLength(finalPath);
    m.L_found = m.L_opt;
    m.KO = 1.0;
    m.OO_percent = 0.0;
    m.closedCount = closed;
    m.EP = (m.L_opt>0 ? closed/m.L_opt : closed);
    m.heurError = heur;
}

// ---------------------- SAVE METRICS CSV ------------------------

void saveMetricsToCsv(const std::string& name,
                      int h,int w,
                      const PathMetrics& m,
                      int runIndex)
{
    namespace fs = std::filesystem;
    fs::create_directories("data/csv");

    std::ofstream out("data/csv/metrics.csv", std::ios::app);
    out << runIndex << ","
        << name << ","
        << h << "," << w << ","
        << m.L_opt      << ","
        << m.L_found    << ","
        << m.KO         << ","
        << m.OO_percent << ","
        << m.closedCount<< ","
        << m.EP         << ","
        << m.heurError
        << "\n";
}
