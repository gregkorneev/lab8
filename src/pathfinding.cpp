#include "pathfinding.h"

#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <iostream>

const double INF = 1e18;

// --------- Общие вспомогательные штуки ---------

struct Node {
    int y;
    int x;
    double g;
    double h;
};

struct NodeCmp {
    bool operator()(const Node &a, const Node &b) const {
        return (a.g + a.h) > (b.g + b.h);
    }
};

static double distCells(int y1, int x1, int y2, int x2) {
    double dy = double(y1 - y2);
    double dx = double(x1 - x2);
    return std::sqrt(dy * dy + dx * dx);
}

static double heuristic(int y, int x, int gy, int gx) {
    return distCells(y, x, gy, gx);
}

// Линия видимости (Bresenham)
static bool lineOfSight(const std::vector<std::string> &grid,
                        int y0, int x0, int y1, int x1) {
    int dy = y1 - y0;
    int dx = x1 - x0;

    int sy = (dy >= 0) ? 1 : -1;
    int sx = (dx >= 0) ? 1 : -1;

    dy = std::abs(dy);
    dx = std::abs(dx);

    int h = (int)grid.size();
    int w = (int)grid[0].size();

    int f = 0;

    auto checkCell = [&](int y, int x) -> bool {
        if (!inBounds(y, x, h, w)) return false;
        return isFree(grid, y, x);
    };

    if (dx >= dy) {
        while (x0 != x1) {
            f += dy;
            if (f >= dx) {
                if (!checkCell(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                    return false;
                y0 += sy;
                f -= dx;
            }
            if (!checkCell(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                return false;
            x0 += sx;
        }
    } else {
        while (y0 != y1) {
            f += dx;
            if (f >= dy) {
                if (!checkCell(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                    return false;
                x0 += sx;
                f -= dy;
            }
            if (!checkCell(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                return false;
            y0 += sy;
        }
    }
    return true;
}

// Восстановление пути
static void reconstructPath(const std::vector<std::vector<Cell>> &parent,
                            Cell start, Cell goal,
                            std::vector<Cell> &path) {
    path.clear();
    Cell cur = goal;
    while (!(cur.y == start.y && cur.x == start.x)) {
        path.push_back(cur);
        Cell p = parent[cur.y][cur.x];
        if (p.y == -1) {
            path.clear();
            return;
        }
        cur = p;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
}

// --------- A* ---------

bool aStar(const std::vector<std::string> &grid,
           Cell start, Cell goal,
           std::vector<Cell> &path,
           int &closedCount) {
    int h = (int)grid.size();
    int w = (int)grid[0].size();

    std::vector<std::vector<double>> g(h, std::vector<double>(w, INF));
    std::vector<std::vector<bool>>   closed(h, std::vector<bool>(w, false));
    std::vector<std::vector<Cell>>   parent(h, std::vector<Cell>(w, Cell{-1, -1}));

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;

    g[start.y][start.x] = 0.0;
    open.push(Node{start.y, start.x, 0.0, heuristic(start.y, start.x, goal.y, goal.x)});

    closedCount = 0;

    int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    int dx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        if (closed[cur.y][cur.x]) continue;
        closed[cur.y][cur.x] = true;
        closedCount++;

        if (cur.y == goal.y && cur.x == goal.x) {
            reconstructPath(parent, start, goal, path);
            return true;
        }

        for (int k = 0; k < 8; ++k) {
            int ny = cur.y + dy[k];
            int nx = cur.x + dx[k];

            if (!inBounds(ny, nx, h, w))  continue;
            if (!isFree(grid, ny, nx))    continue;
            if (closed[ny][nx])           continue;

            double tentativeG = g[cur.y][cur.x] + distCells(cur.y, cur.x, ny, nx);

            if (tentativeG < g[ny][nx]) {
                g[ny][nx] = tentativeG;
                parent[ny][nx] = Cell{cur.y, cur.x};
                open.push(Node{ny, nx, g[ny][nx], heuristic(ny, nx, goal.y, goal.x)});
            }
        }
    }
    return false;
}

// --------- Theta* ---------

bool thetaStar(const std::vector<std::string> &grid,
               Cell start, Cell goal,
               std::vector<Cell> &path,
               int &closedCount) {
    int h = (int)grid.size();
    int w = (int)grid[0].size();

    std::vector<std::vector<double>> g(h, std::vector<double>(w, INF));
    std::vector<std::vector<bool>>   closed(h, std::vector<bool>(w, false));
    std::vector<std::vector<Cell>>   parent(h, std::vector<Cell>(w, Cell{-1, -1}));

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;

    g[start.y][start.x] = 0.0;
    parent[start.y][start.x] = start;
    open.push(Node{start.y, start.x, 0.0, heuristic(start.y, start.x, goal.y, goal.x)});

    closedCount = 0;

    int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    int dx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        if (closed[cur.y][cur.x]) continue;
        closed[cur.y][cur.x] = true;
        closedCount++;

        if (cur.y == goal.y && cur.x == goal.x) {
            reconstructPath(parent, start, goal, path);
            return true;
        }

        for (int k = 0; k < 8; ++k) {
            int ny = cur.y + dy[k];
            int nx = cur.x + dx[k];

            if (!inBounds(ny, nx, h, w))  continue;
            if (!isFree(grid, ny, nx))    continue;
            if (closed[ny][nx])           continue;

            Cell par = parent[cur.y][cur.x];
            double tentativeG;
            Cell newParent;

            if (par.y != -1 && lineOfSight(grid, par.y, par.x, ny, nx)) {
                tentativeG = g[par.y][par.x] + distCells(par.y, par.x, ny, nx);
                newParent = par;
            } else {
                tentativeG = g[cur.y][cur.x] + distCells(cur.y, cur.x, ny, nx);
                newParent = Cell{cur.y, cur.x};
            }

            if (tentativeG < g[ny][nx]) {
                g[ny][nx] = tentativeG;
                parent[ny][nx] = newParent;
                open.push(Node{ny, nx, g[ny][nx], heuristic(ny, nx, goal.y, goal.x)});
            }
        }
    }
    return false;
}

// --------- Сглаживание пути (A*PS) ---------

void smoothPath(const std::vector<std::string> &grid,
                const std::vector<Cell> &in,
                std::vector<Cell> &out) {
    out.clear();
    if (in.empty()) return;

    int i = 0;
    out.push_back(in[0]);

    while (i < (int)in.size() - 1) {
        int j = i + 1;
        while (j < (int)in.size() &&
               lineOfSight(grid, in[i].y, in[i].x, in[j].y, in[j].x)) {
            j++;
        }
        out.push_back(in[j - 1]);
        i = j - 1;
    }
}

// --------- Длина пути ---------

double pathLength(const std::vector<Cell> &path) {
    if (path.size() < 2) return 0.0;
    double sum = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        sum += distCells(path[i-1].y, path[i-1].x, path[i].y, path[i].x);
    }
    return sum;
}

// --------- Dijkstra для L_opt ---------

struct NodeDist {
    int y;
    int x;
    double g;
};
struct NodeDistCmp {
    bool operator()(const NodeDist &a, const NodeDist &b) const {
        return a.g > b.g;
    }
};

bool computeOptimalPathLength(const std::vector<std::string> &grid,
                              Cell start, Cell goal,
                              double &L_opt) {
    int h = (int)grid.size();
    if (h == 0) {
        L_opt = 0.0;
        return false;
    }
    int w = (int)grid[0].size();

    std::vector<std::vector<double>> dist(h, std::vector<double>(w, INF));
    std::vector<std::vector<bool>>   used(h, std::vector<bool>(w, false));

    std::priority_queue<NodeDist, std::vector<NodeDist>, NodeDistCmp> pq;

    dist[start.y][start.x] = 0.0;
    pq.push(NodeDist{start.y, start.x, 0.0});

    int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    int dx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    while (!pq.empty()) {
        NodeDist cur = pq.top();
        pq.pop();

        if (used[cur.y][cur.x]) continue;
        used[cur.y][cur.x] = true;

        if (cur.y == goal.y && cur.x == goal.x) {
            L_opt = cur.g;
            return true;
        }

        for (int k = 0; k < 8; ++k) {
            int ny = cur.y + dy[k];
            int nx = cur.x + dx[k];

            if (!inBounds(ny, nx, h, w))  continue;
            if (!isFree(grid, ny, nx))    continue;

            double step = distCells(cur.y, cur.x, ny, nx);
            double ng   = dist[cur.y][cur.x] + step;

            if (ng < dist[ny][nx]) {
                dist[ny][nx] = ng;
                pq.push(NodeDist{ny, nx, ng});
            }
        }
    }

    L_opt = dist[goal.y][goal.x];
    return (L_opt < INF);
}

// --------- Гладкость, SUP и кривизна ---------

void computeSmoothness(const std::vector<Cell> &path,
                       double &SUP, double &GP,
                       double &curvature) {
    SUP = 0.0;
    GP = 0.0;
    curvature = 0.0;

    if (path.size() < 3) return;

    std::vector<double> angles;
    for (size_t i = 1; i < path.size(); ++i) {
        int dy = path[i].y - path[i-1].y;
        int dx = path[i].x - path[i-1].x;
        double a = std::atan2((double)dy, (double)dx);
        angles.push_back(a);
    }

    const double PI = 3.14159265358979323846;

    for (size_t i = 1; i < angles.size(); ++i) {
        double d = std::fabs(angles[i] - angles[i-1]);
        while (d > PI) d = std::fabs(d - 2.0 * PI);
        SUP += d;
    }

    if (angles.size() > 1) {
        GP = SUP / (double)(angles.size() - 1);
    }

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        double x_im1 = path[i-1].x;
        double y_im1 = path[i-1].y;
        double x_i   = path[i].x;
        double y_i   = path[i].y;
        double x_ip1 = path[i+1].x;
        double y_ip1 = path[i+1].y;

        double vx  = x_ip1 - x_i;
        double vy  = y_ip1 - y_i;
        double dvx = (x_ip1 - x_i) - (x_i - x_im1);
        double dvy = (y_ip1 - y_i) - (y_i - y_im1);

        double num = std::sqrt(dvx*dvx + dvy*dvy);
        double len = std::sqrt(vx*vx + vy*vy);
        double den = std::pow(len, 3.0);

        if (den > 1e-9) curvature += num / den;
    }
}

// --------- Расстояние до препятствий ---------

void computeObstacleDistances(const std::vector<std::string> &grid,
                              const std::vector<Cell> &path,
                              double &minDist, double &avgDist) {
    int h = (int)grid.size();
    if (h == 0 || path.empty()) {
        minDist = 0.0;
        avgDist = 0.0;
        return;
    }
    int w = (int)grid[0].size();

    std::vector<Cell> obstacles;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (grid[y][x] == '#')
                obstacles.push_back(Cell{y, x});

    if (obstacles.empty()) {
        minDist = 0.0;
        avgDist = 0.0;
        return;
    }

    minDist = INF;
    double sum = 0.0;

    for (const Cell &p : path) {
        double best = INF;
        for (const Cell &o : obstacles) {
            double d = distCells(p.y, p.x, o.y, o.x);
            if (d < best) best = d;
        }
        if (best < minDist) minDist = best;
        sum += best;
    }

    avgDist = sum / (double)path.size();
}

// --------- Точность эвристики (RMSE) ---------

double computeHeuristicError(const std::vector<std::string> &grid,
                             Cell goal) {
    int h = (int)grid.size();
    if (h == 0) return 0.0;
    int w = (int)grid[0].size();

    std::vector<std::vector<double>> dist(h, std::vector<double>(w, INF));
    std::priority_queue<NodeDist, std::vector<NodeDist>, NodeDistCmp> pq;

    dist[goal.y][goal.x] = 0.0;
    pq.push(NodeDist{goal.y, goal.x, 0.0});

    int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    int dx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    while (!pq.empty()) {
        NodeDist cur = pq.top();
        pq.pop();

        if (cur.g > dist[cur.y][cur.x]) continue;

        for (int k = 0; k < 8; ++k) {
            int ny = cur.y + dy[k];
            int nx = cur.x + dx[k];

            if (!inBounds(ny, nx, h, w))  continue;
            if (!isFree(grid, ny, nx))    continue;

            double step = distCells(cur.y, cur.x, ny, nx);
            double ng   = dist[cur.y][cur.x] + step;

            if (ng < dist[ny][nx]) {
                dist[ny][nx] = ng;
                pq.push(NodeDist{ny, nx, ng});
            }
        }
    }

    double sumSq = 0.0;
    int count = 0;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (!isFree(grid, y, x)) continue;
            if (dist[y][x] >= INF/2) continue;

            double hVal = heuristic(y, x, goal.y, goal.x);
            double trueCost = dist[y][x];
            double diff = hVal - trueCost;
            sumSq += diff * diff;
            count++;
        }
    }

    if (count == 0) return 0.0;
    return std::sqrt(sumSq / (double)count);
}

// --------- Заполнение всех метрик ---------

void fillMetrics(const std::vector<std::string> &grid,
                 Cell start, Cell goal,
                 const std::vector<Cell> &finalPath,
                 int closedCount,
                 double heurError,
                 PathMetrics &m) {
    m.closedCount = closedCount;
    m.L_found = pathLength(finalPath);

    double L_opt = 0.0;
    bool okOpt = computeOptimalPathLength(grid, start, goal, L_opt);
    if (okOpt) {
        m.L_opt = L_opt;
    } else {
        m.L_opt = m.L_found;
    }

    if (m.L_opt > 0.0) {
        m.KO = m.L_found / m.L_opt;
        m.OO_percent = (m.L_found - m.L_opt) / m.L_opt * 100.0;
    } else {
        m.KO = 0.0;
        m.OO_percent = 0.0;
    }

    if (m.closedCount > 0 && m.L_opt > 0.0) {
        m.EP = m.L_opt / (double)m.closedCount;
    } else {
        m.EP = 0.0;
    }

    int edges = (finalPath.size() > 1) ? (int)finalPath.size() - 1 : 1;
    if (m.closedCount > 0 && edges > 0) {
        m.FV = std::pow((double)m.closedCount, 1.0 / (double)edges);
    } else {
        m.FV = 0.0;
    }

    computeSmoothness(finalPath, m.SUP, m.GP, m.curvature);
    computeObstacleDistances(grid, finalPath, m.minObsDist, m.avgObsDist);
    m.heurError = heurError;
}

// --------- Сохранение метрик в CSV ---------

void saveMetricsToCsv(const std::string &algorithmName,
                      int h, int w,
                      const PathMetrics &m,
                      int runIndex) {
    namespace fs = std::filesystem;
    fs::create_directories("data/csv");

    std::string filename = "data/csv/metrics.csv";
    bool needHeader = !fs::exists(filename);

    std::ofstream out(filename, std::ios::app);
    if (!out.is_open()) {
        std::cerr << "Не удалось открыть файл метрик: " << filename << "\n";
        return;
    }

    if (needHeader) {
        out << "algorithm,runIndex,height,width,"
            << "L_found,L_opt,KO,OO_percent,"
            << "closedCount,EP,FV,"
            << "SUP,GP,"
            << "minObsDist,avgObsDist,"
            << "curvature,heurError\n";
    }

    out << algorithmName << ","
        << runIndex << ","
        << h << "," << w << ","
        << m.L_found << "," << m.L_opt << ","
        << m.KO << "," << m.OO_percent << ","
        << m.closedCount << ","
        << m.EP << "," << m.FV << ","
        << m.SUP << "," << m.GP << ","
        << m.minObsDist << "," << m.avgObsDist << ","
        << m.curvature << "," << m.heurError << "\n";
}
