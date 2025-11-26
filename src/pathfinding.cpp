#include "pathfinding.h"
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

const double INF = 1e18;

// Узел для очереди A*
struct Node {
    int y;
    int x;
    double g;   // расстояние от старта
    double h;   // эвристика до цели
    int py;     // родитель
    int px;
};

// Компаратор для приоритетной очереди (минимальный f = g+h наверху)
struct NodeCmp {
    bool operator()(const Node &a, const Node &b) const {
        return (a.g + a.h) > (b.g + b.h);
    }
};

// Евклидово расстояние
static double distCells(int y1, int x1, int y2, int x2) {
    double dy = double(y1 - y2);
    double dx = double(x1 - x2);
    return std::sqrt(dy * dy + dx * dx);
}

static double heuristic(int y, int x, int gy, int gx) {
    return distCells(y, x, gy, gx);
}

// Линия видимости между двумя клетками (Bresenham)
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

    if (dx >= dy) {
        while (x0 != x1) {
            f += dy;
            if (f >= dx) {
                if (!inBounds(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2), h, w))
                    return false;
                if (!isFree(grid, y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                    return false;
                y0 += sy;
                f -= dx;
            }
            if (!inBounds(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2), h, w))
                return false;
            if (!isFree(grid, y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                return false;
            x0 += sx;
        }
    } else {
        while (y0 != y1) {
            f += dx;
            if (f >= dy) {
                if (!inBounds(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2), h, w))
                    return false;
                if (!isFree(grid, y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                    return false;
                x0 += sx;
                f -= dy;
            }
            if (!inBounds(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2), h, w))
                return false;
            if (!isFree(grid, y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)))
                return false;
            y0 += sy;
        }
    }
    return true;
}

// Восстановление пути по таблице родителей
static void reconstructPath(const std::vector<std::vector<Cell>> &parent,
                            Cell start, Cell goal,
                            std::vector<Cell> &path) {
    path.clear();
    Cell cur = goal;
    while (!(cur.y == start.y && cur.x == start.x)) {
        path.push_back(cur);
        Cell p = parent[cur.y][cur.x];
        if (p.y == -1) { // пути нет
            path.clear();
            return;
        }
        cur = p;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
}

bool aStar(const std::vector<std::string> &grid,
           Cell start, Cell goal,
           std::vector<Cell> &path,
           int &closedCount) {
    int h = (int)grid.size();
    int w = (int)grid[0].size();

    std::vector<std::vector<double> > g(h, std::vector<double>(w, INF));
    std::vector<std::vector<bool> > closed(h, std::vector<bool>(w, false));
    std::vector<std::vector<Cell> > parent(h, std::vector<Cell>(w, Cell{-1, -1}));

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;

    g[start.y][start.x] = 0.0;

    Node s;
    s.y = start.y;
    s.x = start.x;
    s.g = 0.0;
    s.h = heuristic(start.y, start.x, goal.y, goal.x);
    s.py = start.y;
    s.px = start.x;
    open.push(s);

    closedCount = 0;

    int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    int dx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        if (closed[cur.y][cur.x])
            continue;

        closed[cur.y][cur.x] = true;
        closedCount++;

        if (cur.y == goal.y && cur.x == goal.x) {
            reconstructPath(parent, start, goal, path);
            return true;
        }

        for (int k = 0; k < 8; ++k) {
            int ny = cur.y + dy[k];
            int nx = cur.x + dx[k];

            if (!inBounds(ny, nx, h, w))
                continue;
            if (!isFree(grid, ny, nx))
                continue;
            if (closed[ny][nx])
                continue;

            double tentativeG = g[cur.y][cur.x] + distCells(cur.y, cur.x, ny, nx);

            if (tentativeG < g[ny][nx]) {
                g[ny][nx] = tentativeG;
                parent[ny][nx] = Cell{cur.y, cur.x};

                Node n;
                n.y = ny;
                n.x = nx;
                n.g = g[ny][nx];
                n.h = heuristic(ny, nx, goal.y, goal.x);
                n.py = cur.y;
                n.px = cur.x;
                open.push(n);
            }
        }
    }
    return false;
}

bool thetaStar(const std::vector<std::string> &grid,
               Cell start, Cell goal,
               std::vector<Cell> &path,
               int &closedCount) {
    int h = (int)grid.size();
    int w = (int)grid[0].size();

    std::vector<std::vector<double> > g(h, std::vector<double>(w, INF));
    std::vector<std::vector<bool> > closed(h, std::vector<bool>(w, false));
    std::vector<std::vector<Cell> > parent(h, std::vector<Cell>(w, Cell{-1, -1}));

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;

    g[start.y][start.x] = 0.0;
    parent[start.y][start.x] = start;

    Node s;
    s.y = start.y;
    s.x = start.x;
    s.g = 0.0;
    s.h = heuristic(start.y, start.x, goal.y, goal.x);
    s.py = start.y;
    s.px = start.x;
    open.push(s);

    closedCount = 0;

    int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    int dx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        if (closed[cur.y][cur.x])
            continue;

        closed[cur.y][cur.x] = true;
        closedCount++;

        if (cur.y == goal.y && cur.x == goal.x) {
            reconstructPath(parent, start, goal, path);
            return true;
        }

        for (int k = 0; k < 8; ++k) {
            int ny = cur.y + dy[k];
            int nx = cur.x + dx[k];

            if (!inBounds(ny, nx, h, w))
                continue;
            if (!isFree(grid, ny, nx))
                continue;
            if (closed[ny][nx])
                continue;

            Cell par = parent[cur.y][cur.x];
            double tentativeG;
            Cell newParent;

            if (par.y != -1 &&
                lineOfSight(grid, par.y, par.x, ny, nx)) {
                tentativeG = g[par.y][par.x] + distCells(par.y, par.x, ny, nx);
                newParent = par;
            } else {
                tentativeG = g[cur.y][cur.x] + distCells(cur.y, cur.x, ny, nx);
                newParent = Cell{cur.y, cur.x};
            }

            if (tentativeG < g[ny][nx]) {
                g[ny][nx] = tentativeG;
                parent[ny][nx] = newParent;

                Node n;
                n.y = ny;
                n.x = nx;
                n.g = g[ny][nx];
                n.h = heuristic(ny, nx, goal.y, goal.x);
                n.py = newParent.y;
                n.px = newParent.x;
                open.push(n);
            }
        }
    }
    return false;
}

void smoothPath(const std::vector<std::string> &grid,
                const std::vector<Cell> &in,
                std::vector<Cell> &out) {
    out.clear();
    if (in.empty())
        return;

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

double pathLength(const std::vector<Cell> &path) {
    if (path.size() < 2)
        return 0.0;

    double sum = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        sum += distCells(path[i - 1].y, path[i - 1].x,
                         path[i].y, path[i].x);
    }
    return sum;
}
