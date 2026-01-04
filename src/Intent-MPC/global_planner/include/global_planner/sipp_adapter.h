#pragma once

// SIPP 适配器：将 SIPP 的类型和函数封装，供 global_planner 使用
// 注意：SIPP 的原始代码在 sipp_vendor/SIPP/SIPP/源.cpp 中

#include <vector>
#include <utility>
#include <unordered_map>
#include <functional>

// 使用 std 命名空间（与 SIPP 原始代码一致）
using namespace std;

// SIPP 类型定义（从 源.cpp 中提取）
struct Interval {
    int start;
    int end;
    Interval(int s, int e) : start(s), end(e) {}
    bool operator==(const Interval& other) const {
        return start == other.start && end == other.end;
    }
};

namespace std {
    template<>
    struct hash<Interval> {
        size_t operator()(const Interval& interval) const {
            return hash<int>()(interval.start) ^ (hash<int>()(interval.end) << 1);
        }
    };
}

struct Grid {
    int width;
    int height;
    vector<vector<bool>> static_obstacles;
    
    Grid(int w, int h) : width(w), height(h) {
        static_obstacles.resize(w, vector<bool>(h, false));
    }
    
    bool is_valid(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height && !static_obstacles[x][y];
    }
};

struct DynamicObstacle {
    int x, y;
    int start_time;
    int end_time;
    
    DynamicObstacle(int x, int y, int start, int end)
        : x(x), y(y), start_time(start), end_time(end) {}
};

// SIPP 函数声明
// 实际实现在 sipp_vendor 库中，通过链接获取
// 注意：由于 SIPP 函数在 C++ 源文件中，符号会自动导出
extern vector<pair<int, int>> sipp(
    const Grid& grid,
    pair<int, int> start,
    pair<int, int> goal,
    const vector<DynamicObstacle>& obstacles,
    int max_time);

