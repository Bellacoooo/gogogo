#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

// 核心函数：在 OccupancyGrid 上打一个“高斯印章”
void stampGaussian(
    nav_msgs::OccupancyGrid& map,
    const Eigen::Vector2d& mu,
    const Eigen::Matrix2d& Sigma,
    double combined_weight
) {
    const double res = map.info.resolution;
    const unsigned int width  = map.info.width;
    const unsigned int height = map.info.height;

    if (width == 0 || height == 0 || res <= 0.0) {
        std::cerr << "[stampGaussian] invalid map parameters." << std::endl;
        return;
    }

    // 预备一个 double 缓冲区存风险值（与 map.data 同尺寸）
    std::vector<double> risk_buffer(width * height, 0.0);

    // a. 计算包围盒：用对角线近似标准差
    double var_x = Sigma(0, 0);
    double var_y = Sigma(1, 1);
    if (var_x <= 1e-8) var_x = 1e-8;
    if (var_y <= 1e-8) var_y = 1e-8;
    double std_x = std::sqrt(var_x);
    double std_y = std::sqrt(var_y);

    const double k = 3.0; // 3σ 范围
    double min_x_world = mu(0) - k * std_x;
    double max_x_world = mu(0) + k * std_x;
    double min_y_world = mu(1) - k * std_y;
    double max_y_world = mu(1) + k * std_y;

    // 地图原点（世界坐标）
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;

    // 世界坐标 -> 栅格索引
    auto worldToIndex = [&](double wx, double wy, int& ix, int& iy) {
        ix = static_cast<int>(std::floor((wx - origin_x) / res));
        iy = static_cast<int>(std::floor((wy - origin_y) / res));
    };

    int min_ix, min_iy, max_ix, max_iy;
    worldToIndex(min_x_world, min_y_world, min_ix, min_iy);
    worldToIndex(max_x_world, max_y_world, max_ix, max_iy);

    // 边界裁剪
    if (min_ix < 0) min_ix = 0;
    if (min_iy < 0) min_iy = 0;
    if (max_ix >= static_cast<int>(width))  max_ix = static_cast<int>(width)  - 1;
    if (max_iy >= static_cast<int>(height)) max_iy = static_cast<int>(height) - 1;

    if (min_ix > max_ix || min_iy > max_iy) {
        // 包围盒完全在地图外
        return;
    }

    // b. 求逆矩阵（防止奇异）
    Eigen::Matrix2d Sigma_safe = Sigma;
    Sigma_safe(0, 0) = std::max(Sigma_safe(0, 0), 1e-8);
    Sigma_safe(1, 1) = std::max(Sigma_safe(1, 1), 1e-8);
    Eigen::Matrix2d Sigma_inv = Sigma_safe.inverse();

    // c/d. 遍历包围盒，计算马氏距离和风险值
    for (int iy = min_iy; iy <= max_iy; ++iy) {
        for (int ix = min_ix; ix <= max_ix; ++ix) {
            // 栅格中心的世界坐标
            double wx = origin_x + (static_cast<double>(ix) + 0.5) * res;
            double wy = origin_y + (static_cast<double>(iy) + 0.5) * res;

            Eigen::Vector2d n(wx, wy);
            Eigen::Vector2d diff = n - mu;

            double mah_dist_sq = diff.transpose() * Sigma_inv * diff;
            double risk = std::exp(-0.5 * mah_dist_sq);
            double weighted_risk = combined_weight * risk;

            std::size_t idx = static_cast<std::size_t>(iy) * width + static_cast<std::size_t>(ix);
            risk_buffer[idx] += weighted_risk;
        }
    }

    // 将 double 风险值映射到 [0,100] 的 int8，占用率栅格
    map.data.resize(width * height);
    for (std::size_t i = 0; i < risk_buffer.size(); ++i) {
        double v = risk_buffer[i];
        if (v < 0.0) v = 0.0;
        // 正常高斯下 v<=1，这里简单压到 0~100
        if (v > 1.0) v = 1.0;
        map.data[i] = static_cast<int8_t>(std::round(100.0 * v));
    }
}

// 简单单元测试主函数：构建一个小地图，打高斯印章并打印结果
int main(int argc, char** argv) {
    // 构造一个 21x21、分辨率 0.1、原点(0,0) 的 OccupancyGrid
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 0.1;
    grid.info.width  = 21;
    grid.info.height = 21;
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.position.z = 0.0;
    grid.data.assign(grid.info.width * grid.info.height, 0);

    // 情形 1：圆形高斯（Sigma = I * 0.2^2），中心在地图中间
    Eigen::Vector2d mu1(1.0, 1.0);          // 约在 (10,10) 栅格附近
    Eigen::Matrix2d Sigma1 = Eigen::Matrix2d::Zero();
    Sigma1(0, 0) = 0.2 * 0.2;
    Sigma1(1, 1) = 0.2 * 0.2;

    stampGaussian(grid, mu1, Sigma1, 1.0);

    std::cout << "=== Test 1: Isotropic Gaussian (circle-like) ===" << std::endl;
    // 打印地图，y 从上往下，x 从左到右
    for (int iy = static_cast<int>(grid.info.height) - 1; iy >= 0; --iy) {
        for (unsigned int ix = 0; ix < grid.info.width; ++ix) {
            std::size_t idx = static_cast<std::size_t>(iy) * grid.info.width + ix;
            int v = static_cast<int>(grid.data[idx]);
            std::cout << std::setw(4) << v;
        }
        std::cout << std::endl;
    }

    // 情形 2：椭圆高斯（在 x 方向更“宽”），重置地图
    grid.data.assign(grid.info.width * grid.info.height, 0);

    Eigen::Vector2d mu2(1.0, 1.0);
    Eigen::Matrix2d Sigma2 = Eigen::Matrix2d::Zero();
    Sigma2(0, 0) = 0.4 * 0.4;   // x 方向方差更大
    Sigma2(1, 1) = 0.1 * 0.1;   // y 方向方差更小

    stampGaussian(grid, mu2, Sigma2, 1.0);

    std::cout << "\n=== Test 2: Anisotropic Gaussian (ellipse-like) ===" << std::endl;
    for (int iy = static_cast<int>(grid.info.height) - 1; iy >= 0; --iy) {
        for (unsigned int ix = 0; ix < grid.info.width; ++ix) {
            std::size_t idx = static_cast<std::size_t>(iy) * grid.info.width + ix;
            int v = static_cast<int>(grid.data[idx]);
            std::cout << std::setw(4) << v;
        }
        std::cout << std::endl;
    }

    return 0;
}


