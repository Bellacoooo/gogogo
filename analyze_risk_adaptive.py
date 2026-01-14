#!/usr/bin/env python3
"""
风险自适应椭球参数分析工具

功能：
1. 可视化风险裕量 s 随距离、速度的变化
2. 展示各向异性效果（椭球形状）
3. 验证参数设置是否合理

使用方法：
    python3 analyze_risk_adaptive.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches

# ========== 参数配置（从 YAML 文件读取或手动设置） ==========
# 风险裕量计算参数
S0 = 0.15       # 基线膨胀量 (m)
ALPHA = 0.4     # closing speed 系数
BETA = 0.6      # TTC 指数项系数 (m)
TAU = 1.5       # TTC 衰减时间常数 (s)
S_MIN = 0.0     # 最小膨胀量 (m)
S_MAX = 1.2     # 最大膨胀量 (m)

# 各向异性参数
KAPPA = 0.35    # 各向异性强度 [0,1)

# 基线椭球半轴（示例：障碍物尺寸 + 基础安全距离）
A0 = 0.5  # m
B0 = 0.3  # m
C0 = 0.8  # m


def compute_risk_margin(d, vc):
    """
    计算风险裕量 s
    
    参数:
        d: 距离 (m)
        vc: closing speed (m/s)
    
    返回:
        s: 风险裕量 (m)
    """
    ttc = d / (vc + 1e-6)
    s_raw = S0 + ALPHA * vc + BETA * np.exp(-ttc / TAU)
    s = np.clip(s_raw, S_MIN, S_MAX)
    return s


def plot_risk_margin_vs_distance():
    """
    绘制风险裕量 s 随距离 d 的变化（不同 closing speed）
    """
    distances = np.linspace(0.5, 10.0, 100)  # 距离范围：0.5-10m
    vc_values = [0.0, 0.5, 1.0, 1.5, 2.0]    # 不同的 closing speed
    
    plt.figure(figsize=(10, 6))
    for vc in vc_values:
        s_values = [compute_risk_margin(d, vc) for d in distances]
        plt.plot(distances, s_values, label=f'vc = {vc} m/s', linewidth=2)
    
    plt.axhline(y=S_MIN, color='r', linestyle='--', alpha=0.5, label=f's_min = {S_MIN}')
    plt.axhline(y=S_MAX, color='r', linestyle='--', alpha=0.5, label=f's_max = {S_MAX}')
    
    plt.xlabel('距离 d (m)', fontsize=12)
    plt.ylabel('风险裕量 s (m)', fontsize=12)
    plt.title('风险裕量随距离变化（不同 closing speed）', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('risk_margin_vs_distance.png', dpi=150)
    print("✓ 已保存: risk_margin_vs_distance.png")


def plot_risk_margin_heatmap():
    """
    绘制风险裕量 s 的热力图（距离 d vs closing speed vc）
    """
    distances = np.linspace(0.5, 10.0, 50)
    vc_values = np.linspace(0.0, 3.0, 50)
    
    D, VC = np.meshgrid(distances, vc_values)
    S = np.zeros_like(D)
    
    for i in range(len(vc_values)):
        for j in range(len(distances)):
            S[i, j] = compute_risk_margin(D[i, j], VC[i, j])
    
    plt.figure(figsize=(10, 6))
    contour = plt.contourf(D, VC, S, levels=20, cmap='RdYlGn_r')
    plt.colorbar(contour, label='风险裕量 s (m)')
    
    plt.xlabel('距离 d (m)', fontsize=12)
    plt.ylabel('Closing Speed vc (m/s)', fontsize=12)
    plt.title('风险裕量热力图', fontsize=14)
    plt.tight_layout()
    plt.savefig('risk_margin_heatmap.png', dpi=150)
    print("✓ 已保存: risk_margin_heatmap.png")


def plot_ellipsoid_shape():
    """
    可视化椭球形状（不同风险裕量 s 下的各向异性效果）
    """
    s_values = [0.0, 0.3, 0.6, 0.9]  # 不同的风险裕量
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    axes = axes.flatten()
    
    for idx, s in enumerate(s_values):
        ax = axes[idx]
        
        # 计算各向异性膨胀
        delta_parallel = s * (1.0 + KAPPA)
        delta_perp = s * (1.0 - KAPPA)
        
        a = A0 + delta_parallel
        b = B0 + delta_perp
        
        # 绘制基线椭圆（虚线）
        ellipse_base = patches.Ellipse((0, 0), 2*A0, 2*B0, 
                                       fill=False, edgecolor='blue', 
                                       linestyle='--', linewidth=2, 
                                       label='基线椭球')
        ax.add_patch(ellipse_base)
        
        # 绘制风险自适应椭圆（实线）
        ellipse_risk = patches.Ellipse((0, 0), 2*a, 2*b, 
                                       fill=True, facecolor='red', 
                                       alpha=0.3, edgecolor='red', 
                                       linewidth=2, 
                                       label='风险自适应')
        ax.add_patch(ellipse_risk)
        
        # 添加箭头表示长轴方向
        ax.arrow(0, 0, a*0.8, 0, head_width=0.1, head_length=0.1, 
                fc='black', ec='black', linewidth=2)
        ax.text(a*0.9, 0.2, 'a (长轴)', fontsize=10, ha='center')
        ax.arrow(0, 0, 0, b*0.8, head_width=0.1, head_length=0.1, 
                fc='black', ec='black', linewidth=2)
        ax.text(0.2, b*0.9, 'b (短轴)', fontsize=10, ha='left')
        
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x (m)', fontsize=10)
        ax.set_ylabel('y (m)', fontsize=10)
        ax.set_title(f's = {s:.1f}m → a={a:.2f}m, b={b:.2f}m', fontsize=12)
        ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('ellipsoid_shape_comparison.png', dpi=150)
    print("✓ 已保存: ellipsoid_shape_comparison.png")


def plot_anisotropy_effect():
    """
    展示不同各向异性强度 κ 的效果
    """
    kappa_values = [0.0, 0.2, 0.4, 0.6]
    s = 0.6  # 固定风险裕量
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    axes = axes.flatten()
    
    for idx, kappa in enumerate(kappa_values):
        ax = axes[idx]
        
        delta_parallel = s * (1.0 + kappa)
        delta_perp = s * (1.0 - kappa)
        
        a = A0 + delta_parallel
        b = B0 + delta_perp
        
        # 基线椭圆
        ellipse_base = patches.Ellipse((0, 0), 2*A0, 2*B0, 
                                       fill=False, edgecolor='blue', 
                                       linestyle='--', linewidth=2, 
                                       label='基线')
        ax.add_patch(ellipse_base)
        
        # 各向异性椭圆
        ellipse_aniso = patches.Ellipse((0, 0), 2*a, 2*b, 
                                        fill=True, facecolor='orange', 
                                        alpha=0.4, edgecolor='orange', 
                                        linewidth=2, 
                                        label=f'κ={kappa}')
        ax.add_patch(ellipse_aniso)
        
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x (m)', fontsize=10)
        ax.set_ylabel('y (m)', fontsize=10)
        ax.set_title(f'κ = {kappa} → a/b = {a/b:.2f}', fontsize=12)
        ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('anisotropy_effect.png', dpi=150)
    print("✓ 已保存: anisotropy_effect.png")


def plot_ttc_sensitivity():
    """
    展示 TTC 参数 τ 的影响
    """
    tau_values = [1.0, 1.5, 2.0, 2.5]
    distances = np.linspace(0.5, 10.0, 100)
    vc = 1.5  # 固定 closing speed
    
    plt.figure(figsize=(10, 6))
    for tau in tau_values:
        s_values = []
        for d in distances:
            ttc = d / (vc + 1e-6)
            s_raw = S0 + ALPHA * vc + BETA * np.exp(-ttc / tau)
            s = np.clip(s_raw, S_MIN, S_MAX)
            s_values.append(s)
        plt.plot(distances, s_values, label=f'τ = {tau} s', linewidth=2)
    
    plt.xlabel('距离 d (m)', fontsize=12)
    plt.ylabel('风险裕量 s (m)', fontsize=12)
    plt.title(f'TTC 衰减时间常数 τ 的影响（vc = {vc} m/s）', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('ttc_sensitivity.png', dpi=150)
    print("✓ 已保存: ttc_sensitivity.png")


def print_parameter_summary():
    """
    打印当前参数配置摘要
    """
    print("\n" + "="*60)
    print("当前风险自适应椭球参数配置".center(60))
    print("="*60)
    print(f"  基线膨胀量 (s0):          {S0:.2f} m")
    print(f"  Closing speed 系数 (α):   {ALPHA:.2f}")
    print(f"  TTC 指数系数 (β):         {BETA:.2f} m")
    print(f"  TTC 衰减时间 (τ):         {TAU:.2f} s")
    print(f"  最小膨胀量 (s_min):       {S_MIN:.2f} m")
    print(f"  最大膨胀量 (s_max):       {S_MAX:.2f} m")
    print(f"  各向异性强度 (κ):         {KAPPA:.2f}")
    print(f"  基线椭球半轴 (a0,b0,c0):  ({A0:.2f}, {B0:.2f}, {C0:.2f}) m")
    print("="*60)
    
    # 示例场景分析
    print("\n示例场景分析:")
    scenarios = [
        ("远距离、低速", 8.0, 0.3),
        ("中距离、中速", 4.0, 1.0),
        ("近距离、高速", 2.0, 2.0),
        ("极近、极高速", 1.0, 3.0),
    ]
    
    print(f"{'场景':<15} {'距离(m)':<10} {'vc(m/s)':<10} {'TTC(s)':<10} {'s(m)':<10}")
    print("-" * 60)
    for name, d, vc in scenarios:
        s = compute_risk_margin(d, vc)
        ttc = d / (vc + 1e-6)
        print(f"{name:<15} {d:<10.1f} {vc:<10.1f} {ttc:<10.2f} {s:<10.2f}")
    print("="*60 + "\n")


def main():
    """
    主函数：运行所有分析和可视化
    """
    print("\n🚀 风险自适应椭球参数分析工具")
    print_parameter_summary()
    
    print("正在生成可视化图表...")
    plot_risk_margin_vs_distance()
    plot_risk_margin_heatmap()
    plot_ellipsoid_shape()
    plot_anisotropy_effect()
    plot_ttc_sensitivity()
    
    print("\n✅ 所有图表已生成！")
    print("\n生成的文件：")
    print("  1. risk_margin_vs_distance.png   - 风险裕量随距离变化")
    print("  2. risk_margin_heatmap.png       - 风险裕量热力图")
    print("  3. ellipsoid_shape_comparison.png - 椭球形状对比")
    print("  4. anisotropy_effect.png          - 各向异性效果")
    print("  5. ttc_sensitivity.png            - TTC 参数敏感性")
    print("\n💡 提示：使用图像查看器打开这些 PNG 文件进行查看")
    print("       或运行：eog *.png")


if __name__ == "__main__":
    # 检查依赖
    try:
        import matplotlib
        import numpy
    except ImportError as e:
        print(f"❌ 缺少依赖库: {e}")
        print("请安装: pip3 install matplotlib numpy")
        exit(1)
    
    main()

