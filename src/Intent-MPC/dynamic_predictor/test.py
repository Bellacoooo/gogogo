import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import numpy as np

# 清除之前的设置
plt.rcParams.update(plt.rcParamsDefault)

# 强制设置中文字体为首选
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = [
    'WenQuanYi Micro Hei',      # 文泉驿微米黑
    'WenQuanYi Zen Hei',        # 文泉驿正黑
    'Noto Sans CJK JP',         # 思源黑体日本变体（也能显示中文）
    'DejaVu Sans',              # 回退英文字体
    'Bitstream Vera Sans',
    'Computer Modern Sans Serif'
]
plt.rcParams['axes.unicode_minus'] = False

print("当前字体设置:", plt.rcParams['font.sans-serif'])

# 测试中文显示
plt.figure(figsize=(10, 6))
x = np.arange(0, 5, 0.1)
y_pred = np.sin(x)
y_true = np.cos(x)

plt.plot(x, y_pred, 'r--', label='预测轨迹', linewidth=2)
plt.plot(x, y_true, 'g-', label='实际轨迹', linewidth=2)

plt.title('无人机轨迹预测对比分析', fontsize=14, pad=20)
plt.xlabel('时间 (秒)', fontsize=12)
plt.ylabel('位置坐标 (米)', fontsize=12)
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()