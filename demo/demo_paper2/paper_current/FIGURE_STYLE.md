# 当前论文绘图风格

## 正式作图入口（2026-10-27）

本文图件统一从本目录 `run.py plot` 及 `scripts/` 重现，之后将输出复制到论文目录。论文目录的旧脚本不再独立维护。
Fig. 1 的样式参照 Fig. 3 的 `plot_box_sliding_combined_paper_figure.py`：同一蓝色/橙色、空心圆、虚线网格、透明框内图例及轴外左上字母。Fig. 1(c) 右轴速度用点线，数值与归一化不变。
Fig. 4(b) 为 2.4 × 2.25 inch，在当前 LaTeX 插入宽度下高度约为 (a) 的 1.5 倍。
Fig. 2 用 `scripts/crop_scene_pdf.py` 从原始场景生成 `sliding_box_cropped.pdf`，保留矢量图形与原始文件。

记录日期：2026-10-13。用户指定以上一篇接触模型论文为绘图基准，不使用 Nature 作图技能。本文件记录经源代码核对的设置，供后续本论文绘图和修图直接复用。

## 参考来源

- 论文：`D:/papers/sire_contact_model/sire_contact_model_final.tex`。
- 作图代码目录：`D:/code/sire/demo/demo_paper/`。
- `data_plot.ipynb`：刚度范围、性能比较及 `dog_body_height.pdf`，是汇总作图的主要参考。
- `sire/bouncing_ball/demo_bouncing_ball.ipynb`：球体高度轨迹；代码单元 1 导出论文引用的 `sphere_free_fall_height_data_sire.pdf`。
- `sire/bouncing_box/demo_bouncing_box.ipynb`：方块高度轨迹；代码单元 1 导出论文引用的 `box_free_fall_height_data_sire.pdf`。
- `sire/plot_figures/demo_init_penetration_eliminate.ipynb`：初始穿透消除曲线和 inset 的样式参考。

Notebook 中含探索性代码和未实际导出的图。优先选择其 `savefig` 输出被上一篇最终论文引用的代码单元；不同图的局部参数确有差异，不能把某一个单元的设置误认为整篇论文的统一硬约束。

## SciencePlots 基础

```python
import matplotlib.pyplot as plt
import scienceplots

with plt.style.context(['science', 'ieee']):
    # 按参考图设置实际尺寸、线型、颜色和局部字号。
    ...
```

需要网格的图使用 `['science', 'ieee', 'grid']`，或沿用参考代码的 `grid(True, linestyle='--', alpha=0.3)`；不是每张图都需要网格。

本地已安装 SciencePlots 的 `science + ieee` 默认值经读取确认为：serif/Times 字体、基础字号 8 pt、基础线宽 1.0 pt、轴框线宽 0.5 pt、向内刻度、上侧和右侧刻度开启、默认画布 3.3 × 2.5 inch、LaTeX 文本开启。这些是当前安装版本的默认值，未来环境变化时应重新读取，最终以参考代码和实际 PDF 效果为准。

不要自动覆盖为 Nature 风格的 Arial/sans-serif、调色板、无上右刻度等设置。参考代码主要使用原生 LaTeX 文本；仅在环境确实无法使用 LaTeX 时考虑 `no-latex`，并记录差异及核对字体。

## 颜色

参考代码反复使用以下有序配色：

```python
scientific_colors = [
    '#2E86AB', '#A23B72', '#F18F01', '#C73E1D', '#3B1F2B',
    '#6B8E23', '#8B4513', '#4682B4', '#D2691E', '#2F4F4F',
]
```

前三色依次为蓝、紫、橙。在上一篇的求解器对比图中，Our solver/CSA、MuJoCo、Drake 分别使用前三色。当前论文如有不同的方法或信号，按其语义安排颜色，并在各图中保持一致；不能因颜色相同而误用上一篇的方法名称。黑色可用于解析参考、零线或括号；灰色可用于背景辅助曲线。

## 线型、线宽及 marker

主参考的线型顺序：

```python
linestyles = ['-', ':', '--', '-.']
```

球体 notebook 的部分其他代码单元使用 `['-', '--', '-.', ':']`。复用特定图时沿用该图自己的顺序；新图优先采用主参考顺序，明确指定每条曲线的颜色和线型，避免默认 cycle 暗中改变映射。

- 高度时间序列通常用线宽 1.0 pt。
- 刚度范围图使用 0.8 pt；其他对比和 inset 代码常见 0.5--0.6 pt。
- marker 常见 `o`、`v`、`^`，尺寸 1--2 pt；用 marker 区分类别时参考对应代码。
- `markevery` 只控制 marker 的显示密度，保留全部折线数据；原代码可见 10、20 等取值。
- 新图按最终单栏或双栏显示效果调整细节，不把所有图强行统一成相同大小、线宽或 marker 数量。

## 尺寸、字体和导出

上一篇代码有 SciencePlots 默认画布、4.2 × 3.2 inch 和 4 × 4 inch 等局部设置；轴标签常见 10 pt，图例存在默认字号或 8 pt 的局部设置。这些代码尺寸与最终论文插入尺寸不同，应结合 LaTeX 的实际缩放检查可读性。

- 单栏图使用 `figure` 和 `width=\columnwidth`；跨双栏图使用 `figure*` 和 `width=\textwidth`。
- 不为满足某个通用模板而改字体、颜色或线型；允许为当前栏宽修正文字重叠和裁切。
- 优先导出矢量 PDF。参考代码通常使用 `bbox_inches='tight'`，轨迹与刚度图导出 dpi=600，部分性能图 dpi=300。
- 保留可复现绘图代码和源数据。排版或样式修改不改变实验数据、单位、统计定义、平滑方式、图像内容或证据边界。
- 导出后编译当前论文、渲染相关页面并检查字号、图例、线型区别、裁切和布局。检查服务于上述参考风格，不引入 Nature 技能的额外风格门槛。

## 已确定的当前布局

Fig. 4 仅放真机运动 montage，横跨双栏。PPO reward 曲线和速度响应图分别作为独立单栏图，有各自图号和图注；速度图内部可使用 a/b/c 标识三个响应面板。后续图号随论文修改动态更新，独立图和单栏要求保持有效，除非用户另行更改。
