
%注意：有三类奇异点
% 1.如果总推力为零，平移动力学不再约束姿态，R_d不是由平坦输出唯一决定的。——这也是算法特性，不要解决。
% 比如 Mellinger 的微分平坦性求角速度公式中 hω 就含有 m/u1，因此 u1=0 时必然退化。
% 
% 2.第二类是 yaw 构造奇异。——我们不打算解决这个问题，我们认为这是算法特性。
% 例如：定义x_C= [cos(yaw_r);sin(yaw_r);0],期望z轴 z_B x x_C 不能为0， 否则分母为零。z_B x x_C !=0时才能唯一确定 R。
% 又例如tal，如果用ψ=atan2(^1_y,b^1_x),那么当 b1的水平投影为零时，yaw 不可定义。
% 还有其他使用y_C= [-sin(yaw_r);cos(yaw_r);0]来定义yaw方向的，则在另一个方向有奇异性。
% Tal 和 Karaman 的 S 矩阵中有r_ψ^Tr_ψ这样的分母；当 b_x在水平面的投影消失时，yaw rate 映射退化。
% 由 hairy ball theorem，S^2 上任意连续切向量场至少存在一个零点。因此，在四旋翼平坦重构中，若用全局连续规则为每个推力方向 z_b \in S^2 选取一个垂直机体系轴 x_b，则该选择规则至少在某个推力方向处发生退化。
% 
% 3.第三类是轨迹光滑性不足或动力学不可行。若要计算角速度，需要位置至少三阶可导；——这个在我们这里似乎没问题？因为都是解析的基准曲线。
% 若要计算角加速度，需要位置至少四阶可导，yaw 至少二阶可导。Tal 和 Karaman 明确要求 x_ref ∈ C^4、ψ_ref ∈ C^2。


### Hirsch 1.6 定理、Hairy Ball Theorem 与四旋翼 yaw 重构奇异

Hirsch 第五章的 Theorem 1.6 给出的是 degree 的两个基本性质：同伦映射具有相同的 degree；若边界映射可以扩张到高一维紧流形内部，则该边界映射的 degree 为零。其后 Hirsch 用该定理解释 hairy ball theorem：偶数维球面 $S^{2n}$ 上任意连续切向量场至少存在一个零点。对 $S^2$ 而言，这句话可写为：对任意连续切向量场 $X:S^2\to TS^2$，至少存在一个点 $p\in S^2$，使得 $X(p)=0_{T_pS^2}$。

在四旋翼微分平坦重构中，平动加速度首先确定期望推力方向，也就是机体 $z$ 轴方向 $z_b\in S^2$。为了构造完整姿态 $R_d=[x_b\ y_b\ z_b]\in SO(3)$，还需要为每个 $z_b$ 选取一个垂直于 $z_b$ 的单位方向 $x_b\in T_{z_b}S^2$，满足 $x_b^\top z_b=0$ 和 $\|x_b\|=1$。因此，yaw 重构本质上是在球面 $S^2$ 的每个点 $z_b$ 上选取一个单位切向量 $x_b(z_b)$。Hairy ball theorem 表明，这种全局连续选择规则至少在某个推力方向处发生退化。

常见的投影 yaw 构造为

$$
c_\psi=(\cos\psi,\sin\psi,0)^\top,\qquad
\tilde{x}_b=(I-z_bz_b^\top)c_\psi,\qquad
x_b=\frac{\tilde{x}_b}{\|\tilde{x}_b\|},\qquad
y_b=z_b\times x_b .
$$

该构造的退化条件为

$$
\tilde{x}_b=0
\quad\Longleftrightarrow\quad
(I-z_bz_b^\top)c_\psi=0
\quad\Longleftrightarrow\quad
z_b=\pm c_\psi .
$$

也就是说，当 yaw 参考方向 $c_\psi$ 与推力方向 $z_b$ 平行时，$c_\psi$ 在垂直于 $z_b$ 的平面上的投影长度为零，归一化公式失效。这里姿态本身仍然可以被选取，因为 $z_b$ 已经确定，而绕 $z_b$ 的剩余相位仍是一维自由度。给定任意单位向量 $x_0\in T_{z_b}S^2$，所有可选补轴可以写为

$$
x_b(\chi)=x_0\cos\chi+(z_b\times x_0)\sin\chi,\qquad
y_b(\chi)=z_b\times x_b(\chi),
$$

并由此得到

$$
R_d(\chi)=[x_b(\chi)\ y_b(\chi)\ z_b].
$$

因此，退化点处的选择本质上是选择剩余相位 $\chi$。该相位对平坦重构高阶量的影响为：$\chi$ 影响参考姿态 $R_d$，$\dot\chi$ 影响参考角速度 $\Omega_d$，$\ddot\chi$ 影响参考角加速度 $\dot\Omega_d$。若两种补轴选择满足

$$
R_2(t)=R_1(t)R_z(\chi(t)),
$$

并采用机体系角速度定义 $\widehat{\Omega}=R^\top\dot R$，则有

$$
\Omega_2=R_z(\chi)^\top\Omega_1+\dot\chi e_3,
$$

以及

$$
\dot\Omega_2
=
R_z(\chi)^\top\dot\Omega_1
-\dot\chi\, e_3\times\bigl(R_z(\chi)^\top\Omega_1\bigr)
+\ddot\chi e_3 .
$$

这说明：退化点处任意补一个方向只能给出某一时刻的姿态；若还需要平滑的参考角速度和参考角加速度，必须同时指定剩余相位 $\chi(t)$ 的一阶和二阶时间变化。硬切换相位会导致角速度尖峰和角加速度尖峰；平滑延续相位可以减小这种尖峰。




### PX4 `bodyzToAttitude` 中的 yaw 补轴构造

设世界系竖直单位向量为 $e_3=(0,0,1)^\top$，yaw 给出的水平前向方向为

$$
c_\psi=(\cos\psi,\sin\psi,0)^\top .
$$

PX4 代码先构造与 $c_\psi$ 正交的水平侧向方向

$$
y_\psi=e_3\times c_\psi=(-\sin\psi,\cos\psi,0)^\top .
$$

由于 `thrustToAttitude` 调用 `bodyzToAttitude(-thr_sp, yaw_sp)`，期望机体 $z$ 轴方向定义为

$$
z_b=\frac{-\mathrm{thr}_{sp}}{\|\mathrm{thr}_{sp}\|}.
$$

当 $\|\mathrm{thr}_{sp}\|$ 接近零时，PX4 取安全水平默认值

$$
z_b=e_3 .
$$

在普通分支 $|z_{b,3}|\ge \varepsilon$ 中，代码先取

$$
\bar x_b=y_\psi\times z_b .
$$

若 $z_{b,3}<0$，代码执行 $\bar x_b\leftarrow -\bar x_b$。这可以统一写成

$$
\bar x_b=\eta(z_b)(y_\psi\times z_b),
\qquad
\eta(z_b)=
\begin{cases}
1, & z_{b,3}\ge 0,\\
-1, & z_{b,3}<0 .
\end{cases}
$$

随后归一化得到

$$
x_b=\frac{\bar x_b}{\|\bar x_b\|},
\qquad
y_b=z_b\times x_b .
$$

最终姿态矩阵为

$$
R_{\mathrm{sp}}=[x_b\ y_b\ z_b]\in SO(3).
$$

这个构造的几何意义可以从恒等式看出：

$$
y_\psi\times z_b=z_{b,3}c_\psi-(c_\psi^\top z_b)e_3 .
$$

加入 $\eta(z_b)$ 之后，其水平投影满足

$$
\Pi_{xy}\bar x_b=|z_{b,3}|c_\psi,
\qquad
\Pi_{xy}=\operatorname{diag}(1,1,0).
$$

也就是说，PX4 通过 $z_{b,3}<0$ 时对 $x_b$ 取负号，使机体 $x$ 轴的水平投影在倒置状态下仍指向 yaw 给定的前向方向 $c_\psi$。从姿态合法性角度看，这个取负号属于绕 $z_b$ 的相位分支选择；从 PX4 yaw 语义角度看，它固定了倒置状态下的“机头朝前”分支。

当 $|z_{b,3}|<\varepsilon$ 时，代码进入水平推力分支，直接取

$$
x_b=e_3,
\qquad
y_b=z_b\times e_3 .
$$

此时 yaw 水平参考方向的作用由补轴规则替代。该分支本质上是在 yaw 构造退化附近人为选择一个绕 $z_b$ 的剩余相位。

因此，PX4 的做法可以概括为：先由推力方向确定 $z_b$，再用 yaw 水平侧向方向 $y_\psi$ 与 $z_b$ 做叉乘得到 $x_b$，并在倒置和水平推力附近通过相位分支选择构造合法姿态矩阵 $R_{\mathrm{sp}}\in SO(3)$。