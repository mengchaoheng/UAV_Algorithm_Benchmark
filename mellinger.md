# Mellinger 微分平坦性推导的修正版整理

## 预备知识：归一化向量的一阶与二阶导数

设

$$
b=\frac{v}{\|v\|},
\qquad
\rho=\|v\|>0.
$$

于是

$$
v=\rho b.
$$

先求 $\rho$ 的导数。由 $\rho=\sqrt{v^Tv}$，有 $\dot\rho=\frac{v^T\dot v}{\|v\|}$。又因为 $v=\rho b$，所以 $v/\|v\|=b$，因此 $\dot\rho=b^T\dot v$。

对 $v=\rho b$ 求导：

$$
\dot v=\dot\rho\,b+\rho\dot b.
$$

移项并除以 $\rho$：

$$
\dot b
=
\frac{\dot v-\dot\rho\,b}{\rho}.
$$

代入 $\dot\rho=b^T\dot v$：

$$
\dot b
=
\frac{\dot v-b(b^T\dot v)}{\rho}
=
\frac{(I-bb^T)\dot v}{\rho}.
$$

因此

$$
\dot b
=
\frac{(I-bb^T)\dot v}{\|v\|}.
$$

接着求二阶导数。由

$$
v=\rho b
$$

再次求导：

$$
\ddot v
=
\ddot\rho\,b+2\dot\rho\,\dot b+\rho\ddot b.
$$

因为 $b^Tb=1$，对时间求导有 $b^T\dot b=0$。再次求导有

$$
\dot b^T\dot b+b^T\ddot b=0,
$$

即

$$
b^T\ddot b=-\|\dot b\|^2.
$$

对 $\ddot v=\ddot\rho\,b+2\dot\rho\,\dot b+\rho\ddot b$ 左乘 $b^T$：

$$
b^T\ddot v
=
\ddot\rho+\rho b^T\ddot b.
$$

代入 $b^T\ddot b=-\|\dot b\|^2$，得到 $\ddot\rho=b^T\ddot v+\rho\|\dot b\|^2$。

再回到

$$
\rho\ddot b
=
\ddot v-\ddot\rho\,b-2\dot\rho\,\dot b.
$$

代入 $\dot\rho=b^T\dot v$ 和 $\ddot\rho=b^T\ddot v+\rho\|\dot b\|^2$：

$$
\rho\ddot b
=
(I-bb^T)\ddot v
-
2(b^T\dot v)\dot b
-
\rho\|\dot b\|^2b.
$$

除以 $\rho$，得到归一化向量的二阶导数公式：

$$
\ddot b
=
\frac{(I-bb^T)\ddot v}{\|v\|}
-
2\frac{b^T\dot v}{\|v\|}\dot b
-
\|\dot b\|^2b.
$$

在 Mellinger 的推导中，取

$$
v=A=a+gz_W,
\qquad
b=z_B,
\qquad
\dot v=j,
\qquad
\ddot v=s,
\qquad
\rho=\lambda=\|A\|.
$$

因此

$$
\dot z_B
=
\frac{(I-z_Bz_B^T)j}{\lambda}.
$$

以及

$$
\ddot z_B
=
\frac{(I-z_Bz_B^T)s}{\lambda}
-
2\frac{z_B^Tj}{\lambda}\dot z_B
-
\|\dot z_B\|^2z_B.
$$

这两个公式分别说明：jerk 决定推力方向的一阶变化，snap 决定推力方向的二阶变化；同时，所有公式都要求 $\lambda=\|a+gz_W\|>0$。

## 1. 坐标与符号约定

采用 Mellinger 的世界系和机体系约定：

- 世界系为 $W=\{x_W,y_W,z_W\}$，其中 $z_W$ 向上；
- 机体系为 $B=\{x_B,y_B,z_B\}$，其中 $z_B$ 为总推力方向；
- 旋转矩阵为

$$
{}^W R_B = R = [x_B\ y_B\ z_B]\in SO(3),
$$

其中 $x_B,y_B,z_B$ 都用世界系坐标表示。

角速度的几何向量写作

$$
{}^W\omega_{B/W}=p x_B+q y_B+r z_B.
$$

它的机体系坐标为

$$
\Omega = {}^B\omega_{B/W}
=
\begin{bmatrix}
p\\q\\r
\end{bmatrix}.
$$

姿态运动学应写为

$$
\dot R = R\widehat{\Omega}.
$$

刚体 Euler 方程应写为

$$
I\dot\Omega
=
\tau^B-\Omega\times I\Omega,
$$

其中

$$
\tau^B=
\begin{bmatrix}
u_2\\u_3\\u_4
\end{bmatrix}.
$$

注意：Mellinger ICRA 和 thesis 中常写 $\omega_{BW}$，它有时表示几何角速度，有时又作为机体系列向量进入 $\dot R=R\widehat\omega$ 和 Euler 方程。严格写作时，应区分 ${}^W\omega_{B/W}$ 和 $\Omega={}^B\omega_{B/W}$。

## 2. 平动模型与平坦输出

Mellinger 的平动模型为

$$
m\ddot r=-mgz_W+u_1z_B.
$$

平坦输出选为

$$
\sigma=
\begin{bmatrix}
x&y&z&\psi
\end{bmatrix}^T,
$$

其中

$$
r=
\begin{bmatrix}
x&y&z
\end{bmatrix}^T.
$$

定义

$$
a=\ddot r,\qquad j=\overset{...}{r},\qquad s=r^{(4)}.
$$

由平动方程可得

$$
u_1z_B=m(a+gz_W).
$$

令

$$
A=a+gz_W,\qquad \lambda=\|A\|.
$$

因此在 $\lambda\neq0$ 时，

$$
z_B=\frac{A}{\lambda},
\qquad
u_1=m\lambda.
$$

这一步是微分平坦恢复的第一步：位置二阶导数决定总推力方向和总推力大小。

## 3. 参考姿态的构造

给定 yaw 角 $\psi$，定义中间 yaw frame 的 $x_C$ 轴为

$$
x_C=
\begin{bmatrix}
\cos\psi\\
\sin\psi\\
0
\end{bmatrix}.
$$

Mellinger 的姿态构造为

$$
y_B=\frac{z_B\times x_C}{\|z_B\times x_C\|},
\qquad
x_B=y_B\times z_B.
$$

于是

$$
R=[x_B\ y_B\ z_B].
$$

该构造要求

$$
\|z_B\times x_C\|\neq0.
$$

该条件失败时，$x_C$ 与 $z_B$ 平行，yaw 约束与推力方向构造发生图表奇异，$x_B,y_B$ 无法通过该公式唯一且连续地确定。

## 4. $z_B$ 的一阶导数：jerk 如何进入角速度

因为

$$
z_B=\frac{A}{\lambda},
\qquad
A=a+gz_W,
\qquad
\dot A=j,
$$

且

$$
\dot\lambda=z_B^Tj,
$$
====================================

补充说明：这里 $\dot\lambda=z_B^Tj$ 来自向量范数的求导。由 $\lambda=\|A\|=\sqrt{A^TA}$，在 $\lambda>0$ 时有
$$
\dot\lambda
=
\frac{1}{2}(A^TA)^{-1/2}\frac{d}{dt}(A^TA)
=
\frac{A^T\dot A}{\|A\|}.
$$

又因为 $A=a+gz_W$，其中 $z_W$ 是世界系常向量，所以 $\dot A=\dot a=j$。代入上式得
$$
\dot\lambda
=
\frac{A^Tj}{\|A\|}.
$$

由 $z_B=A/\|A\|$，可得 $A/\|A\|=z_B$，所以
$$
\dot\lambda=z_B^Tj.
$$

物理含义是：$j$ 沿 $z_B$ 的分量改变推力大小 $\lambda$，而 $j$ 垂直于 $z_B$ 的分量改变推力方向。

====================================

所以

$$
\dot z_B
=
\frac{j-(z_B^Tj)z_B}{\lambda}
=
\frac{(I-z_Bz_B^T)j}{\lambda}.
$$

这说明：jerk 沿 $z_B$ 的分量只改变总推力大小；jerk 垂直于 $z_B$ 的分量改变推力方向，因此改变姿态。

另一方面，由刚体运动学可知，对任意机体固连向量 $b=Rb_0$，有

$$
\dot b
=
{}^W\omega_{B/W}\times b.
$$

因此

$$
\dot z_B
=
{}^W\omega_{B/W}\times z_B.
$$

代入

$$
{}^W\omega_{B/W}=p x_B+q y_B+r z_B,
$$

得到

$$
\dot z_B
=
q x_B-p y_B.
$$

令

$$
h_\omega=\dot z_B
=
\frac{(I-z_Bz_B^T)j}{\lambda},
$$

则

$$
p=-h_\omega^Ty_B,
\qquad
q=h_\omega^Tx_B.
$$

这部分是 Mellinger ICRA 和 thesis 中正确的部分。

## 5. 第三个角速度分量 $r$ 的修正

ICRA 论文直接写

$$
r=\dot\psi\,z_W^Tz_B.
$$

这个公式一般不成立。原因是 Mellinger 使用的是 Z-X-Y Euler 角关系。thesis 中写出的完整角速度分解为

$$
{}^W\omega_{B/W}
=
[x_C\ y_B\ z_W]
\begin{bmatrix}
\dot\phi\\
\dot\theta\\
\dot\psi
\end{bmatrix}.
$$

同时

$$
{}^W\omega_{B/W}
=
R
\begin{bmatrix}
p\\q\\r
\end{bmatrix}.
$$

于是

$$
[x_C\ y_B\ z_W]^{-1}R
\begin{bmatrix}
p\\q\\r
\end{bmatrix}
=
\begin{bmatrix}
\dot\phi\\
\dot\theta\\
\dot\psi
\end{bmatrix}.
$$

修正后的做法是：先由 jerk 求出 $p,q$，再用上式第三个标量方程和给定的 $\dot\psi$ 解 $r$。

从 (2.21) 两边点乘 $z_B$ 可见问题所在：

$$
r
=
\dot\phi\,x_C^Tz_B
+
\dot\psi\,z_W^Tz_B.
$$

ICRA 的公式只保留了第二项

$$
\dot\psi\,z_W^Tz_B,
$$

漏掉了

$$
\dot\phi\,x_C^Tz_B.
$$

若使用 Mellinger 的 Z-X-Y 欧拉角矩阵，则

$$
x_C^Tz_B=\sin\theta,
\qquad
z_W^Tz_B=\cos\phi\cos\theta,
$$

因此完整关系为

$$
r=\dot\phi\sin\theta+\dot\psi\cos\phi\cos\theta.
$$

ICRA 的简式相当于额外假设 $\dot\phi\sin\theta=0$，适合接近水平或特殊运动，不能作为一般的大角度微分平坦恢复公式。

## 6. 角加速度的一阶推导

从

$$
m a=-mgz_W+u_1z_B
$$

求导得到

$$
m j=\dot u_1z_B+u_1\dot z_B.
$$

再次求导得到

$$
m s=\ddot u_1z_B+2\dot u_1\dot z_B+u_1\ddot z_B.
$$

由

$$
\dot z_B
=
{}^W\omega_{B/W}\times z_B
$$

继续求导：

$$
\ddot z_B
=
{}^W\alpha_{B/W}\times z_B
+
{}^W\omega_{B/W}\times
(
{}^W\omega_{B/W}\times z_B
).
$$

因此

$$
m s
=
\ddot u_1z_B
+
2\dot u_1({}^W\omega_{B/W}\times z_B)
+
u_1\,{}^W\omega_{B/W}\times({}^W\omega_{B/W}\times z_B)
+
u_1\,{}^W\alpha_{B/W}\times z_B.
$$

令

$$
h_\alpha
=
{}^W\alpha_{B/W}\times z_B.
$$

则

$$
h_\alpha
=
\frac{m}{u_1}s
-
\frac{\ddot u_1}{u_1}z_B
-
2\frac{\dot u_1}{u_1}h_\omega
-
{}^W\omega_{B/W}\times h_\omega.
$$

又因为

$$
{}^W\alpha_{B/W}=R\dot\Omega
=
\dot p\,x_B+\dot q\,y_B+\dot r\,z_B,
$$

所以

$$
h_\alpha
=
{}^W\alpha_{B/W}\times z_B
=
\dot q\,x_B-\dot p\,y_B.
$$

因此

$$
\dot p=-h_\alpha^Ty_B,
\qquad
\dot q=h_\alpha^Tx_B.
$$

这部分与 Mellinger thesis (2.23) 的思路一致。

## 7. 第三个角加速度分量 $\dot r$ 的修正

ICRA 论文写

$$
{}^W\alpha_{B/W}^Tz_B
=
\ddot\psi\,z_W^Tz_B.
$$

这个公式一般不成立。若暂时沿用 ICRA 的简化式

$$
r=\dot\psi\,z_W^Tz_B,
$$

则至少应有

$$
\dot r
=
\ddot\psi\,z_W^Tz_B
+
\dot\psi\,z_W^T\dot z_B.
$$

其中

$$
\dot z_B=q x_B-p y_B.
$$

所以至少会出现

$$
\dot r
=
\ddot\psi\,z_W^Tz_B
+
\dot\psi
\left(
qz_W^Tx_B-pz_W^Ty_B
\right).
$$

ICRA 的写法漏掉了第二项。

更严格的修正版应采用 thesis 的处理方式：对完整 Euler-rate 方程

$$
{}^W\omega_{B/W}
=
[x_C\ y_B\ z_W]
\begin{bmatrix}
\dot\phi\\
\dot\theta\\
\dot\psi
\end{bmatrix}
$$

求导，得到一个关于

$$
\dot p,\dot q,\dot r
$$

的线性关系。thesis 将其写成

$$
A
\begin{bmatrix}
\dot p\\
\dot q\\
\dot r
\end{bmatrix}
+b
=
\begin{bmatrix}
\ddot\phi\\
\ddot\theta\\
\ddot\psi
\end{bmatrix}.
$$

修正后的做法是：先由 snap 求出 $\dot p,\dot q$，再用上式第三个标量方程和给定的 $\ddot\psi$ 解 $\dot r$。

## 8. 输入恢复

总推力为

$$
u_1=m\|a+gz_W\|.
$$

力矩输入由 Euler 方程恢复：

$$
\begin{bmatrix}
u_2\\u_3\\u_4
\end{bmatrix}
=
I
\begin{bmatrix}
\dot p\\\dot q\\\dot r
\end{bmatrix}
+
\begin{bmatrix}
p\\q\\r
\end{bmatrix}
\times
I
\begin{bmatrix}
p\\q\\r
\end{bmatrix}.
$$

这里的 $[p,q,r]^T$ 和 $[\dot p,\dot q,\dot r]^T$ 都是机体系坐标。

## 9. ICRA 论文中应标记的问题

### 9.1 角速度符号混用

ICRA 式 (4) 写成

$$
\dot\omega_{BW}
=
I^{-1}
\left(
-\omega_{BW}\times I\omega_{BW}
+
\begin{bmatrix}
u_2\\u_3\\u_4
\end{bmatrix}
\right).
$$

问题：$I$ 是沿 $x_B-y_B-z_B$ 轴的机体系惯量矩阵，Euler 方程中的角速度列向量必须是机体系坐标 $\Omega=[p,q,r]^T$。因此该式应读成

$$
\dot\Omega
=
I^{-1}
\left(
-\Omega\times I\Omega+\tau^B
\right).
$$

若把 $\omega_{BW}$ 当成世界系坐标列向量，则不能使用常数 $I$，而应使用

$$
I^W=RIR^T.
$$

### 9.2 第三个角速度分量 $r$ 的公式过度简化

ICRA 写

$$
r=\dot\psi\,z_W^Tz_B.
$$

修正后应从完整 Euler-rate 关系求 $r$。点乘 $z_B$ 可见至少应有

$$
r
=
\dot\phi\,x_C^Tz_B
+
\dot\psi\,z_W^Tz_B.
$$

因此 ICRA 少了 $\dot\phi\,x_C^Tz_B$。

### 9.3 $z_B$ 轴角加速度公式过度简化

ICRA 写

$$
{}^W\alpha_{B/W}^Tz_B
=
\ddot\psi\,z_W^Tz_B.
$$

修正后应由完整 Euler-rate 导数方程求 $\dot r$。ICRA 的表达漏掉了由于姿态图表变化产生的耦合项。

### 9.4 三重积断言一般不成立

ICRA 声称

$$
z_B^T(\omega_{CW}\times\omega_{BC})=0.
$$

一般情况下这个三重积不为零。它对应 yaw frame 与 body frame 相对运动之间的耦合项，正是 $\dot r$ 中容易漏掉的部分。

## 10. Thesis 中可保留与需注意的地方

### 10.1 可保留的部分

Thesis 中的姿态恢复公式

$$
z_B=\frac{a+gz_W}{\|a+gz_W\|},
\qquad
y_B=\frac{z_B\times x_C}{\|z_B\times x_C\|},
\qquad
x_B=y_B\times z_B
$$

可以保留，但需注明非奇异条件。

Thesis 中用 $h_\omega$ 求 $p,q$ 的推导可以保留：

$$
h_\omega=\frac{m}{u_1}
\left(
j-(z_B^Tj)z_B
\right),
$$

$$
p=-h_\omega^Ty_B,
\qquad
q=h_\omega^Tx_B.
$$

Thesis 中用完整 (2.22) 第三个标量方程求 $r$，可以视为对 ICRA 的修正。

Thesis 中用 $h_\alpha$ 求 $\dot p,\dot q$ 的推导可以保留。

Thesis 中用完整 (2.25) 第三个标量方程求 $\dot r$，可以视为对 ICRA 的修正。

### 10.2 需注意的地方

Thesis 仍然存在记号混用：$\omega_{BW}$ 有时像几何向量，有时作为机体系列向量使用。严格整理时应写成

$$
{}^W\omega_{B/W}=R\Omega,
\qquad
\Omega={}^B\omega_{B/W}.
$$

Thesis 中出现 $\alpha_{BW}$ 时也应明确：参与叉乘的是几何角加速度向量

$$
{}^W\alpha_{B/W},
$$

而进入 Euler 方程的是机体系坐标

$$
\dot\Omega=
\begin{bmatrix}
\dot p\\\dot q\\\dot r
\end{bmatrix}.
$$

## 11. 奇异点与影响

### 11.1 零推力奇异

姿态恢复第一步要求

$$
\lambda=\|a+gz_W\|\neq0.
$$

当

$$
a=-gz_W
$$

时，

$$
u_1=0.
$$

此时四旋翼处于自由落体型参考加速度，总推力为零，推力方向 $z_B$ 无法由平动方程确定。因此 $R$、$\Omega$、$\dot\Omega$ 都无法由平坦输出唯一恢复。

影响：

- $z_B=(a+gz_W)/\|a+gz_W\|$ 无定义；
- $\dot z_B$ 中存在 $1/\|a+gz_W\|$；
- 角速度前馈会发散或无定义；
- 角加速度前馈会更严重地病态；
- 工程上不能用简单归一化 fallback 声称恢复了平坦映射。

### 11.2 姿态构造 yaw 奇异

Mellinger 的姿态构造要求

$$
\|z_B\times x_C\|\neq0.
$$

当

$$
z_B\parallel x_C
$$

时，$y_B$ 无法定义。

影响：

- $x_B,y_B$ 无法由该 chart 唯一恢复；
- 在奇异点附近，归一化叉乘会导致单位向量剧烈变化；
- $R$ 会对平坦输出的小扰动极端敏感；
- $r$ 与 $\dot r$ 的计算会病态；
- Mellinger 的 practical fix 是比较 $(x_B,y_B,z_B)$ 与 $(-x_B,-y_B,z_B)$ 哪个更接近当前实际姿态，但这只是分支选择，不能消除该 chart 的数学奇异。

### 11.3 Euler-rate 反解奇异

Thesis 中用

$$
[x_C\ y_B\ z_W]^{-1}
$$

反解 Euler-rate，因此需要

$$
y_B\times z_W\neq0.
$$

影响：

- 即使姿态矩阵 $R$ 本身良好，Z-X-Y Euler-rate chart 仍可能奇异；
- 用 (2.22) 求 $r$ 或用 (2.25) 求 $\dot r$ 会失败；
- 这是 Euler 图表奇异，而非 $SO(3)$ 本身奇异。

### 11.4 输入与执行器可行性

即使平坦映射数学上非奇异，恢复出的

$$
u_1,u_2,u_3,u_4
$$

仍可能超过执行器能力。

影响：

- 电机饱和会使实际轨迹不再满足平坦参考；
- 姿态、角速度、角加速度前馈在执行器限制下不能完全实现；
- 动态不可行轨迹需要限幅、QP 分配、时间缩放或重新规划。

## 12. 与 Tal/Karaman 的关系

修正后的 Mellinger 版本与 Tal/Karaman 的核心结构一致。

Mellinger 的 jerk 方程可写为

$$
j=\dot\tau z_B+\tau R[e_3]^T_\times\Omega,
$$

其中

$$
\tau=\frac{u_1}{m}.
$$

Tal/Karaman 写为

$$
j=\tau R[i_z]^T_\times\Omega+\dot\tau b_z.
$$

二者只是坐标方向和推力符号约定不同。

Mellinger 修正版用

$$
\text{jerk 方程}+\text{yaw-rate 方程}
$$

求 $\Omega$。

Tal/Karaman 用一个矩阵方程统一求

$$
\begin{bmatrix}
\Omega\\
\dot\tau
\end{bmatrix}.
$$

Mellinger 修正版用

$$
\text{snap 方程}+\text{yaw-acceleration 方程}
$$

求 $\dot\Omega$。

Tal/Karaman 用一个矩阵方程统一求

$$
\begin{bmatrix}
\dot\Omega\\
\ddot\tau
\end{bmatrix}.
$$

因此，Tal/Karaman 的写法更规整，Mellinger thesis 的修正版与它在几何上等价。

## 13. 可能的稳妥的实现版本

若要完全避免 Mellinger 的 Euler-rate 反解和记号混乱，可以采用直接流形导数法。

先由平坦输出构造

$$
R(t)=[x_B(t)\ y_B(t)\ z_B(t)].
$$

然后解析求

$$
\dot R(t),\qquad \ddot R(t).
$$

定义

$$
\Omega=\mathrm{vee}(R^T\dot R).
$$

再定义

$$
\dot\Omega=
\mathrm{vee}\left(R^T\ddot R-\widehat{\Omega}^2\right).
$$

这样得到的 $R,\Omega,\dot\Omega$ 来自同一条 $SO(3)$ 曲线，因此必然满足

$$
\dot R=R\widehat\Omega.
$$

这也是整理 Mellinger 推导时最安全的数学版本。



## Mellinger 修正版中的奇异性位置

第一层是推力方向层。定义

$$
A=a+gz_W,
\qquad
u_1=m\|A\|,
\qquad
z_B=\frac{A}{\|A\|}.
$$

最早的奇异从 $\|A\|=0$ 开始，即 $a=-gz_W$。此时 $u_1=0$，推力方向 $z_B$ 无法定义，姿态 $R$、角速度 $\Omega$、角加速度 $\dot\Omega$ 都无法由平坦输出唯一恢复。

第二层是角速度前馈层。由于

$$
h_\omega=\dot z_B
=
\frac{(I-z_Bz_B^T)j}{\|A\|}
=
\frac{m}{u_1}\left(j-(z_B^Tj)z_B\right),
$$

所以 $\|A\|\to0$ 时，$h_\omega$ 病态；进一步地，

$$
p=-h_\omega^Ty_B,
\qquad
q=h_\omega^Tx_B
$$

也会病态。

第三层是角加速度前馈层。由于

$$
h_\alpha
=
\frac{m}{u_1}s
-
\frac{\ddot u_1}{u_1}z_B
-
2\frac{\dot u_1}{u_1}h_\omega
-
\omega^W\times h_\omega,
$$

所以 $h_\alpha$ 同时含有 $1/u_1$、$\dot u_1/u_1$、$\ddot u_1/u_1$，并依赖 $h_\omega$。因此 $\|A\|\to0$ 时，$h_\alpha$ 比 $h_\omega$ 更敏感。

第四层是 yaw 构造层。即使 $\|A\|>0$，仍需

$$
\|z_B\times x_C\|>0.
$$

因为

$$
y_B=\frac{z_B\times x_C}{\|z_B\times x_C\|},
\qquad
x_B=y_B\times z_B.
$$

当 $z_B\parallel x_C$ 时，$x_B,y_B$ 无法由该公式稳定构造，进而影响 $R$、$p,q$ 的投影、$r$ 和 $\dot r$。

第五层是 Euler-rate 反解层。Mellinger thesis 修正版用

$$
[x_C\ y_B\ z_W]^{-1}
$$

求 $r$ 和 $\dot r$，因此还要求该矩阵可逆，等价地需要 $y_B\times z_W$ 远离零。该奇异主要影响 $r,\dot r$，属于 yaw/Euler-rate 图表问题。

总结：最早从 $\|A\|=\|a+gz_W\|$ 开始；$h_\omega$ 直接受它影响；$h_\alpha$ 直接且更强地受它影响；$\|z_B\times x_C\|$ 和 $y_B\times z_W$ 是后续姿态/yaw 图表层的奇异。