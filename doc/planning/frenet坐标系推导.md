# Frenet坐标系公式推导

用 $l^`$  表示 $\frac{{\rm d}l}{{\rm d}s}$ ；$\dot{l}$表示 $\frac{{\rm d}l}{{\rm d}t}$；$\dot{s}$ 表示$\frac{{\rm d}s}{{\rm d}t}$ 

$l$ 表述为$s$ 的函数$l=f(s)$

$B(x_c,y_c)$ 点为当前点,其切线向量和法向量分别为$\vec{N_c}$,$\vec{T_c}$

$\vec{T_c}=(cos\theta_c,sin\theta_c)$

$\vec{N_c}=(-sin\theta_c,cos\theta_c)$

$A(x_r,y_r)$点为参考线上的匹配点(math point),其切线向量和法向量分别为$\vec{N_r}$,$\vec{T_r}$

$\vec{T_r}=(cos\theta_r,sin\theta_r)$

$\vec{N_r}=(-sin\theta_r,cos\theta_r)$



$O$点为笛卡尔坐标系原点



问题：
求$l^`$  , $l^{``}$ ,$\dot{s}$ ,$\ddot{s}$

## 求$l^`$

$$l^`=\frac{{\rm d}l}{{\rm d}s} =\quad{\frac{{\rm d}l}{{\rm d}t} \over \frac {{\rm d}s}{{\rm d}t}}=\quad{\dot{l} \over \dot{s}}$$

$\vec{l}=\vec{AB}=(\vec{OB}-\vec{OA})$

$l=\vec{l}.\vec{N_r}=(\vec{OB}-\vec{OA}).\vec{N_r}$

$\dot{l}=(\dot{\vec{OB}}-\dot{\vec{OA}}).\vec{N_r}+(\vec{OB}-\vec{OA}).\dot{\vec{N_r}}$

$\dot{\vec{N_r}}=\frac{{\rm d}\vec{N_r}}{{\rm d}s}=\frac{{\rm d}\vec{N_r}}{{\rm d}\theta_r}.\frac{{\rm d}\theta_r}{{\rm d}s}.\frac{{\rm d}s}{{\rm d}t}$

由于$\frac{{\rm d}\theta_r}{{\rm d}s}=k_r$ (参考点曲率)，$\frac{{\rm d}s}{{\rm d}t}=\dot{s}$

所以

$\dot{\vec{N_r}}=\frac{{\rm d}\vec{N_r}}{{\rm d}\theta_r}.k_r.\dot{s}$

由于$\vec{N_r}=(-sin\theta_r,cos\theta_r)$

所以

$\frac{{\rm d}\vec{N_r}}{{\rm d}\theta_r}=(-cos\theta_r,-sin\theta_r)$

$\dot{\vec{N_r}}=(-cos\theta_r,-sin\theta_r).k_r.\dot{s}=-\vec{T_r}.k_r.\dot{s}$

$(\vec{OB}-\vec{OA}).\dot{\vec{N_r}}=(l.\vec{N_r}).(-\vec{T_r}.k_r.\dot{s})$

由于$\vec{N_r}和$$\vec{T_r}$正交

所以$(\vec{OB}-\vec{OA}).\dot{\vec{N_r}}=(l.\vec{N_r}).(-\vec{T_r}.k_r.\dot{s})=0$

$\dot{l}=(\dot{\vec{OB}}-\dot{\vec{OA}}).\vec{N_r}+(\vec{OB}-\vec{OA}).\dot{\vec{N_r}}=(\dot{\vec{OB}}-\dot{\vec{OA}}).\vec{N_r}$

$O$为原点,$B$ 为汽车当前位置，所以$\dot{\vec{OB}}=v_c$ (注意这里的$v_c$ 中的下标表示的是当前点的意思)

同理 $\dot{\vec{OA}}=v_r$

从而得到

$\dot{l}=(v_c.\vec{T_c}-\dot{s}.\vec{T_r}).\vec{N_r}=v_c.\vec{T_c}.\vec{N_r}$

带入$\vec{T_c}=(cos\theta_c,sin\theta_c)$，$\vec{N_r}=(-sin\theta_r,cos\theta_r)$

得到

$\dot{l}=v_c.(cos\theta_c,sin\theta_c).(-sin\theta_r,cos\theta_r)=v_csin(\theta_c-\theta_r)$

$\dot{\vec{v_c}}=\frac{{\rm d}\vec{OB}}{{\rm d}t}=\frac{{\rm d}(\vec{OA}+l.\vec{N_r})}{{\rm d}t}=\dot{\vec{OA}}+\dot{l}.\vec{N_r}+l.\dot{N_r}=\dot{s}.\vec{T_r}+\dot{l}.\vec{N_r}-k_r.l.\dot{s}.\vec{T_r}=\dot{s}.(1-k_r.l).\vec{T_r}+\dot{s}.\vec{N_r}$

由于$\vec{N_r}和$$\vec{T_r}$正交

所以

$v_c=\sqrt{(\dot{s}(1-k_r.l))^2+\dot{l}^2}$

得到

$\dot{l}=v_c.(cos\theta_c,sin\theta_c).(-sin\theta_r,cos\theta_r)=v_csin(\theta_c-\theta_r)=\sqrt{(\dot{s}(1-k_r.l))^2+\dot{l}^2}.sin(\theta_c-\theta_r)$

令 $\Delta\theta=\theta_c-\theta_r$

$\dot{l}^2=[(\dot{s}(1-k_r.l))^2+\dot{l}^2].sin(\Delta\theta)^2$



$\dot{l}^2.cos(\Delta\theta)^2=\dot{s}^2(1-k_r.l)^2.sin(\Delta\theta)^2$



$\quad{\dot{l}^2 \over \dot{s}^2}=(1-k_r.l)^2.\quad {sin(\Delta\theta)^2 \over cos(\Delta\theta)^2}=(1-k_r.l)^2.tan(\Delta\theta)^2$

**所以**

$$l^`=\quad{\dot{l} \over \dot{s}}=(1-k_r.l).tan(\Delta\theta)$$

## 求$l^{``}$





