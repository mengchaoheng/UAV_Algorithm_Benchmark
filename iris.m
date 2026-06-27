clear; clc;
% PX4的控制分配：
% 对于一般模型：y = [Mx; My; Mz; Fx; Fy; Fz]=B * u, 其中u为舵机时表示偏转角，电机时表示转速平方O^2。单位化时首先将u单位化
% y=B*U*u_norm, 其中U包含每个执行器u的最大幅值。PX4的控制效应矩阵就是 B_px4=B*U，对应最大转速产生的力和力矩。
% 并且为了避免对每个机型的B_px4都调节控制增益，
% 进一步将其单位化:
% D*y=D*B_px4*u_norm ，其中D为B_px4每行公因子，提取算法接近:
% 先取 mix=pinv(B_px4),然后选取mix的非零列的绝对值的平均值做缩放因子，每列因子作为D对角阵。定义
% B_norm=D*B_px4
% 是实际分配采用的分配矩阵。同时，由于y=k*e来自误差控制器，k为控制增益，D和k不必区分，PX4的控制增益K=D*k就吸收了D。
% 认为控制器输出就已经是单位化的。这样只需要调K参数。实际上填B_px4时重要的是元素间的比例关系，行的绝对幅值无关紧要。
% 

% Sun et al. 2022 Table II equivalent physical scale.
m = 0.75;
J = diag([2.5, 2.1, 4.3])*1e-3;   % kg*m^2

CT = 8.5;                          % max thrust per normalized actuator [N] =Fmax = ct*O_max^2 ，我们在sdf文件取O_max=2.3726e+03
  
ct = 1.51e-6;  % motorConstant
cq = 2.37e-8; 
kappa = cq/ct;                     % yaw moment / thrust [m]  =momentConstant

% Iris Gazebo Classic geometry and actuator order, unchanged.
pos = [ ...
     0.13,  0.22, -0.023;
    -0.13, -0.20, -0.023;
     0.13, -0.22, -0.023;
    -0.13,  0.20, -0.023];
KM = [kappa; kappa; -kappa; -kappa];

axis = [0; 0; -1];

% 论文中常见的几种建模方式：
%对于模型y=B1*u1 ,其中执行器作为转速平方u=[O_1^2; O_2^2;...]，
% B1 = zeros(6,4);
% 
% for i = 1:4
%     r = pos(i,:)';
%     moment = ct * cross(r, axis) - ct * KM(i) * axis;
%     force  = ct * axis;
%     B1(:,i) = [moment; force]; 
% end

%对于模型y=B2*u2 ,其中执行器作为推力u=[f_1; f_2;...]，f_i=ct*O^2,
B2 = zeros(6,4);

for i = 1:4
    r = pos(i,:)';
    moment = 1 * cross(r, axis) - 1 * KM(i) * axis;
    force  = 1 * axis;
    B2(:,i) = [moment; force]; %Sun et al. 2022 and also the defualt formula of this simulation
end

% 对于PX4:y=B3*u_norm , u_norm是归一化的[0,1],B3 元素对于最大转速产生的力和力矩
B3 = zeros(6,4);

for i = 1:4
    r = pos(i,:)';
    moment = CT * cross(r, axis) - CT * KM(i) * axis;
    force  = CT * axis;
    B3(:,i) = [moment; force];
end

% in px4:
[D, B3_norm, mix_norm, scale, mix_raw] = px4_normalize_B(B3, true);
B_px4 = [B3(6,:); B3(1:3,:)] % [Fz; Mx; My; Mz]
B_px4_norm=[B3_norm(6,:); B3_norm(1:3,:)]

% in this simulation:
B = [B2(6,:); B2(1:3,:)] % [Fz; Mx; My; Mz]