function [sigmao, str_line] = InitSigmao(ur, dt)
% INPUT 
% 参考输入信号：ur
% 采样时间间隔：dt同参考输入ur
% OUTPUT
% 输出初始化的sigmao矩阵
% 直线标志：str_line 1表示参考信号为直线，0表示参考信号不是直线
% 函数拟合使用工具箱：CFtool

% 初始化相关参数
L = length(ur);
ti = 0;
tf = ti+dt*(L-1);
% v0 = 0;
% v1 = 0;
q0 = ur(1);
q1 = ur(L);
%h = q1-q0;
T = tf-ti;

% 产生采样时间
t1 = [0:dt:tf]';
i = 1; % 确定Sigmao的个数
% 计算基函数参数

% 生成直线基函数
% [a_l0, a_l1] = linefuntion(q0,q1,ti,tf);
% Line trajectory function 直线
% sigmao(i,:) = a_l0+a_l1*t1;
P1=fit(t1,ur,'poly1','Normalize','on');
sigmao(i,:) = P1(t1);
% Calculate Frechet-distance between two function
% TODO:后期把这条判断直线的语句去掉，有直线我也能解决
frechet_value = frechet(t1,ur,t1,sigmao(i,:)');
if(frechet_value<=0.01)
    str_line = 1; %直线标志位，是一条直线
    fprintf('%s\n','轨迹是一条直线');
else
    i = i+1;
    
    % a_pa = polyfit(ur,t1,2);
    % Parabola 二次曲线
    % sigmao(i,:) = a_pa(1)+a_pa(2)*t1+a_pa(3)*t1.^2;
    P2=fit(t1,ur,'poly2','Normalize','on');
    sigmao(i,:) = P2(t1);
    i = i+1;
    
    % a_c = polyfit(ur,t1,3);
    % Cubic trajectory function 三次曲线
    % sigmao(i,:) = a_c(1)+a_c(2)*t1+a_c(3)*t1.^2+a_c(4)*t1.^3;
    P3=fit(t1,ur,'poly3','Normalize','on');
    sigmao(i,:) = P3(t1);
    i = i+1;
    
    % a_f = polyfit(ur,t1,4);
    % 四次曲线
    % sigmao(i,:) = a_f(1)+a_f(2)*t1+a_f(3)*t1.^2+a_f(4)*t1.^3+a_f(5)*t1.^4;
%     P4=fit(t1,ur,'poly4');
%     sigmao(i,:) = P4(t1);
%     i = i+1;
    
    % a_p = polyfit(ur,t1,5);
    % Polynomial_5 trajectory function 五次曲线
    % sigmao(i,:) = a_p(1)+a_p(2)*t1+a_p(3)*t1.^2+a_p(4)*t1.^3+a_p(5)*t1.^4+a_p(6)*t1.^5;
    P5=fit(t1,ur,'poly5','Normalize','on');
    sigmao(i,:) = P5(t1);
    i = i+1;
    
    % 两条正弦曲线组合
    sin2=fit(t1,ur,'sin2','Normalize','on');
    sigmao(i,:) = sin2(t1);
    i = i+1;
    
    % 傅立叶级数组合
    fourier2=fit(t1,ur,'fourier2','Normalize','on');
    sigmao(i,:) = fourier2(t1);
    
%     P6=fit(t1,ur,'poly6');
%     sigmao(i,:) = P6(t1);
    
    
    str_line = 0;% 直线标志位，不是直线
end
