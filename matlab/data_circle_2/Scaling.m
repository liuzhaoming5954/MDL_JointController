function [us, sigmas, B, Ts] = Scaling(ui, sigmao, s, dt)
% function [us, bate] = Scaling(ui, s, k, dt)
% INPUT 
% ui: n*m矩阵，分段以后的控制参考信号
% sigmao: j*k矩阵，初始阶段的字母表
% s: 一个行向量，ui矩阵每一行实际有的采样点个数
% dt: 采样时间间隔
% m: ui的m
% k: sigmao的k
% 
% OUTPUT
% us: n*max(m,k)矩阵
% sigmas: j*max(j,k)矩阵
% bate: 1*n的延展向量
% Ts: 拉长之后每一段segment的时长T

% 计算矩阵长度
[n,m]=size(ui);
[j,k]=size(sigmao);

% 首先计算出m和k的较大值,以及一段segment新的总时长T
mk = max(m, k);
Ts = dt*(mk-1);

% 计算Bs向量,输出bate向量
for i=1:n
    Bs(i) = (mk-1)/(s(i)-1);
    B(i) = (s(i)-1)/(mk-1);
end

% 计算输出us矩阵
for i=1:mk
    x1(i)=dt*(i-1);
end
for i=1:n
    for p=1:s(i)
        x(p)=Bs(i)*dt*(p-1);
    end
    y = ui(i,1:s(i)); 
    if length(x) == 1
        for ii = 1:1:length(x1)
            us(i,ii) = y;
        end
    else
        us(i,:) = interp1(x, y, x1, 'PCHIP');
    end        
    clear x;
    clear y;
end

% 计算输出sigmas矩阵
BBs = (mk-1)/(k-1);
for p=1:mk
    xx1(p)=dt*(p-1);
end
for i=1:j
    for p=1:k
        xx(p)=BBs*dt*(p-1);
    end
    sigmas(i,:) = interp1(xx, sigmao(i,:), xx1, 'PCHIP');
    clear xx;
end

