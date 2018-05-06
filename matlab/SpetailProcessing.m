function [sigma, A] = SpetailProcessing(sigmas, us, dt, eps1)
% 本函数用于生成最终字母表和MDLg字符串
% INPUT 
% 离散化处理以后的初始化字母表:sigmas
% 参考输入信号：us
% 采样时间间隔：dt同参考输入ur
% 控制误差:eps1
% OUTPUT
% 最终的字母表：sigma
% 直线标志：合并基元的参数A

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 这个函数先故考虑是直线的情况
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 计算矩阵G,j*j
for i=1:j
    for p=1:j
        G(i,p) = InnerProducts(sigmas(i,:), sigmas(p,:), dt);
    end
end
% 计算向量v
%A = [];
i=1;
v=[];
while (i<n+1)
    for p=1:j
        v(p,1) = InnerProducts(us(i,:), sigmas(p,:), dt); 
    end
    alpha = G\v;
    if (abs(InnerProducts(us(i,:), us(i,:), dt)-alpha'*G*alpha) < eps1)
        A = [A;alpha']; % A的每一行是一个segment对应的参数表示
        i = i+1;
    else
        for p=1:mk
            uiv(p) = us(i,p) - alpha'* sigmas(:,p);
        end
        sigmas = [sigmas;uiv];
        % 重新计算矩阵G,j+1*j+1
        j = j+1;
        for i=1:j
            for p=1:j
                G(i,p) = InnerProducts(sigmas(i,:), sigmas(p,:), dt);
            end
        end
        i = 1;
        clear A;
        A = [];
    end
    clear uiv;
end
sigma = sigmas;