function [ui,s] = Segmentation(ur, dt, eps)
% INPUT
% ur：输出参考信号的采样，是一个向量，长度是d
% d：采样点数量
% dt：采样间隔
% eps：epsilon是放大边界值，由使用者给定
% OUTPUT
% ui: 是一个n*m的矩阵，有n个segment，每个segment有m个采样
% s: 每一段segment的采样点数目

% 计算ur的长度
d=length(ur);

% 计算微分
for k = 1:d-1
    urd(k) = (ur(k+1)-ur(k))/ dt;
end

for k = 1:d-2
    urdd(k) = (urd(k+1)-urd(k))/dt;
end

% 确定每个segment的起始点
endpoint(1) = 1;
i = 2;
for k = 2:d-2
    if (abs(urdd(k))-eps)*(abs(urdd(k-1))-eps) < 0
        endpoint(i) = k;
        i = i+1;
    end
end
endpoint(i) = d;
%i = i-1;
segmentnumber = i-1;

% 获取每个segment的采样点数
%segmentsample(1) = endpoint(1);
for n = 1:segmentnumber
        s(n) = endpoint(n+1)-endpoint(n);
end

% 提取segment最大采样数
samplemax = 0;
for k = 1:segmentnumber
    if samplemax < s(k)
        samplemax = s(k);
    end
end

% 输出矩阵ui
for n = 1:segmentnumber
    for m = 1:samplemax
        if m <= s(n)
            ui(n,m) = ur(endpoint(n)+m-1);
        else
            ui(n,m) = 0;
        end
    end
end

