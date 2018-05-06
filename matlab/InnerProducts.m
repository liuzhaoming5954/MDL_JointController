% 求内积函数
function V=InnerProducts(u, v, dt)
% INPUT
% u,v分别是两个向量
% dt是时间截个
% OUTPUT
% 输出是一个数，标量

mk=length(u);

V = 0;
for s=1:mk-1
    V = V+dt/2*(u(s+1)*v(s+1)+u(s)*v(s));
end
