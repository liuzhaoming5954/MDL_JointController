% clear;
% load('sigma.mat') % 导入基数据
% sigmas_j1 = cell2mat(sigmas(1,1));
% sigmas_j2 = cell2mat(sigmas(2,1));
% sigmas_j3 = cell2mat(sigmas(3,1));
% sigmas_j4 = cell2mat(sigmas(4,1));
% figure(1);
% plot(sigmas_j1(1,:),'DisplayName','1');hold all;plot(sigmas_j1(2,:),'DisplayName','2');plot(sigmas_j1(3,:),'DisplayName','3');plot(sigmas_j1(4,:),'DisplayName','4');plot(sigmas_j1(5,:),'DisplayName','5');plot(sigmas_j1(6,:),'DisplayName','6');;plot(sigmas_j1(7,:),'DisplayName','7');hold off;
% legend('1','2','3','4','5','6','7');
for i=1:1:5
    figure(i);
    sigmas_j = cell2mat(sigmas(i,1));
    [m,n] = size(sigmas_j);
    for qi=1:1:m
        subplot(2,5,qi);
        plot(sigmas_j(qi,:));
        hold;
        grid on;
%         str = ['joint ',num2str(qi)];
%         title(str);
%         xlabel('time(s)');% x轴名称
%         ylabel('angle(rad)'); 
    end
end

%% 测试曲线拟合工具
