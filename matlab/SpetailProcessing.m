function [sigma, A] = SpetailProcessing(sigmas, us, dt, eps1)
% ��������������������ĸ���MDLg�ַ���
% INPUT 
% ��ɢ�������Ժ�ĳ�ʼ����ĸ��:sigmas
% �ο������źţ�us
% ����ʱ������dtͬ�ο�����ur
% �������:eps1
% OUTPUT
% ���յ���ĸ��sigma
% ֱ�߱�־���ϲ���Ԫ�Ĳ���A

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ��������ȹʿ�����ֱ�ߵ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������G,j*j
for i=1:j
    for p=1:j
        G(i,p) = InnerProducts(sigmas(i,:), sigmas(p,:), dt);
    end
end
% ��������v
%A = [];
i=1;
v=[];
while (i<n+1)
    for p=1:j
        v(p,1) = InnerProducts(us(i,:), sigmas(p,:), dt); 
    end
    alpha = G\v;
    if (abs(InnerProducts(us(i,:), us(i,:), dt)-alpha'*G*alpha) < eps1)
        A = [A;alpha']; % A��ÿһ����һ��segment��Ӧ�Ĳ�����ʾ
        i = i+1;
    else
        for p=1:mk
            uiv(p) = us(i,p) - alpha'* sigmas(:,p);
        end
        sigmas = [sigmas;uiv];
        % ���¼������G,j+1*j+1
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