i = 1;
t = PVT5.time;
pp1 = polyfit(t,PVT5.j1p,1);
sigmao(i,:) = pp1(1)+pp1(2)*t;
i = i+1;
pp2 = polyfit(PVT5.time,PVT5.j1p,2);
sigmao(i,:) = pp1(1)+pp1(2)*t;
pp3 = polyfit(PVT5.time,PVT5.j1p,3);
pp4 = polyfit(PVT5.time,PVT5.j1p,4);
pp5 = polyfit(PVT5.time,PVT5.j1p,5);
% mymodel = fittype('b*t+c','independent','t');
% fit1= fit(PVT1.time,PVT1.j1p,mymodel,'start',[0,0]);

pp1(1)