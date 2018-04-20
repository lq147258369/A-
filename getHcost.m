%% calculate the cost from the goal point to current point
function cost = getHcost(startPoint,goalPoint,n)
[xs,ys]=ind2sub([n,n],startPoint.num); %ind2sub����Ԫ�ص�indexת���ɶ�Ӧ���±꣬startPoint.num��30*30�ľ���
[xg,yg]=ind2sub([n,n],goalPoint.num); %I,J����������ΪIND��Ԫ������������е��кź��к�
cost = abs(xs-xg)+abs(ys-yg);
end