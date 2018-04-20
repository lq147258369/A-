%% calculate the cost from the goal point to current point
function cost = getHcost(startPoint,goalPoint,n)
[xs,ys]=ind2sub([n,n],startPoint.num); %ind2sub矩阵元素的index转换成对应的下标，startPoint.num是30*30的矩阵
[xg,yg]=ind2sub([n,n],goalPoint.num); %I,J代表的是序号为IND的元素在这个矩阵中的行号和列号
cost = abs(xs-xg)+abs(ys-yg);
end