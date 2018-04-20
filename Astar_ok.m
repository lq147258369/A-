%% A*
%%by xiongzhan 20170325
clc;
clear;
% pause(3); 
%% 初始数据
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map
starpoint = [80 90];
goalpoint = [50 30];
res = 5;
iterators = dims/res;
iterators = ceil(iterators)+[1 1];
n=ceil(105/5);
starNum=ceil(starpoint(1,1)/res)*n+ceil(starpoint(1,2)/res);
 %starNum = randi(n*n,[1,1]);
goalNum=ceil(goalpoint(1,1)/res)*n+ceil(goalpoint(1,2)/res);
 %goalNum = randi(n*n,[1,1]);
banper=0.25;
for i = 1:iterators(2)
    for j = 1:iterators(1)
        testPos = limsMin + [j-1 i-1]*res; %to counteract 1 based indexing
        %notice, that i and j have also been swapped here so that the only
        %thing left to do is invert the y axis. 
        mapArray(i,j) = botSim.pointInsideMap(testPos);
    end
end

%% 地图初始化?
figure('name','A*','NumberTitle','off','MenuBar','none');
global point 
% 将point作为一个公共数据库，使子数据也能使用数据
for i=1:n*n
    point(i).num = i;
    point(i).father=[];
    point(i).Gcost=[];
    point(i).Hcost=[];
end
%% 设置障碍
banList=[randi(n*n,[1,floor(banper*n*n)])];%floor函数朝负无穷方向取整;随机生成最大值n*n,1*floor（225）的矩阵
%load banList
A=[];
A = banList(find(banList==goalNum));%find(a),找出banList中goalNum所在行和列
for j = 1:length(banList)
    if banList(j)~=goalNum || banList(j)~=starNum %~=不等于
        point(banList(j)).Gcost = Inf;%inf正无穷大
    end
end
point(starNum).Gcost=0;
point(starNum).father = point(starNum).num;
point(starNum).Hcost=getHcost(point(starNum),point(goalNum),n);

%% A* key point
openList = [];
closeList = [];
closeListNum=[];
openListNum=[];
openList = [openList,point(starNum)];
while length(openList) %返回openList的长度
    % openList是待被检测的节点，为空说明无解
    costList = getCost(openList,point(goalNum),n);
    % 计算openlist中节点的代价，以代价最小的节点为当前节点，继续向外扩展
    currentPoint = openList(find(costList==min(costList),1));
    openList(find(min(costList)==costList,1))=[];
    closeList = [closeList,currentPoint];
    neighbourNum = getNeighbour(currentPoint,n);
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
    for i = 1:length(neighbourNum)
%         ne1 = neighbourNum(ii);
%         if ne1==146 && currentPoint.num==166
%             ne1
%         end
        if neighbourNum(i)==point(goalNum).num
            point(neighbourNum(i)).father = currentPoint.num;
            point(goalNum).father = currentPoint.num;
            disp('ok')
            routPlot(goalNum,n);
            return;
        end
            %首先判断是否是障碍物/closeList
            log1=0;
            try
                tmp=point(neighbourNum(i)).Gcost;
                if tmp ==inf
                    log1 = 1;
                end
            catch
                log1=0;
            end
            %判断是否在closelistNum中，如果是则跳过??
            if log1 || ismember(neighbourNum(i),closeListNum)
                continue;
            elseif (ismember(neighbourNum(i),openListNum))
            %判断是否在openList中，如果在要分情况考虑，判断节点新路径的G代价是否小于原路径?
            %neighbourNum(ii)在openList中，若以neighbourNum(ii)为父节点Gcost小，则用neighbourNum(ii)为父节点。?
            oldGcost = getGcost(point(neighbourNum(i)),n);
            father = point(neighbourNum(i)).father;
            point(neighbourNum(i)).father = currentPoint.num;
            newGcost = getGcost(point(neighbourNum(i)),n);
            if newGcost>oldGcost
                %代价大于原路径，将父节点重置。
                point(neighbourNum(i)).father = father;
            else
                %代价小于原路径，将父节点用当前节点替代，修改Gcost。
                point(neighbourNum(i)).Gcost = newGcost;
            end
            continue;
        elseif ~ismember(neighbourNum(i),closeListNum)
            %前面的情况都排除时，将节点加入到openList并储存父节点、代价等信息。?
            point(neighbourNum(i)).father = currentPoint.num;
            point(neighbourNum(i)).Gcost = getGcost(point(neighbourNum(i)),n);
            point(neighbourNum(i)).Hcost = getHcost(point(neighbourNum(i)),point(goalNum),n);
            openList = [openList,point(neighbourNum(i))];
            end
    end
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
    pause(0.1);
    mydrawnow(starNum,goalNum,banList,closeListNum,openListNum,n);
end

