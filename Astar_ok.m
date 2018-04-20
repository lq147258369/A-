%% A*
%%by xiongzhan 20170325
clc;
clear;
% pause(3); 
%% ��ʼ����
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

%% ��ͼ��ʼ��?
figure('name','A*','NumberTitle','off','MenuBar','none');
global point 
% ��point��Ϊһ���������ݿ⣬ʹ������Ҳ��ʹ������
for i=1:n*n
    point(i).num = i;
    point(i).father=[];
    point(i).Gcost=[];
    point(i).Hcost=[];
end
%% �����ϰ���
banList=[randi(n*n,[1,floor(banper*n*n)])];%floor�������������ȡ��;����������ֵn*n,1*floor��225���ľ���
%load banList
A=[];
A = banList(find(banList==goalNum));%find(a),�ҳ�banList��goalNum�����к���
for j = 1:length(banList)
    if banList(j)~=goalNum || banList(j)~=starNum %~=������
        point(banList(j)).Gcost = Inf;%inf�������
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
while length(openList) %����openList�ĳ���
    % openList�Ǵ������Ľڵ㣬Ϊ��˵���޽�
    costList = getCost(openList,point(goalNum),n);
    % ����openlist�нڵ�Ĵ��ۣ��Դ�����С�Ľڵ�Ϊ��ǰ�ڵ㣬����������չ
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
            %�����ж��Ƿ����ϰ���/closeList
            log1=0;
            try
                tmp=point(neighbourNum(i)).Gcost;
                if tmp ==inf
                    log1 = 1;
                end
            catch
                log1=0;
            end
            %�ж��Ƿ���closelistNum�У������������??
            if log1 || ismember(neighbourNum(i),closeListNum)
                continue;
            elseif (ismember(neighbourNum(i),openListNum))
            %�ж��Ƿ���openList�У������Ҫ��������ǣ��жϽڵ���·����G�����Ƿ�С��ԭ·��?
            %neighbourNum(ii)��openList�У�����neighbourNum(ii)Ϊ���ڵ�GcostС������neighbourNum(ii)Ϊ���ڵ㡣?
            oldGcost = getGcost(point(neighbourNum(i)),n);
            father = point(neighbourNum(i)).father;
            point(neighbourNum(i)).father = currentPoint.num;
            newGcost = getGcost(point(neighbourNum(i)),n);
            if newGcost>oldGcost
                %���۴���ԭ·���������ڵ����á�
                point(neighbourNum(i)).father = father;
            else
                %����С��ԭ·���������ڵ��õ�ǰ�ڵ�������޸�Gcost��
                point(neighbourNum(i)).Gcost = newGcost;
            end
            continue;
        elseif ~ismember(neighbourNum(i),closeListNum)
            %ǰ���������ų�ʱ�����ڵ���뵽openList�����游�ڵ㡢���۵���Ϣ��?
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

