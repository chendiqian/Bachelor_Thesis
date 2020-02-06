%增大仓库，改善绘图的卡顿，增加出货口，基本完成充电桩功能,合理设置充电桩位置以避免碰撞，并能显示电量不足
%利用A_direction矩阵的设定值，设置单行线，并完善机器人在货架下面对撞问题
%去除冗余变量，移动至机器人struct；货物增加状态，方便画图，减少变量
%把寻路整个过程放到子程序中去，动态生成货物，开另一个定时器动态分配任务，即机器人动态出现。
%============================================================================
%版本介绍：
%使用A*最短路径算法，比较迪杰斯特拉算法而言效率更高，
%货物动态生成，机器人动态规划，发现有未处理的任务立即前往
%较结题答辩版本，增加了仓库面积、机器人数量、出货口，解决问题能力大大提高
%多机器人协同工作，有碰撞检测机制，显示上加以区分
%增加了充电桩功能

function final()
close all
%用户定义货物与机器人数量
num_of_tasks = 30;     %若干个任务点
robot_num = 10;          %机器人数量
%画图预定义
set(figure,'units','normalized','position',[0.1 0.1 0.8 0.8]);
%函数句柄预定义，比function占用更少的内存
dynamical_draw_handle = @dynamical_draw;
%===============================
%开始程序
[Matrix,Matrix_no_obstacle,n,m] = advance_definition();  %地图，障碍物生成
if(num_of_tasks < robot_num)
    robot_num = num_of_tasks;
end

global tasks;
tasks = zeros(num_of_tasks,5);      %任务栏，前两项为坐标，第三项为出货口，第四项为状态，第五项为被哪个机器人预约（完成任务后该项-1）
%状态：0-未生成或待处理， 1-已被机器人预定但未取走, 2-已预定且刚搬走， 3-已经搬回来（结束）
global fflag;        %用于查看该点是否已有货物

fflag = zeros(m,n);

global robot;
%=======================机器人信息
robot = struct('Spoint',{},'free',{},'first',{},'Epoint',{},'P_sum',{},'carry',{},'end',{},'last_point',{},'prior',{},...
    'battery',{},'temp',{},'low',{},'temp_spoint',{});
for I=1:robot_num
    robot(I).Spoint = [1+I,2];
    
    robot(I).free = 1;
    robot(I).first = 1;
    robot(I).Epoint = zeros(1,2);
    robot(I).P_sum = [];
    robot(I).carry = 3; %3 是空闲机器人，1是货物，2是返回货架
    robot(I).end = -1;
    robot(I).last_point = robot(I).Spoint;
    robot(I).prior = 0;
    robot(I).battery = 600;
    robot(I).temp = zeros(1,3);
    robot(I).low = 0; %显示低电量的状态，比较鸡肋
    robot(I).temp_spoint = zeros(1,2);
    
    %gen1_handle(Matrix,m,n,num_of_tasks);
end
clear I;
%动态生成货物
t=timer;
t.StartDelay = 0;%延时开始
t.ExecutionMode = 'fixedRate';%启用循环执行
t.Period = 0.5;%循环间隔
t.TasksToExecute = num_of_tasks;%循环次数
t.TimerFcn = {@cargo_gen,Matrix}; %传递输入参数
start(t);%开始执行

T=timer;
T.StartDelay = 0;%延时开始
T.ExecutionMode = 'fixedRate';%启用循环执行
T.Period = 0.2;%循环间隔
T.TasksToExecute = 10000;%循环次数
T.TimerFcn = {@find_free_robot,num_of_tasks,m,n,Matrix,Matrix_no_obstacle,robot_num}; %传递输入参数
start(T);%开始执行

%正式开始
while(1)
    empty_index = -1;    %默认机器人都还没跑完
    while(1)
        dynamical_draw_handle(robot_num,num_of_tasks);
        for I=1:robot_num
            [c,~] = size(robot(I).P_sum);
            if(c==0 && robot(I).free == 0 && robot(I).temp(1)~= 0)
                empty_index = I;   %在跑的机器人跑完了
                break;
            elseif(c==1)
                robot(I).last_point = robot(I).P_sum;
            end
        end
        if (empty_index > 0)
            break;       %有机器人跑完了，就跳出这个无限循环，否则一直继续画机器人路径
        end
    end
    robot(empty_index).free = 1;    %刚才的机器人重新有空
    
    if(min(tasks(:,4))~=0)
        %robot(empty_index).P_sum=[robot(empty_index).temp(1),robot(empty_index).temp(2)];
        robot(empty_index).end = 1;
        robot(empty_index).temp = [0,0,0];
    end
    
    if(min(tasks(:,4))==3)
        break;
    end
end

end
%===========================子函数定义============================
function find_free_robot(~,~,num_of_tasks,m,n,Matrix,Matrix_no_obstacle,robot_num)
global robot tasks;
distri_route_handle = @distri_route;
task2robot_handle = @task2robot;
for i=1:num_of_tasks
    if(tasks(i,1)==0)
        break;
    elseif(tasks(i,4)==0 && tasks(i,1)~=0)
        for I=1:robot_num
            if(robot(I).free == 1)  %free=1表示机器人有空，且任务栏有待执行的货物
                task2robot_handle(I,i); %分配任务
            end
            
            if(robot(I).free == 2 && robot(I).temp(1)~=0)
                distri_route_handle(m,n,Matrix,Matrix_no_obstacle,I);
                break;
            end
        end
    end
end
end

function distri_route(m,n,Matrix,Matrix_no_obstacle,I)
findroute_handle = @findroute;
global robot;
[P] = findroute_handle(robot(I).Spoint, robot(I).Epoint,m,n,Matrix_no_obstacle); %从当前位置到货物点
robot(I).Spoint = [robot(I).temp(1),robot(I).temp(2)];        %到了货架点，终点变成了起点
robot(I).Epoint = [6 * robot(I).temp(3),64];
P_0=P;                       %刚才的路径矩阵P调用到P_0中去

[P] = findroute_handle(robot(I).Spoint, robot(I).Epoint, m,n,Matrix);  %拿到货物之后的寻路，用带障碍的矩阵
P_0=[6 * robot(I).temp(3),64;
    6 * robot(I).temp(3),64;
    6 * robot(I).temp(3),64;
    P;P_0];     %将两个路径矩阵连接起来,且出货口需要停留

robot(I).Spoint = [6 * robot(I).temp(3),64];      %要把货架放回去，起点是出货点
robot(I).Epoint = [robot(I).temp(1),robot(I).temp(2)];    %终点是刚才货物点

[P] = findroute_handle(robot(I).Spoint, robot(I).Epoint, m,n,Matrix);   %寻路，起点是刚才放完货架的地方
P=[P;P_0];             %将两个路径矩阵连接起来，用于画

%判断电量，若放一次货物，再到充电桩，加上余量30，小于剩余电量，则认为电量不足
[length,~]=size(P);
if((length+m+n+30) >= robot(I).battery)  %电量不够
    robot(I).low = 1;
    robot(I).Spoint = robot(I).temp_spoint;
    
    if(robot(I).temp(1) < 15)
        robot(I).Epoint = [9,2];
    else
        robot(I).Epoint = [21,2];
    end            %选择充电桩
    
    [P] = findroute_handle(robot(I).Spoint, robot(I).Epoint,m,n,Matrix_no_obstacle); %从当前位置到充电桩
    P_0=[robot(I).Epoint(1),robot(I).Epoint(2);
        robot(I).Epoint(1),robot(I).Epoint(2);
        robot(I).Epoint(1),robot(I).Epoint(2);
        P];
    
    robot(I).Spoint = robot(I).Epoint;
    robot(I).Epoint = [robot(I).temp(1),robot(I).temp(2)];   %终点还是那个任务点
    [P] = findroute_handle(robot(I).Spoint, robot(I).Epoint,m,n,Matrix_no_obstacle); %充个电，仍然是no_obstacle
    P_0=[P;P_0];
    
    robot(I).Spoint = [robot(I).temp(1),robot(I).temp(2)];
    robot(I).Epoint = [6*robot(I).temp(3),64];
    [P] = findroute_handle(robot(I).Spoint, robot(I).Epoint, m,n,Matrix);  %拿到货物之后的寻路，用带障碍的矩阵
    P_0=[6 * robot(I).temp(3),64;
        6 * robot(I).temp(3),64;
        6 * robot(I).temp(3),64;
        P;P_0];
    
    robot(I).Spoint = [6 * robot(I).temp(3),64];      %要把货架放回去，起点是出货点
    robot(I).Epoint = [robot(I).temp(1),robot(I).temp(2)];    %终点是刚才货物点
    [P] = findroute_handle(robot(I).Spoint, robot(I).Epoint, m,n,Matrix);   %寻路，起点是刚才放完货架的地方
    P=[P;P_0];             %将两个路径矩阵连接起来，用于画
end
clear length
%确认路径
robot(I).P_sum=[robot(I).P_sum;flipud(P)];  %倒置矩阵，才能画路径
robot(I).free=0;  %free=0表示任务路径规划完了，正在跑
end

function [Matrix,Matrix_no_obstacle,n,m] = advance_definition()
A=zeros(29,66);
A=uint8(A);
[m,n] = size(A);

for i=3:m-2
    for j=3:n-5
        if((rem(i-3,3)==1 || rem(i-3,3)==2) && (rem(j-3,10)>=1 && rem(j-3,10)<=8))
            A(i,j) = 1;
        else
            A(i,j) = 0;
        end
    end
end
A(1,:) = 2;
A(m,:) = 2;
A(:,1) = 2;
A(:,n) = 2;  %墙壁单独设定

global A_direction;
A_direction=[
0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	;
0	4	8	8	8	8	8	8	8	8	8	8	1	8	8	8	8	8	8	8	8	8	1	8	8	8	8	8	8	8	8	8	1	8	8	8	8	8	8	8	8	8	1	8	8	8	8	8	8	8	8	8	1	8	8	8	8	8	8	8	8	1	1	1	1	0	;
0	5	5	5	5	5	5	5	5	5	5	5	6	5	5	5	5	5	5	5	5	5	6	5	5	5	5	5	5	5	5	5	6	5	5	5	5	5	5	5	5	5	6	5	5	5	5	5	5	5	5	5	6	5	5	5	5	5	5	5	5	5	4	0	3	0	;
0	5	5	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	8	0	3	0	;
0	5	5	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	8	0	3	0	;
0	5	5	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	5	2	3	0	;
0	5	5	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	8	0	3	0	;
0	5	5	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	8	0	3	0	;
0	5	5	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	5	2	3	0	;
0	5	5	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	8	0	3	0	;
0	5	5	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	8	0	3	0	;
0	5	5	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	5	0	3	0	;
0	5	5	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	8	0	3	0	;
0	5	5	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	8	0	3	0	;
0	2	9	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	9	9	2	12	0	;
0	6	6	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	11	7	0	4	0	;
0	6	6	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	11	7	0	4	0	;
0	6	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	6	6	2	4	0	;
0	6	6	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	11	7	0	4	0	;
0	6	6	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	11	7	0	4	0	;
0	6	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	6	6	2	4	0	;
0	6	6	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	11	7	0	4	0	;
0	6	6	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	11	7	0	4	0	;
0	6	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	5	6	9	9	9	9	9	9	9	9	6	6	2	4	0	;
0	6	6	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	10	11	9	9	9	9	9	9	9	9	11	7	0	4	0	;
0	6	6	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	10	11	13	13	13	13	13	13	13	13	11	7	0	4	0	;
0	6	6	6	6	6	6	6	6	6	6	5	6	6	6	6	6	6	6	6	6	5	6	6	6	6	6	6	6	6	6	5	6	6	6	6	6	6	6	6	6	5	6	6	6	6	6	6	6	6	6	5	6	6	6	6	6	6	6	6	6	6	3	0	4	0	;
0	3	7	7	7	7	7	7	7	7	7	1	7	7	7	7	7	7	7	7	7	1	7	7	7	7	7	7	7	7	7	1	7	7	7	7	7	7	7	7	7	1	7	7	7	7	7	7	7	7	7	1	7	7	7	7	7	7	7	7	7	1	1	1	1	0	;
0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	
];
A_direction = uint8(A_direction);


global B;
B = zeros(m,n,3);
B=uint8(B);
for i=1:m                %白色是可以走的路，黑色是障碍物
    for j=1:n
        if(A(i,j)==0)
            B(i,j,:) = 255;    %255->white
        elseif(A(i,j)==2)
            B(i,j,:) = [96,96,96];
        else
            B(i,j,:) = 0;
        end
    end
end

Matrix_no_obstacle = inf;
Matrix_no_obstacle = Matrix_no_obstacle(ones(m,n));
Matrix_no_obstacle(1,:) = -inf;
Matrix_no_obstacle(m,:) = -inf;
Matrix_no_obstacle(:,1) = -inf;
Matrix_no_obstacle(:,n) = -inf;

Matrix = zeros(m,n);
%%向地图添加障障碍
for i = 1:m
    for j=1:n
        if(A(i,j)==1 || A(i,j)==2)
            Matrix(i,j)=-inf;
        else
            Matrix(i,j)=inf;
        end
    end
end
clear i j A
end

function cargo_gen(~,~,Matrix)
global B tasks fflag;
[num,~] = size(tasks);
while(1)
    reg(1)=unidrnd(23)+3;
    reg(2)=unidrnd(60)+3;
    if(Matrix(reg(1),reg(2))~=-inf || fflag(reg(1),reg(2))==1)  %要求生成在货架上，不能产生重叠的任务
        continue;
    else
        break;
    end
end

for i =1:num
    if(tasks(i,1) == 0)
        break;
    end
end

fflag(reg(1),reg(2)) = 1;
%设定出货口
if(reg(1) >=4 && reg(1) <=8)
    reg(3) = 1;
elseif(reg(1) >=10 && reg(1) <=14)
    reg(3) = 2;
elseif(reg(1) >=16 && reg(1) <=20)
    reg(3) = 3;
else
    reg(3) = 4;
end

if(i <= num)
    h = 1:3;
    tasks(i,h) = reg(h);
end

B(reg(1),reg(2),1) = 255;
B(reg(1),reg(2),2) = 0;
B(reg(1),reg(2),3) = 0;
clear reg h i
end

function [P]=findroute(Spoint,Epoint,m,n,Matrix)
%寻路预定义========================
Matrix(Spoint(1),Spoint(2)) = 0;
Matrix(Epoint(1),Epoint(2)) = inf;
G=Matrix;%计算值G
F=Matrix;%F
openlist=Matrix;
closelist=Matrix;
parentx=Matrix;
parenty=Matrix;
openlist(Spoint(1),Spoint(2)) =0;
global A_direction;
%========================route finding=========================
while(1)
    num=inf;%消耗值
    for p=1:m
        for q=1:n
            if(openlist(p,q)==0 && closelist(p,q)~=1)%如果在开启列表并没在关闭列表中
                %Outpoint=[p,q];%第一次 起始点
                if(F(p,q)>=0 && num>F(p,q))% 在开启列表内选择一个最小的F值的节点；num是与最小的f比较，找最小的
                    num=F(p,q);
                    Nextpoint=[p,q];
                end
            end
        end
    end
    clear p q;
    closelist(Nextpoint(1),Nextpoint(2))=1;%起始点进入关闭列表
    kkk=[1,2;
        2,1;
        2,3;
        3,2];
    lanenoright=[1,2;
        2,1;
        2,1;
        3,2];
    lanenodown=[1,2;
        2,1;
        2,3;
        2,1];
    laneup=[1,2;
        1,2;
        1,2;
        1,2];
    lanenoup=[2,1;
        2,1;
        3,2;
        2,3];
    lanenoleft=[1,2;
        2,3;
        2,3;
        3,2];
    laneright=[2,3;
        2,3;
        2,3;
        2,3];
    laneleft=[2,1;
        2,1;
        2,1;
        2,1];
    
    laneleftup=[2,1;
        1,2;
        1,2;
        2,1];
    laneleftdown=[2,1;
        2,1;
        3,2;
        3,2];
    lanedown=[3,2;
        3,2;
        3,2;
        3,2];
    lanerightdown=[2,3;
        2,3;
        3,2;
        3,2];
    lanerightup=[2,3;
        2,3;
        1,2;
        1,2];
    laneupdown=[3,2;
        3,2;
        1,2;
        1,2];
    
    
    for p = 1:4
        A_i = Nextpoint(1);
        A_j = Nextpoint(2);
        
        if (A_direction(A_i,A_j) == 0)
           i=kkk(p,1);
           j=kkk(p,2);
        elseif (A_direction(A_i,A_j) == 1)
           i=laneleft(p,1);
           j=laneleft(p,2);

        elseif (A_direction(A_i,A_j) == 2)
           i=laneright(p,1);
           j=laneright(p,2);

        elseif (A_direction(A_i,A_j) == 3)
           i=laneup(p,1);
           j=laneup(p,2);

        elseif (A_direction(A_i,A_j) == 4)
           i=lanedown(p,1);
           j=lanedown(p,2);

        elseif (A_direction(A_i,A_j) == 5)
           i=lanerightdown(p,1);
           j=lanerightdown(p,2);

        elseif (A_direction(A_i,A_j) == 6)
           i=lanerightup(p,1);
           j=lanerightup(p,2);

        elseif (A_direction(A_i,A_j) == 7)
           i=laneleftup(p,1);
           j=laneleftup(p,2);

        elseif (A_direction(A_i,A_j) == 8)
           i=laneleftdown(p,1);
           j=laneleftdown(p,2);

         elseif (A_direction(A_i,A_j) == 9)
           i=lanenoleft(p,1);
           j=lanenoleft(p,2);

          elseif (A_direction(A_i,A_j) == 10)
           i=lanenoup(p,1);
           j=lanenoup(p,2);

           elseif (A_direction(A_i,A_j) == 11)
           i=lanenodown(p,1);
           j=lanenodown(p,2);

            elseif (A_direction(A_i,A_j) == 12)
           i=laneupdown(p,1);
           j=laneupdown(p,2);
            elseif (A_direction(A_i,A_j) == 13)
           i=lanenoright(p,1);
           j=lanenoright(p,2);
           
        end
        
        %end
        k = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);%周围8个点的G值 和当前点的G值
        if(closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)==1)
            continue;%前点的G值 跳过
        elseif (k == -inf)% 障碍区 不予考虑，并加入到 关闭列表内在于给予考略
            G(Nextpoint(1)-2+i,Nextpoint(2)-2+j) = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
            closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=1;
        elseif (k == inf)% 计算可走区域的值
            distance = abs(i-2)+abs(j-2);
            G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1),Nextpoint(2))+distance;%计算到 下一步的G
            openlist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=0;%把中心点的除去不可达的区域，加入到开放列表内
            
            H=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
            
            F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)+H;%计算到 下一步的F 消耗值
            parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);%记录该点的父节点的x坐标
            parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);%记录该点的父节点的y坐标
        else
            distance = abs(i-2)+abs(j-2);
            if(k>(distance+G(Nextpoint(1),Nextpoint(2))))
                k=distance+G(Nextpoint(1),Nextpoint(2));%如果更新后的g值比原先的小，更新它，说明这一步走对了
                
                H =abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=k+H;%更新F值，并把父节点换成当前的节点
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
            end
        end
        if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))||num==inf)%已把终结的放到开发列表内。在上下方向上
            parentx(Epoint(1),Epoint(2))=Nextpoint(1);
            parenty(Epoint(1),Epoint(2))=Nextpoint(2);
            break;
        end
    end
    if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))||num==inf)
        parentx(Epoint(1),Epoint(2))=Nextpoint(1);
        parenty(Epoint(1),Epoint(2))=Nextpoint(2);
        break;
    end
end
%==============================循环找回溯路径=================================
P=[];
%P=uint8(P);
while(1)
    if(num==inf)%没路径
        break;
    end
    P=[P;Epoint];%把路径的尾节点加入到路径矩阵中
    Epoint=[parentx(Epoint(1),Epoint(2)),parenty(Epoint(1),Epoint(2))];
    if(parentx(Epoint(1),Epoint(2))==Spoint(1) & parenty(Epoint(1),Epoint(2))==Spoint(2))%如果回溯到开始节点 则结束回溯
        P=[P;Epoint];
        break;
    end
end
P=[P;Spoint];
end

function drawmap()
global B;
imagesc(B);
axis square;
set(gca,'XTick',0.5:size(B,2)+0.5,'YTick',0.5:size(B,1)+0.5,...
    'XTickLabel','','YTicklabel','','dataaspect',[1 1 1],...
    'XGrid','on','YGrid','on','GridColor','k','GridAlpha',1)
end

function dynamical_draw(robot_num,num_of_tasks)
clf;
global robot B tasks;
drawmap_handle = @drawmap;
drawmap_handle();
%========================机器人当前位置========================
cmap = flipud(colormap('jet'));
cmap(1,:) = ones(3,1);
cmap(end,:) = zeros(3,1);
colormap(flipud(cmap));
hold on;

for H=1:robot_num
    if(~isempty(robot(H).P_sum))
        %=============显示方式，红绿蓝==============
        if(robot(H).carry==1)
            rectangle('Position',[robot(H).P_sum(1,2)-0.5,robot(H).P_sum(1,1)-0.5,1,1],'facecolor','r');
        elseif(robot(H).carry==2)
            for j=1:num_of_tasks   %找出这个机器人运哪个货物
                if(tasks(j,5) == H)
                    break;
                end
            end
            
            if(robot(H).temp(1)==robot(H).P_sum(1,1) && robot(H).temp(2)==robot(H).P_sum(1,2))
                tasks(j,4) = 3;  %结束程序
                tasks(j,5) = -1; %被哪个机器人预定的状态要清空
            end
            clear j;
            
            rectangle('Position',[robot(H).P_sum(1,2)-0.5,robot(H).P_sum(1,1)-0.5,1,1],'facecolor','b');
        elseif(robot(H).carry==3)
            for i=1:num_of_tasks   %找出这个机器人运哪个货物
                if(tasks(i,5) == H)
                    break;
                end
            end
            
            if(robot(H).temp(1)==robot(H).P_sum(1,1) && robot(H).temp(2)==robot(H).P_sum(1,2))
                tasks(i,4) = 2;
            end
            clear i;
            
            if(robot(H).battery < 200)
                robot(H).low = 1;
            end
            %显示电量
            if(robot(H).low == 0)
                plot(robot(H).P_sum(1,2),robot(H).P_sum(1,1),'go','MarkerSize',5,'LineWidth',6); %电量充足，绿色
            elseif(robot(H).low == 1)
                plot(robot(H).P_sum(1,2),robot(H).P_sum(1,1),'yo','MarkerSize',5,'LineWidth',6); %电量不足，黄色
            end
        end
        hold on;
        
        if(robot(H).temp(1)==robot(H).P_sum(1,1) && robot(H).temp(2)==robot(H).P_sum(1,2)) %如果之前是没货物，跑过去就拿了货物，变红色
            if robot(H).carry == 3
                robot(H).carry = 1;
            elseif robot(H).carry == 2  %如果之前是拿着空货架，那到了任务点就是把空货架放下
                robot(H).carry = 3;
            end
        elseif(robot(H).P_sum(1,1)== 6 * robot(H).temp(3) && robot(H).P_sum(1,2)== 64)
            robot(H).carry = 2;
        end
        
        if((robot(H).P_sum(1,1) == 9 || robot(H).P_sum(1,1) == 21) && robot(H).P_sum(1,2) == 2 && robot(H).low == 1)
            robot(H).battery = 500;
            robot(H).low = 0;
        end
        
        %优先级
        robot(H).prior = robot(H).free * 100 + (66-robot(H).P_sum(1,2)) + abs(14 - robot(H).P_sum(1,1));
    elseif (isempty(robot(H).P_sum) && (robot(H).first==1 || robot(H).last_point(2)~=2))
        plot(robot(H).last_point(1,2),robot(H).last_point(1,1),'go','MarkerSize',5,'LineWidth',6); %电量充足，绿色
    end
end
pause(0.03);

for P=1:num_of_tasks
    if tasks(P,1)==0
        continue;
    elseif tasks(P,1)~=0 && tasks(P,4) <= 1
        B(tasks(P,1),tasks(P,2),:) = [255,0,0];
    elseif tasks(P,1)~=0 && tasks(P,4) == 2
        B(tasks(P,1),tasks(P,2),:) = [255,255,255];
    elseif tasks(P,1)~=0 && tasks(P,4) == 3
        B(tasks(P,1),tasks(P,2),:) = [0,0,255];
    end
end

x=1:4;
B(6*x,64,:)=50;   %出货口配色
B(21,2,:)=[255,255,0];
B(9,2,:)=[255,255,0];  %充电桩
drawmap_handle();
clear x P;

for IND=1:robot_num
    if(~isempty(robot(IND).P_sum))
        %============================
        %碰撞检测
        ind = robot(IND).P_sum(1,:);
        
        if(robot(IND).end == -1)
            robot(IND).P_sum(1,:) = [];  %没跑完，允许删除P矩阵，否则是结束了，要停留在地图上
            robot(IND).battery = robot(IND).battery - 1;    %电量减一
            if(robot(IND).carry == 2 || robot(IND).carry == 3)
                robot(IND).battery = robot(IND).battery - 1;
            end
        end
        
        %假设数值越小，优先级越高
        for L = 1:robot_num
            if(L~=IND)
                [p1,~]=size(robot(IND).P_sum);
                [l1,~]=size(robot(L).P_sum);
                if(p1>1 && l1>1)
                    if(robot(L).P_sum(1,1) == robot(IND).P_sum(1,1) && robot(L).P_sum(1,2) == robot(IND).P_sum(1,2) && robot(IND).prior >= robot(L).prior)
                        robot(IND).P_sum = [ind;robot(IND).P_sum];
                        break;
                    end
                end
            end
        end
        clear p1 l1 L ind
    end
end
end

function task2robot(I,j)
global tasks robot;
if(robot(I).first == 1) %第一次执行起点是随机的，否则就不是了
    robot(I).first = 0;
else
    robot(I).Spoint = [robot(I).temp(1),robot(I).temp(2)];
end
robot(I).temp_spoint = [robot(I).temp(1),robot(I).temp(2)]; %暂时的起点，万一电量不足

robot(I).Epoint = [tasks(j,1),tasks(j,2)];  %终点为第一个未处理的任务点
robot(I).free = 2;                %free=2表示已分配任务，且执行中
robot(I).temp = [tasks(j,1),tasks(j,2),tasks(j,3)];      %当前机器人的当前任务为任务栏的第一个未被处理的
tasks(j,4) = 1;    %已被机器人预定
tasks(j,5) = I;    %被哪个机器人预定
end