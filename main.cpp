#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <limits>
#include <valarray>
#include <fstream>
#include <cmath>
#include <windows.h>

using namespace std;

const int density = 20;// 机器人密度
const double pi = 3.14159; // 圆周率
const double MAX = 99999;//最大值
const double max_v = 5;//最大线速度
const double max_w = 4;//最大角速度
const double min_v = 0;//最小线速度
const double min_w = 0;//最小线速度
const double max_acc_v = 16;//最大线加速度
const double max_acc_w = 10;//最大角加速度
const double dt = 0.02;//时间分辨率 20 ms
const double predict_time = 0.1;//预测时间
const double predict_step = 0.1;
const double goal_tolerance = 0.05; // 到达目标的最大距离
const int max_iterations = 100;//最大迭代次数
const double heading_weight = 1.0; //朝向权重
const double distance_weight = 1.0; // 距离权重
const double velocity_weight = 0.5; // 速度权重

char map[1024]{};// 没啥用的地图
int frame_ID{}; // 帧ID
long int money{}; // 当前金钱
int angle_temp = 1; // 默认惨值改变为-1，默认为1
vector<string> robotOrder_0_trade{};//机器人 0 交易指令集: 输出含 1 个一组
vector<string> robotOrder_1_trade{};//机器人 1 交易指令集: 输出含 1 个一组
vector<string> robotOrder_2_trade{};//机器人 2 交易指令集: 输出含 1 个一组
vector<string> robotOrder_3_trade{};//机器人 3 交易指令集: 输出含 1 个一组

vector<string> robotOrder_0_motion{};//机器人 0 运动指令集：输出含 2 个一组
vector<string> robotOrder_1_motion{};//机器人 1 运动指令集：输出含 2 个一组
vector<string> robotOrder_2_motion{};//机器人 2 运动指令集：输出含 2 个一组
vector<string> robotOrder_3_motion{};//机器人 3 运动指令集：输出含 2 个一组

// 机器人类
struct Robot{
    int workbench_ID{}; // -1：不处于工作台附近 [0，总工作台总数-1]: 工作台按顺序排序，0开始
    int item_ID{}; //携带的物品ID，[1,7],  0 表示未携带物品
    double time_Val{}; //时间价值系数: 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0
    double crash_Val{}; //碰撞价值系数: 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0。
    double angleSpeed{};//单位：弧度/秒。正数：表示逆时针。
    double lineSpeed_X{};//由二维向量描述线速度，单位：米/秒
    double lineSpeed_Y{};//由二维向量描述线速度，单位：米/秒
    double lineSpeed{};
    double angle{}; // 朝向：弧度范围范围[-π,π]。方向示例：0：表示右方向。π/2：表示上方向。-π/2：表示下方向
    double position_X{};//浮点坐标x y
    double position_Y{};//浮点坐标x y
    double r{}; //当前半径 r, 更新半径的时候，记得更新面积
    double mass{}; // 当前质量 = 面积 * 密度
    int targetBench = -2;//当前目的工作台，买卖操作后记得更新,初始是没有目标的，-2是单线的开始，-1是当前待更新
    int status_sellORbuy{};//1卖，0买
    int level = 1;//交易等级，默认是 1
    double angleDis{}; // 与目标机器人的角度差
}robot[5]; // 0-3
//工作台类
struct workbench{
    int type{}; //[1,9] 类型
    double position_X{};//浮点坐标x y
    double position_Y{};//浮点坐标x y
    int time_prodRemaining{}; //-1：表示没有生产。0：表示生产因输出格满而阻塞。>=0：表示剩余生产帧数。
    int status_rawGrid{}; //二进制位表描述，例如 48(110000) 表示拥有物品 4 和 5。
    int status_prodGrid{}; //0：表示无。1：表示有。
    int sum_workbench{}; //当前地图工作台总数 存在workbench[0]中
}workbench[51];//工作台按顺序排序，1开始

// 轨迹的结构体
struct Trajectory {
    vector<double> x;   // 轨迹的x坐标
    vector<double> y;   // 轨迹的y坐标
    vector<double> theta;// 轨迹的角度
    vector<double> v;   // 轨迹的线速度
    vector<double> w;   // 轨迹的角速度
};

// 控制动作
struct ControlAction {
    double v;   // 速度
    double w;   // 角速度
};

bool initMap(); // 读取地图信息
bool Print_robotOrder(); //输出当前帧的指令集，Ok换行结束
bool inline readFrameData();// 获取帧信息
double cal_Dis(double x1, double y1, double x2, double y2);// 计算距离
void Navigation(int robot);//导航
int findBench(int robotID,int sellORbuy, int buytype);//找工作台
void setRobot(int robotID);//设置机器人状态
Trajectory dwaControl(int robotID);//dwa算法
Robot computeRobotState(int robotID, double v, double w);//生成新状态
double computeCost(struct robot, int targetBench);//计算代价
double costFunction_gpt(int robotID, Robot goal_state);//gpt 代价函数


void adjust_Angle(int robotID, double rotate = 2);// 角度调整
void adjust_Speed(int robot);//速度调整
void sell_algorithm(int robot);//卖出策略
void buy_algorithm();//买入策略

int main() {

    //挂载调试

//    Sleep(10000);

    // 设置随机数种子


    initMap();
    puts("OK");
    cout.flush();

    srand(time(NULL));
    while(scanf("%d",&frame_ID) != EOF){
        readFrameData();

        //规划函数
        for (int i = 0; i < 4; ++i) {
            setRobot(i);
        }

        //把数据打印
        ofstream of;
        of.open("C:/Users/86195/Desktop/out.txt",ios::app);
        of << frame_ID << endl ;
        for (int i = 0; i < 4; ++i) {
            of << robot[i].workbench_ID << ends << robot[i].item_ID
               << ends << robot[i].time_Val << ends << robot[i].crash_Val
               << ends << robot[i].angleSpeed
               << ends << robot[i].lineSpeed_X << ends << robot[i].lineSpeed_Y
               << ends << robot[i].angle
               << ends << robot[i].position_X << ends << robot[i].position_Y << ends << robot[i].targetBench <<  ends << robot[i].angleDis <<endl;
        }
        for (int i = 0; i < workbench[50].sum_workbench; ++i) {
            of << i << ends << workbench[i].type << ends <<workbench[i].position_X << ends << workbench[i].position_Y << ends << workbench[i].time_prodRemaining << ends << workbench[i].status_rawGrid << ends << workbench[i].status_prodGrid << endl;
        }

        of << robotOrder_0_motion[0] << endl << robotOrder_0_motion[1] << endl;
        //of << robotOrder_0_trade[0] << endl;

        of << robotOrder_1_motion[0] << endl << robotOrder_1_motion[1] << endl;
        //of << robotOrder_1_trade[0] << endl;

        of << robotOrder_2_motion[0] << endl << robotOrder_2_motion[1] << endl;
        //of << robotOrder_2_trade[0] << endl;

        of << robotOrder_3_motion[0] << endl << robotOrder_3_motion[1] << endl;
        //of << robotOrder_3_trade[0] << endl;

        of.close();

        //输出指令
        Print_robotOrder();
    }
    return 0;
}




/**
  * @brief          : 设置机器人状态
  * @param          : int robotID
  * @retval         :
*/
void setRobot(int robotID) {
    //说明是最起始状态：单线开始咯
    if(robot[robotID].targetBench == -2){
        robot[robotID].targetBench = findBench(robotID,0,1);
    }

    //是否在目标工作台附近
    if(robot[robotID].workbench_ID == robot[robotID].targetBench){
        //要卖
        if(robot[robotID].status_sellORbuy == 1){
            switch (robotID) {
                case 0:
                    robotOrder_0_trade.push_back("sell " + to_string(robotID));
                    break;
                case 1:
                    robotOrder_1_trade.push_back("sell " + to_string(robotID));
                    break;
                case 2:
                    robotOrder_2_trade.push_back("sell " + to_string(robotID));
                    break;
                case 3:
                    robotOrder_3_trade.push_back("sell " + to_string(robotID));
                    break;
                default:
                    break;
            }

            //卖完更新状态，转为要买
            //如果卖的是7，那卖完7重新单线
            if(robot[robotID].item_ID == 7){
                robot[robotID].item_ID = 0;
                robot[robotID].targetBench = -2; //重新开始单线
                robot[robotID].status_sellORbuy = 0;
                robot[robotID].level = 1;
            }else{
                robot[robotID].item_ID = 0;
                robot[robotID].targetBench = -1;
                robot[robotID].status_sellORbuy = 0;
                robot[robotID].level++;
            }

        }
        if(robot[robotID].status_sellORbuy == 0){
            //要买
            if(robot[robotID].status_sellORbuy == 1) {
                switch (robotID) {
                    case 0:
                        robotOrder_0_trade.push_back("buy " + to_string(robotID));
                        break;
                    case 1:
                        robotOrder_1_trade.push_back("buy " + to_string(robotID));
                        break;
                    case 2:
                        robotOrder_2_trade.push_back("buy " + to_string(robotID));
                        break;
                    case 3:
                        robotOrder_3_trade.push_back("buy " + to_string(robotID));
                        break;
                    default:
                        break;
                }
            }
            //卖完更新状态，转为要卖
            robot[robotID].item_ID = workbench[robot[robotID].workbench_ID].type;
            robot[robotID].targetBench = -1;
            robot[robotID].status_sellORbuy = 1;
        }

    }


    //下一步卖
    if (robot[robotID].status_sellORbuy == 1) {
        robot[robotID].targetBench = findBench(robotID, 1, 0);
        Navigation(robotID);
    }
    else {
    //下一步 买
        if (robot[robotID].level == 1) {
            // 买 123
            robot[robotID].targetBench = findBench(robotID,0,1);
            Navigation(robotID);
        }
        if (robot[robotID].level == 2) {
            //买 456
            robot[robotID].targetBench = findBench(robotID,0,2);
            Navigation(robotID);

        }
        if (robot[robotID].level == 3) {
            //买 7
            robot[robotID].targetBench = findBench(robotID,0,3);
            Navigation(robotID);
        }
    }
}




/**
  * @brief          : 读取帧信息
  * @retval         :
*/
bool inline readFrameData() {
    //当前金钱
    cin >> money;
    //工作台总数
    cin >> workbench[50].sum_workbench;
    //读入工作台信息
    for (int i = 0; i < workbench[50].sum_workbench; ++i) {
        cin >> workbench[i].type;
        cin >> workbench[i].position_X >> workbench[i].position_Y;
        cin >> workbench[i].time_prodRemaining >> workbench[i].status_rawGrid >> workbench[i].status_prodGrid;

        //确定当前收购的材料数量
    }
    //读取机器人数据
    for (int j = 0; j < 4; ++j) {
        cin >> robot[j].workbench_ID >> robot[j].item_ID
            >> robot[j].time_Val >> robot[j].crash_Val
            >> robot[j].angleSpeed
            >> robot[j].lineSpeed_X >> robot[j].lineSpeed_Y
            >> robot[j].angle
            >> robot[j].position_X >> robot[j].position_Y;
        //计算线速度大小
        robot[j].lineSpeed = sqrt(robot[j].lineSpeed_X * robot[j].lineSpeed_X + robot[j].lineSpeed_Y * robot[j].lineSpeed_Y);
        //确定当前半径
        if(robot[j].item_ID == 0)   robot[j].r = 0.45;
        else robot[j].r = 0.53;
        //计算质量
        robot[j].mass = pi * robot[j].r * robot[j].r * density;
        //将angle (-pi,pi) 转成(0,2pi)
        if(robot[j].angle < 0 ) robot[j].angle = pi - robot[j].angle;

    }
    string s;
    cin >> s;
    //清空输入缓冲流
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    workbench[50].position_X = 25.0;
    workbench[50].position_Y = 25.0;
    return true;
}

/**
  * @brief          : 读取当前地图数据
  * @retval         :
*/
bool initMap() {
    while (fgets(map, sizeof map, stdin)) {
        if (map[0] == 'O' && map[1] == 'K') {
            return true;
        }
        //do something

    }
    return false;
}



/**
  * @brief          : 输出操作指令
  * @retval         :
*/
bool Print_robotOrder(){

    //输出帧ID
    cout << frame_ID << endl;

    //指令
    //输出机器人 0 运动指令
    cout << robotOrder_0_motion[0] << endl;
    robotOrder_0_motion.erase(robotOrder_0_motion.begin());
    cout << robotOrder_0_motion[0] << endl;
    robotOrder_0_motion.erase(robotOrder_0_motion.begin());

    //输出机器人 1 运动指令
    cout << robotOrder_1_motion[0] << endl;
    robotOrder_1_motion.erase(robotOrder_1_motion.begin());
    cout << robotOrder_1_motion[0] << endl;
    robotOrder_1_motion.erase(robotOrder_1_motion.begin());

    //输出机器人 2 运动指令
    cout << robotOrder_2_motion[0] << endl;
    robotOrder_2_motion.erase(robotOrder_2_motion.begin());
    cout << robotOrder_2_motion[0] << endl;
    robotOrder_2_motion.erase(robotOrder_2_motion.begin());

    //输出机器人 3 运动指令
    cout << robotOrder_3_motion[0] << endl;
    robotOrder_3_motion.erase(robotOrder_3_motion.begin());
    cout << robotOrder_3_motion[0] << endl;
    robotOrder_3_motion.erase(robotOrder_3_motion.begin());

    if(robot[0].workbench_ID == robot[0].targetBench){
        //输出机器人 0 交易指令
        if(!robotOrder_0_trade.empty()){
            cout << robotOrder_0_trade[0] << endl;
            robotOrder_0_trade.erase(robotOrder_0_trade.begin());
        }

    }


    if(robot[1].workbench_ID == robot[1].targetBench){
        //输出机器人 1 交易指令
        if(!robotOrder_1_trade.empty()){
            cout << robotOrder_1_trade[0] << endl;
            robotOrder_1_trade.erase(robotOrder_1_trade.begin());
        }

    }

    if(robot[2].workbench_ID == robot[2].targetBench) {
        //输出机器人 2 交易指令
        if (!robotOrder_2_trade.empty()) {
            cout << robotOrder_2_trade[0] << endl;
            robotOrder_2_trade.erase(robotOrder_2_trade.begin());
        }
    }

        if(robot[3].workbench_ID == robot[3].targetBench) {
            //输出机器人 3 交易指令
            if (!robotOrder_3_trade.empty()) {
                cout << robotOrder_3_trade[0] << endl;
                robotOrder_3_trade.erase(robotOrder_3_trade.begin());
            }

        }

    //输出OK
    cout << "OK" << endl;
    cout.flush();
    //清空robotOrder容器，并回收空间
    //vector <string>().swap(robotOrder);
    return true;
}




/**
  * @brief          : 距离函数
  * @param          : double x1, double y1, double x2, double y2
  * @retval         : 当前距离，double
*/
double cal_Dis(double x1, double y1, double x2, double y2){
    return pow((pow(x1-x2,2) + pow(y1-y2,2)),0.5);
}


/**
  * @brief          : 角度调整 没用咯
  * @param          : 机器人编号、目标工作台编号
  * @retval         :
*/
void adjust_Angle(int robotID, double rotate){
    //机器人和目标点的向量 X Y
    double vector_robotTobenchX = workbench[robot[robotID].targetBench].position_X - robot[robotID].position_X;
    double vector_robotTobenchY = workbench[robot[robotID].targetBench].position_Y - robot[robotID].position_Y;
    //工作台与以机器人为原点的正方向向量
    double vector_positiveX = robot[robotID].position_X;
    //工作台与以机器人为原点的正方向的夹角
    double angle_bench_positive {};
    double cos1 = (vector_robotTobenchX * vector_positiveX)
                  /
                  (pow(vector_robotTobenchX * vector_robotTobenchX + vector_robotTobenchY * vector_robotTobenchY,0.5) * pow(vector_positiveX * vector_positiveX,0.5));
    angle_bench_positive = acos(cos1);
    //角度差
    double angle_dis{};
    angle_dis = robot[robotID].angle -  angle_bench_positive;
    robot[robotID].angleDis = angle_dis;
    if(rotate == 2.0){
        if(angle_dis > 0){
            //需要向右调整角度
            //orderSet.push_back("rotate " + to_string(robotID) + " -2" );
        }else if(angle_dis < 0){
            //需要向左调整角度
            //orderSet.push_back("rotate " + to_string(robotID) + " 2" );
        }else{
            //等于  0， 朝向正确
            //orderSet.push_back("rotate " + to_string(robotID) + " 0");
        }
    }else{
        //非默认传参
        //orderSet.push_back("rotate " + to_string(robotID) + " " + to_string(rotate));
    }


}

/**
  * @brief          : 速度调整 没用咯
  * @param          : 机器人编号
  * @retval         :
*/
void adjust_Speed(int robotID){
    if(robot[robotID].position_X <= 0.8 || robot[robotID].position_X >= 49 || robot[robotID].position_Y <= 0.8 || robot[robotID].position_Y >= 49) {
        //robotOrder.push_back("forward " + to_string(robotID) + " -4");
        angle_temp = - angle_temp;
        if(robot[robotID].angleSpeed > 0){
            //adjust_Angle(robotID,-3);

        }else{
           // adjust_Angle(robotID,3);
        }
    }else{
        //orderSet.push_back("forward " + to_string(robotID) + " 4");
    }

}


/**
  * @brief          : 路线导航
  * @param          : int robot, int targetBench
  * @retval         :
*/
void Navigation(int robotID) {
    /*
    // 生成速度指令
    adjust_Speed(robotID);

    //生成旋转指令
    if(angle_temp == -1) return;
    adjust_Angle(robotID);
*/
    //每个机器人会得到 6 组 最优轨迹的运动参量数据
    Trajectory best_traj = dwaControl(robotID);
    //依次将 v，w 存入指令集，
    switch (robotID) {
        case 0:
            for (int i = 0; i < best_traj.v.size(); ++i) {
                robotOrder_0_motion.push_back("rotate " + to_string(robotID) + " " + to_string(best_traj.w[i]));
                robotOrder_0_motion.push_back("forward " + to_string(robotID) + " " + to_string(best_traj.v[i]));
            }
            break;
        case 1:
            for (int i = 0; i < best_traj.v.size(); ++i) {
                robotOrder_1_motion.push_back("rotate " + to_string(robotID) + " " + to_string(best_traj.w[i]));
                robotOrder_1_motion.push_back("forward " + to_string(robotID) + " " + to_string(best_traj.v[i]));
            }
            break;
        case 2:
            for (int i = 0; i < best_traj.v.size(); ++i) {
                robotOrder_2_motion.push_back("rotate " + to_string(robotID) + " " + to_string(best_traj.w[i]));
                robotOrder_2_motion.push_back("forward " + to_string(robotID) + " " + to_string(best_traj.v[i]));
            }
            break;
        case 3:
            for (int i = 0; i < best_traj.v.size(); ++i) {
                robotOrder_3_motion.push_back("rotate " + to_string(robotID) + " " + to_string(best_traj.w[i]));
                robotOrder_3_motion.push_back("forward " + to_string(robotID) + " " + to_string(best_traj.v[i]));
            }
            break;
    }

}

/**
  * @brief          : 卖出策略
  * @param          : 
  * @retval         : 
*/
void sell_algorithm(int robotID) {}



/**
  * @brief          : 买入策略
  * @param          : 
  * @retval         : 
*/
void buy_algorithm() {}



/**
  * @brief          : 找最近的工作台
  * @param          : int robotID, int typeBench, int sellORbuy 1 sell,0 buy
  * @retval         : 符合条件的 Bench
*/
int findBench(int robotID,int sellORbuy, int buytype) {
    //初始最近距离是与第一个工作台
    double dis = MAX;
    int temp = 0;//初始最近的合适工作台
    if(sellORbuy == 1){
        //卖 找卖家
        if(robot[robotID].item_ID == 1){
            //找能卖 1号物品的4、5号工作台
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                //判断 材料格 没有 1 号商品，即可卖给这个工作台
                if( (workbench[i].type == 4 || workbench[i].type == 5)
                    && workbench[i].status_rawGrid != 2 && workbench[i].status_rawGrid != 6 && workbench[i].status_rawGrid != 10 ){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }
                }
            }
            return temp;
        }
        if (robot[robotID].item_ID == 2){
            //找能卖 2 号物品的4、6号工作台
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                //判断 材料格 没有 2 号商品，即可卖给这个工作台
                if( (workbench[i].type == 4 || workbench[i].type == 6)
                    && workbench[i].status_rawGrid != 4 && workbench[i].status_rawGrid != 6 && workbench[i].status_rawGrid != 12 ){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }

                }
            }
            return temp;
        }
        if(robot[robotID].item_ID == 3){
            //找能卖 2 号物品的5、6号工作台
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                //判断 材料格 没有 3 号商品，即可卖给这个工作台
                if( (workbench[i].type == 5 || workbench[i].type == 6)
                    && workbench[i].status_rawGrid != 8 && workbench[i].status_rawGrid != 10 && workbench[i].status_rawGrid != 12 ){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }

                }
            }
            return temp;
        }
        if(robot[robotID].item_ID == 4){
            //找能卖 4 号物品的 7 号工作台
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                //判断 材料格 没有 4号商品，即可卖给这个工作台
                if( (workbench[i].type == 7)
                    && workbench[i].status_rawGrid != 16 && workbench[i].status_rawGrid != 48 && workbench[i].status_rawGrid != 88 ){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }

                }
            }
            return temp;
        }
        if(robot[robotID].item_ID == 5){
            //找能卖 5 号物品的 7 号工作台
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                //判断 材料格 没有 5号商品，即可卖给这个工作台
                if( (workbench[i].type == 7)
                    && workbench[i].status_rawGrid != 32 && workbench[i].status_rawGrid != 48 && workbench[i].status_rawGrid != 104 ){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }

                }
            }
            return temp;
        }
        if(robot[robotID].item_ID == 6){
            //找能卖 6 号物品的 7 号工作台
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                //判断 材料格 没有 6号商品，即可卖给这个工作台
                if( (workbench[i].type == 7)
                    && workbench[i].status_rawGrid != 72 && workbench[i].status_rawGrid != 88 && workbench[i].status_rawGrid != 104 ){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }

                }
            }
            return temp;
        }
        if(robot[robotID].item_ID == 7){
            //找能卖 7 号物品的 8、9 号工作台
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                //判断 是否是 8 、9
                if( workbench[i].type == 8 || workbench[i].type == 9){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }
                }
            }
            return temp;
        }

    }
    else{
        //买
        if(buytype == 1){
            //找 123
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                if( (workbench[i].type == 1 || workbench[i].type == 2 || workbench[i].type == 3 ) && workbench[i].status_prodGrid == 1){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }
                }
            }
            return temp;
        }
        if(buytype == 2){
            //找 345
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                if( (workbench[i].type == 3 || workbench[i].type == 4 || workbench[i].type == 5 ) && workbench[i].status_prodGrid == 1){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }
                }
            }
            return temp;
        }
        if(buytype == 3){
            //找 7
            for (int i = 1; i < workbench[50].sum_workbench; ++i) {
                if( workbench[i].type == 7 && workbench[i].status_prodGrid == 1){
                    if(cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y) < dis){
                        dis = cal_Dis(robot[robotID].position_X,robot[robotID].position_Y,workbench[i].position_X,workbench[i].position_Y);
                        temp = i;
                    }
                }
            }
            return temp;
        }



    }
    //搜索失败
    return -2;// 重新开始单线，别闲着
}


/**
  * @brief          : 机器人状态函数
  * @param          : 机器人编号，时间分辨率dt
  * @retval         : 机器人结构体
*/
Robot computeRobotState(int robotID, double v, double w){
    Robot newrobot{};
    newrobot.position_X = robot[robotID].position_X + robot[robotID].lineSpeed_X * dt;
    newrobot.position_Y = robot[robotID].position_Y + robot[robotID].lineSpeed_Y * dt;
    newrobot.angle = robot[robotID].angle + robot[robotID].angleSpeed * dt;

    newrobot.lineSpeed = v;
    newrobot.angleSpeed = w;
    return newrobot;
}




/**
  * @brief          : 计算距离代价函数
  * @param          : 机器人编号，目标 x y
  * @retval         : 返回代价
*/
double computeCost(Robot rrobot, int targetBench) {
    //忽略速度代价
    //只考虑 距离代价 、朝向角代价

    double dx = workbench[targetBench].position_X - rrobot.position_X;
    double dy = workbench[targetBench].position_Y - rrobot.position_Y;
    double dtheta = atan2(dy, dx);  // 使用反正切函数计算方向角度

    return sqrt(dx * dx + dy * dy) + 2.0 * dtheta;
}

/**
  * @brief          : DWA算法
  * @param          : 待规划的机器人ID
  * @retval         : 最优方案
*/
Trajectory dwaControl(int robotID){
    Trajectory best_trajectory;
    double best_cost = INFINITY;

    //迭代计算最优轨迹
    for (int i = 0; i < max_iterations; ++i) {

        // 随机生成控制输入
        double v = ((double)rand() / RAND_MAX) * (max_v - min_v) + min_v;
        double w = ((double)rand() / RAND_MAX) * (max_w - min_w) + min_w;
        double acc_v = ((double)rand() / RAND_MAX) * 2.0 * max_acc_v - max_acc_v;
        double acc_w = ((double)rand() / RAND_MAX) * 2.0 * max_acc_w - max_acc_w;

        //轨迹预测
        Trajectory traj;
        Robot current_robot = robot[robotID];
        for (int j = 0; j < predict_time * 1000; ++j) {
            current_robot = computeRobotState(robotID,v,w);
            traj.x.push_back(current_robot.position_X);
            traj.y.push_back(current_robot.position_Y);
            traj.theta.push_back(current_robot.angle);
            traj.v.push_back(current_robot.lineSpeed);
            traj.w.push_back(current_robot.angleSpeed);

            //应用加速度限制
            v += acc_v * dt;
            w += acc_w * dt;
            v = max(min(v,max_v),min_v);
            w = max(min(w,max_w),min_w);
        }
        //计算代价：现在是距离优先，不考虑朝向角代价和速度代价,所以基本是直线最短
        double cost = computeCost(current_robot,robot[robotID].targetBench);
        //更新最优轨迹
        if(cost < best_cost){
            best_trajectory = traj;
            best_cost = cost;
            // 判断是否达到目标
            if (cost < goal_tolerance) {
                break;
            }
        }
    }//
    return best_trajectory;
}



double costFunction_gpt(int robotID, Robot goal_state){
    double heading_cost = heading_weight * abs(robot[robotID].angle- goal_state.angle);
    double distance_cost = distance_weight * sqrt(pow(robot[robotID].position_X- goal_state.position_X, 2) + pow(robot[robotID].position_Y - goal_state.position_Y, 2));
    double velocity_cost = velocity_weight * abs(robot[robotID].lineSpeed - goal_state.lineSpeed);
    return heading_cost + distance_cost + velocity_cost;
}




/*
 *  ofstream of;
        of.open("C:/Users/86195/Desktop/out.txt",ios::app);
        of << frame_ID << ends << money << endl;
        for (int i = 0; i < 4; ++i) {
            of << robot[i].workbench_ID << ends << robot[i].item_ID
                << ends << robot[i].time_Val << ends << robot[i].crash_Val
                << ends << robot[i].angleSpeed
                << ends << robot[i].lineSpeed_X << ends << robot[i].lineSpeed_Y
                << ends << robot[i].angle
                << ends << robot[i].position_X << ends << robot[i].position_Y << endl;
        }
        for (int i = 1; i <= workbench[0].sum_workbench; ++i) {
            of << i << ends <<workbench[i].position_X << ends << workbench[i].position_Y << ends << workbench[i].time_prodRemaining << ends << workbench[i].status_rawGrid << ends << workbench[i].status_prodGrid << endl;
        }
        of.close();

*/

