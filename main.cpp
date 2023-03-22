#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <limits>
#include <valarray>
#include <fstream>
#include <cmath>
using namespace std;

const int density = 20;// 机器人密度
const double pi = 3.14159; // 圆周率

char map[1024]{};
int frame_ID{}; // 帧ID
long int money{}; // 当前金钱
vector<string> robotOrder{};//机器人指令集

// 机器人类
struct robot{
    int workbench_ID{}; // -1：不处于工作台附近 [0-8]: 工作台按顺序排序，0开始
    int item_ID{}; //携带的物品ID，[0,7]:  0 表示未携带物品
    double time_Val{}; //时间价值系数: 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0
    double crash_Val{}; //碰撞价值系数: 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0。
    double angleSpeed{};//单位：弧度/秒。正数：表示逆时针。
    double lineSpeed_X{};//由二维向量描述线速度，单位：米/秒
    double lineSpeed_Y{};//由二维向量描述线速度，单位：米/秒
    double angle{}; // 朝向：弧度范围范围[-π,π]。方向示例：0：表示右方向。π/2：表示上方向。-π/2：表示下方向
    double position_X{};//浮点坐标x y
    double position_Y{};//浮点坐标x y
    double r{}; //当前半径 r, 更新半径的时候，记得更新面积
    double mass{}; // 当前质量 = 面积 * 密度
    int targetBench{};//当前目的工作台，买卖操作后记得更新
}robot[5];
//工作台类
struct workbench{
    int type{}; //[1,9] 类型
    double position_X{};//浮点坐标x y
    double position_Y{};//浮点坐标x y
    int time_prodRemaining{}; //-1：表示没有生产。0：表示生产因输出格满而阻塞。>=0：表示剩余生产帧数。
    int status_rawGrid{}; //二进制位表描述，例如 48(110000) 表示拥有物品 4 和 5。
    int status_prodGrid{}; //0：表示无。1：表示有。
    int sum_workbench{}; //当前地图工作台总数 存在workbench[0]中
}workbench[51];//工作台按顺序排序，0开始

bool initMap(); // 读取地图信息
bool Print_robotOrder(); //输出当前帧的指令集，Ok换行结束
bool inline readFrameData();// 获取帧信息
double cal_Dis(double x1, double y1, double x2, double y2);// 距离
void angle_Adjust(int robot, int targetBench);// 角度调整
void speed_Adjust(int robot);//速度调整
void Navigation(int robot, int targetBench);//导航
void sell_algorithm();//卖出策略
void buy_algorithm(int robot);//买入策略
int findBench(int robotID);//找工作台

int main() {
    initMap();
    puts("OK");
    cout.flush();

    //测试操作
    robotOrder.emplace_back("forward 0 4");
    robotOrder.emplace_back("forward 0 4");
    robotOrder.emplace_back("forward 0 5");

    /*
    robotOrder.emplace_back("rotate 0 3.14159");
    robotOrder.emplace_back("forward 1 4");
    robotOrder.emplace_back("rotate 0 3.14159");
    robotOrder.emplace_back("forward 2 4");
    robotOrder.emplace_back("rotate 0 3.14159");
    robotOrder.emplace_back("forward 3 4");
    robotOrder.emplace_back("rotate 3 3.14159");
     */

    while(scanf("%d",&frame_ID) != EOF){
        readFrameData();
        //规划函数


        //把在判题器帧数据打印
        ofstream of;
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


        //输出指令
        Print_robotOrder();
    }
    return 0;
}


/**
  * @brief          : 读取帧信息
  * @retval         :
*/
bool inline readFrameData() {
    //当前金钱
    cin >> money;
    //工作台总数
    cin >> workbench[0].sum_workbench;
    //读入工作台信息
    for (int i = 1; i <= workbench[0].sum_workbench; ++i) {
        cin >> workbench[i].type;
        cin >> workbench[i].position_X >> workbench[i].position_Y;
        cin >> workbench[i].time_prodRemaining >> workbench[i].status_rawGrid >> workbench[i].status_prodGrid;
    }
    //读取机器人数据
    for (int j = 0; j < 4; ++j) {
        cin >> robot[j].workbench_ID >> robot[j].item_ID
            >> robot[j].time_Val >> robot[j].crash_Val
            >> robot[j].angleSpeed
            >> robot[j].lineSpeed_X >> robot[j].lineSpeed_Y
            >> robot[j].angle
            >> robot[j].position_X >> robot[j].position_Y;
        //确定当前半径
        if(robot[j].item_ID == 0)   robot[j].r = 0.53;
        else robot[j].r = 0.45;

        //计算质量
        robot[j].mass = pi * robot[j].r * robot[j].r * density;

    }
    string s;
    cin >> s;
    //清空输入缓冲流
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
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
    for (auto & iter : robotOrder) {
        cout << iter << endl;
    }
    //输出OK
    cout << "OK" << endl;
    cout.flush();
    //清空robotOrder容器，并虎回收空间
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
  * @brief          : 角度调整
  * @param          : 机器人编号、目标工作台编号
  * @retval         :
*/
void angle_Adjust(int robotID, int targetBenchID){
    //机器人和目标点的向量 X Y
    double vector_robotTobenchX = workbench[targetBenchID].position_X - robot[robotID].position_X;
    double vector_robotTobenchY = workbench[targetBenchID].position_Y - robot[robotID].position_Y;
    //工作台与以机器人为原点的正方向向量
    double vector_positiveX = 10.0;
    //工作台与以机器人为原点的正方向的夹角
    double angle_bench_positive {};
    double cos1 = (vector_robotTobenchX * vector_positiveX)
                  /
                  (pow(vector_robotTobenchX * vector_robotTobenchX + vector_robotTobenchY * vector_robotTobenchY,0.5) * pow(vector_positiveX * vector_positiveX,0.5));
    angle_bench_positive = acos(cos1);
    //角度差
    double angle_dis{};
    angle_dis = robot[robotID].angle -  angle_bench_positive;
    if(angle_dis > 0){
        //需要向右调整角度
        robotOrder.push_back("rotate " + to_string(robotID) + " 2");

    }else if(angle_dis < 0){
        //需要向左调整角度
        robotOrder.push_back("rotate " + to_string(robotID) + " -2");
    }else{
        //等于  0， 朝向正确
    }

}

/**
  * @brief          : 速度调整
  * @param          : 机器人编号
  * @retval         :
*/
void speed_Adjudt(int robotID){
    robotOrder.push_back("forward " + to_string(robotID) + "6");
}


/**
  * @brief          : 路线导航
  * @param          : int robot, int targetBench
  * @retval         :
*/
void Navigation(int robot, int targetBench) {
    angle_Adjust(robot,targetBench);
    speed_Adjudt(robot);
}

/**
  * @brief          : 卖出策略
  * @param          : 
  * @retval         : 
*/
void sell_algorithm(int robotID) {
    switch(robot[robotID].item_ID) {
        case 1:
            findBench();
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            break;
        case 6:
            break;
        case 7:
            break;
    }


}



/**
  * @brief          : 买入策略
  * @param          : 
  * @retval         : 
*/
void buy_algorithm() {

}

/**
  * @brief          : 找最近的工作台
  * @param          : int robotID, int typeBench
  * @retval         : 符合条件的Bench
*/
int findBench(int robotID, int typeBench) {
    if(typeBench == 4)
    for (int i = 0; i < workbench[0].sum_workbench; ++i) {
        if(workbench[i].type == typeBench && ){

        }
    }
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
        of.close();*/

