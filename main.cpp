#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <limits>

using namespace std;

char map[1024]{};
int frame_ID{}; // 帧ID
long int money{}; // 当前金钱
vector<string> robotOrder{};//机器人指令集

// 机器人类
struct robot{
    int workbench_ID{}; // -1：不处于工作台附近 [0-8]: 工作台按顺序排序，0开始
    int item_ID{}; //携带的物品ID，[0,7]:  0 表示未携带物品
    float time_Val{}; //时间价值系数: 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0
    float crash_Val{}; //碰撞价值系数: 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0。
    float angleSpeed{};//单位：弧度/秒。正数：表示逆时针。
    float lineSpeed_X{};//由二维向量描述线速度，单位：米/秒
    float lineSpeed_Y{};//由二维向量描述线速度，单位：米/秒
    float angle{}; // 朝向：弧度范围范围[-π,π]。方向示例：0：表示右方向。π/2：表示上方向。-π/2：表示下方向
    float position_X{};//浮点坐标x y
    float position_Y{};//浮点坐标x y
}robot[5];
//工作台类
struct workbench{
    int type{}; //[1,9] 类型
    float position_X{};//浮点坐标x y
    float position_Y{};//浮点坐标x y
    int time_prodRemaining{}; //-1：表示没有生产。0：表示生产因输出格满而阻塞。>=0：表示剩余生产帧数。
    int status_rawGrid{}; //二进制位表描述，例如 48(110000) 表示拥有物品 4 和 5。
    int status_prodGrid{}; //0：表示无。1：表示有。
    int sum_workbench{}; //当前地图工作台总数 存在workbench[0]中
}workbench[51];

bool initMap(); // 读取地图信息
bool Print_robotOrder(); //输出当前帧的指令集，Ok换行结束
bool inline readFrameData();// 获取帧信息

int main() {
    initMap();
    puts("OK");
    cout.flush();

    //测试操作
    robotOrder.emplace_back("forward 0 4");
    robotOrder.emplace_back("rotate 0 3.14159");
    robotOrder.emplace_back("forward 1 4");
    robotOrder.emplace_back("rotate 0 3.14159");
    robotOrder.emplace_back("forward 2 4");
    robotOrder.emplace_back("rotate 0 3.14159");
    robotOrder.emplace_back("forward 3 4");
    robotOrder.emplace_back("rotate 3 3.14159");

    while(scanf("%d",&frame_ID) != EOF){
        readFrameData();
        //规划函数

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
    return true;
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