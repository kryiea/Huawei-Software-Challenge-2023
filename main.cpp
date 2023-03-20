#include <iostream>
#include <cstdio>
#include <vector>

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
    int ID{}; //[1,9]
    float position_X{};//浮点坐标x y
    float position_Y{};//浮点坐标x y
    int time_prodRemaining{}; //-1：表示没有生产。0：表示生产因输出格满而阻塞。>=0：表示剩余生产帧数。
    int status_rawGrid{}; //二进制位表描述，例如 48(110000) 表示拥有物品 4 和 5。
    int status_prodGrid{}; //0：表示无。1：表示有。
    int sum_workbench{}; //当前地图工作台总数 存在workbench[0]中
}workbench[51];

bool initMap(); // 读取地图信息
bool Print_robotOrder(); //输出当前帧的指令集，Ok换行结束
bool readFrameData();// 获取帧信息

int main() {
    initMap();
    puts("OK");
    //fflush(stdout);
    cout << flush;

    //测试操作
    robotOrder.push_back("forward 0 4");
    robotOrder.push_back("rotate 0 3.14159");
    robotOrder.push_back("forward 1 4");
    robotOrder.push_back("rotate 0 3.14159");
    robotOrder.push_back("forward 2 4");
    robotOrder.push_back("rotate 0 3.14159");
    robotOrder.push_back("forward 3 4");
    robotOrder.push_back("rotate 3 3.14159");

    while(scanf("%d",&frame_ID) != EOF){
        //读入帧信息
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
bool readFrameData() {
    cin >> money;
    //读取工作台数据
    cin >> workbench[0].sum_workbench;
    int  i {};
    do{
        i = 0;
        cin >> i;
        cin >> workbench[i].position_X >> workbench[i].position_Y;
        cin >> workbench[i].time_prodRemaining >> workbench[i].status_rawGrid >> workbench[i].status_prodGrid;
    }
    while(i < workbench[0].sum_workbench);

    //读取机器人数据
    for (int j = 0; j < 4; ++j) {
        cin >> robot[j].workbench_ID >> robot[j].item_ID
            >> robot[j].time_Val >> robot[j].crash_Val
            >> robot[j].angleSpeed
            >> robot[j].lineSpeed_X >> robot[j].lineSpeed_Y
            >> robot[j].angle
            >> robot[j].position_X >> robot[j].position_Y;
    }

    //读取OK
    string s;
    cin >> s;
    if("OK" == s) return true;
    return false;
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
    //指令map
    for (auto & iter : robotOrder) {
        cout << iter << endl;
    }
    //输出OK
    printf("OK\n");
    //刷新stream流
    fflush(stdout);
    return true;
}