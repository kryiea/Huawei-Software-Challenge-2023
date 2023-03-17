#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>
#include <fstream>

using namespace std;

//char map[105][105]{};
char map[1024]{};
int frame_ID{}; // ֡ID
long int money{};
unordered_map<string, string> robotOrder;


struct robot{
    int workbench_ID{}; // -1�������ڹ���̨���� [0-8]: ����̨��˳������0��ʼ
    int item_ID{}; //Я������ƷID��[0,7]:  0 ��ʾδЯ����Ʒ
    float time_Val{}; //ʱ���ֵϵ��: Я����ƷʱΪ[0.8, 1]�ĸ���������Я����ƷʱΪ 0
    float crash_Val{}; //��ײ��ֵϵ��: Я����ƷʱΪ[0.8, 1]�ĸ���������Я����ƷʱΪ 0��
    float angleSpeed{};//��λ������/�롣��������ʾ��ʱ�롣
    float lineSpeed_X{};//�ɶ�ά�����������ٶȣ���λ����/��
    float lineSpeed_Y{};//�ɶ�ά�����������ٶȣ���λ����/��
    float angle{}; // ���򣺻��ȷ�Χ��Χ[-��,��]������ʾ����0����ʾ�ҷ��򡣦�/2����ʾ�Ϸ���-��/2����ʾ�·���
    float position_X{};//��������x y
    float position_Y{};//��������x y
}robot[5];
struct workbench{
    int ID{}; //[1,9]
    float position_X{};//��������x y
    float position_Y{};//��������x y
    int time_prodRemaining{}; //-1����ʾû��������0����ʾ���������������������>=0����ʾʣ������֡����
    int status_rawGrid{}; //������λ������������ 48(110000) ��ʾӵ����Ʒ 4 �� 5��
    int status_prodGrid{}; //0����ʾ�ޡ�1����ʾ�С�
    int sum_workbench{};
}workbench[10];

bool readMap(); // ��ȡ��ͼ��Ϣ
bool Print_robotOrder(unordered_map<string, string> robotOrder); //�����ǰ֡��ָ���Ok���н���
bool readFrameData();// ��ȡ֡��Ϣ

int main() {
    readMap();
    puts("OK");
    fflush(stdout);

    while(scanf("%d",&frame_ID) != EOF){
        //printf("%d\n", frame_ID);

        //����֡��Ϣ
        readFrameData();

        ofstream ofs;
        ofs.open("output.txt",ios::app);
        for (int j = 0; j < 7; ++j) {
            ofs << robot[j].workbench_ID << " " << robot[j].item_ID<< " "
                << robot[j].time_Val << " "<< robot[j].crash_Val<< " "
                << robot[j].angleSpeed<< " "
                << robot[j].lineSpeed_X<< " " << robot[j].lineSpeed_Y<< " "
                << robot[j].angle<< " "
                << robot[j].position_X << " "<< robot[j].position_Y<< " ";
        }
         ofs.close();
        //�滮����
        printf("OK\n");
        fflush(stdout);
    }

    return 0;
}


/**
  * @brief          : ��ȡ֡��Ϣ
  * @note           : 
  * @param          : 
  * @retval         : 
*/
bool readFrameData() {
    cin >> money;
    //��ȡ����̨����
    cin >> workbench[0].sum_workbench;
    int  i {};
    do{
        i = 0;
        cin >> i;
        cin >> workbench[i].position_X >> workbench[i].position_Y;
        cin >> workbench[i].time_prodRemaining >> workbench[i].status_rawGrid >> workbench[i].status_prodGrid;
    }
    while(i <= workbench[0].sum_workbench);

    //��ȡ����������
    for (int j = 0; j < 4; ++j) {
        cin >> robot[j].workbench_ID >> robot[j].item_ID
            >> robot[j].time_Val >> robot[j].crash_Val
            >> robot[j].angleSpeed
            >> robot[j].lineSpeed_X >> robot[j].lineSpeed_Y
            >> robot[j].angle
            >> robot[j].position_X >> robot[j].position_Y;
    }


    //��ȡOK
    string s;
    cin >> s;
    if("OK" == s) return true;
    return false;
}

/**
  * @brief          : ��ȡ��ǰ��ͼ����
  * @note           : 
  * @param          :
  * @retval         : 
*/
bool readMap() {
    while (fgets(map, sizeof map, stdin)) {
        if (map[0] == 'O' && map[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}


/**
  * @brief          : �������ָ��
  * @note           :
  * @param          :
  * @retval         :
*/
bool Print_robotOrder(unordered_map<string, string> robotOrder){
    return true;
}