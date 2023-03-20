#include <iostream>
#include <cstdio>
#include <vector>

using namespace std;

char map[1024]{};
int frame_ID{}; // ֡ID
long int money{}; // ��ǰ��Ǯ
vector<string> robotOrder{};//������ָ�

// ��������
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
//����̨��
struct workbench{
    int ID{}; //[1,9]
    float position_X{};//��������x y
    float position_Y{};//��������x y
    int time_prodRemaining{}; //-1����ʾû��������0����ʾ���������������������>=0����ʾʣ������֡����
    int status_rawGrid{}; //������λ������������ 48(110000) ��ʾӵ����Ʒ 4 �� 5��
    int status_prodGrid{}; //0����ʾ�ޡ�1����ʾ�С�
    int sum_workbench{}; //��ǰ��ͼ����̨���� ����workbench[0]��
}workbench[51];

bool initMap(); // ��ȡ��ͼ��Ϣ
bool Print_robotOrder(); //�����ǰ֡��ָ���Ok���н���
bool readFrameData();// ��ȡ֡��Ϣ

int main() {
    initMap();
    puts("OK");
    //fflush(stdout);
    cout << flush;

    //���Բ���
    robotOrder.push_back("forward 0 4");
    robotOrder.push_back("rotate 0 3.14159");
    robotOrder.push_back("forward 1 4");
    robotOrder.push_back("rotate 0 3.14159");
    robotOrder.push_back("forward 2 4");
    robotOrder.push_back("rotate 0 3.14159");
    robotOrder.push_back("forward 3 4");
    robotOrder.push_back("rotate 3 3.14159");

    while(scanf("%d",&frame_ID) != EOF){
        //����֡��Ϣ
        readFrameData();

        //�滮����

        //���ָ��
        Print_robotOrder();
    }
    return 0;
}


/**
  * @brief          : ��ȡ֡��Ϣ
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
    while(i < workbench[0].sum_workbench);

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
  * @brief          : �������ָ��
  * @retval         :
*/
bool Print_robotOrder(){
    //���֡ID
    cout << frame_ID << endl;
    //ָ��map
    for (auto & iter : robotOrder) {
        cout << iter << endl;
    }
    //���OK
    printf("OK\n");
    //ˢ��stream��
    fflush(stdout);
    return true;
}