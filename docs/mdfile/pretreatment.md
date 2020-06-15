# 전처리과정

전처리 과정에는 Set 코드와 Sync 코드가 해당됩니다.

## 1. Set

[vlpt_set_L.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_set_L.cpp)라는 파일을 기준으로 LeftSetVertical이라는 콜백함수를 보겠습니다.

```c

pt_front->x = 0;
pt_back->x = 0;
info_F->ring = 100;
info_B->ring = 100;

for (int j = 0; j < left_input_custom.cpoints.size() ; j++)
{
    if(fabs(left_input_custom.cpoints[j].y - 0.5) < 0.03 && left_input_custom.cpoints[j].x > 1)
    {
        if(info_F->ring >= left_input_custom.infos[j].ring)
        {
            pt_front->x = left_input_custom.cpoints[j].x;
            pt_front->y = left_input_custom.cpoints[j].y;
            pt_front->z = left_input_custom.cpoints[j].z;
            info_F->ring = left_input_custom.infos[j].ring;
        }
    }

    if(fabs(left_input_custom.cpoints[j].y - 0.5) < 0.03 && left_input_custom.cpoints[j].x < -1.5)
    {
        if(info_B->ring >= left_input_custom.infos[j].ring)
        {
            pt_back->x = left_input_custom.cpoints[j].x;
            pt_back->y = left_input_custom.cpoints[j].y;
            pt_back->z = left_input_custom.cpoints[j].z;
            info_B->ring = left_input_custom.infos[j].ring;
        }

    }
}
        
```
이 부분은 처음 임의의 point 변수 pt_front와 pt_back의 x좌표를 0으로 임의의 info 변수 info_f와 info_B의 ring을 100으로 초기화를 해줍니다.
이 임의의 변수들은 Lidar 센서 기준으로 바로 앞에 있는 point와 뒤에 있는 Point를 저장하기 위한 역할을 합니다.
여기서 if문은 Lidar 센서 기준 바로 앞의 point를 찾아내기위한 조건문입니다.

```c

std_pt->x = 0;
std_pt->y = 0;
std_pt->z = CAR_HEIGHT;
if(fabs(pt_front->z - pt_back->z) < 0.02)
{
    std_pt->z = (pt_front->z + pt_back->z) / 2;
}


```

여기서는 바로 앞, 뒤 point와 비교할 기준 point를 저장할 변수를 초기화 해주었습니다.
의미하는 것은 좌표계가 Lidar 센서 기준이므로 센서에서 차량 높이만큼 내려간 point, 쉽게 말하면 센서 바로 아래 point입니다.

```c
float alpha_front_L = -100;
float alpha_back_L = -100;

if(pt_front->x != 0 && pt_back->x != 0)
{
    alpha_front_L = atanf((pt_front->z - std_pt->z) / (pt_front->x - std_pt->x));

    alpha_front_L = alpha_front_L * 180.0 / M_PI;

    alpha_back_L = atanf((std_pt->z - pt_back->z) / (std_pt->x - pt_back->x));

    alpha_back_L = alpha_back_L * 180.0 / M_PI;
}

if(fabs(alpha_front_L - alpha_back_L) < 2 && alpha_front_L != -100)
{
    L_Yd_L = (alpha_front_L + alpha_back_L) / 2;
}
 
if(fabs(alpha_front_L) > 5 || fabs(alpha_back_L) > 5)
{
    L_Yd_L = L_Yd;
}

if(pitch_calibration == false)
{
    L_Yd_L = L_Yd;
}
```
alpha_front_L와 alpha_back_L은 마찬가지로 측정된 기울기를 저장할 변수이며 모두-100으로 초기화해주었습니다.
그 후 첫번째 if문은 pt_front와 pt_back에 값이 들어온다면 기울기를 계산해 alpha 변수에 저장합니다.
두번째 if문은 앞과 뒤의 기울기의 차이가 2도 즉 오차범위 +-2도 안에 앞뒤 기울기가 비슷하고 alpha에 값이 있다면 앞뒤 기울기의 평균값을 변수에 저장합니다.
세번째는 앞뒤 기울기의 차이가 너무 크다면 원래 직접 측정하여 저장해놓은 평균 기울기를 다시 저장합니다. 이 세번째 if문일 경우는 앞에 물체나 오르막이 있을 경우에 대해 대비하였습니다.
네번째는 bool로 지금까지 설명한 알고리즘을 켜고 끌수있습니다.

```c

float C_R_Matrix[3][3];
C_R_Matrix[0][0] = cos(L_Zd*M_PI/180)*cos(L_Yd_L*M_PI/180);
C_R_Matrix[0][1] = -sin(L_Zd*M_PI/180)*cos(L_Xd*M_PI/180) + cos(L_Zd*M_PI/180)*sin(L_Yd_L*M_PI/180)*sin(L_Xd*M_PI/180);
C_R_Matrix[0][2] = sin(L_Zd*M_PI/180)*sin(L_Xd*M_PI/180) + cos(L_Zd*M_PI/180)*sin(L_Yd_L*M_PI/180)*cos(L_Xd*M_PI/180);
C_R_Matrix[1][0] = sin(L_Zd*M_PI/180)*cos(L_Xd*M_PI/180);
C_R_Matrix[1][1] = cos(L_Zd*M_PI/180)*cos(L_Xd*M_PI/180) + sin(L_Zd*M_PI/180)*sin(L_Yd_L*M_PI/180)*sin(L_Xd*M_PI/180);
C_R_Matrix[1][2] = -cos(L_Zd*M_PI/180)*sin(L_Xd*M_PI/180) + sin(L_Zd*M_PI/180)*sin(L_Yd_L*M_PI/180)*cos(L_Xd*M_PI/180);
C_R_Matrix[2][0] = -sin(L_Yd_L*M_PI/180);
C_R_Matrix[2][1] = cos(L_Yd_L*M_PI/180)*sin(L_Xd*M_PI/180);
C_R_Matrix[2][2] = cos(L_Yd_L*M_PI/180)*cos(L_Xd*M_PI/180);
        
```

이제 위에서 결정한 기울기를 Rotation Matrix에 사용하여 point를 보정하였습니다.

이러한 알고리즘을 작성한 이유는 과속방지턱이나 급정지, 급과속과 같은 큰 흔들림 외란때문이었습니다.
장점은 흔들림 없이 땅이라는 것을 수평으로 유지할 수 있고 그로인해 높이로 지면 제거의 정확성을 높일 수 있었습니다.

아래 동영상은 이 알고리즘의 결과입니다.
왼쪽이 적용 전이며, 오른쪽이 적용 후입니다. 사진을 클릭하면 동영상으로 이동합니다.

[<img src="/docs/images/before_image" width="300">](/docs/videos/before.mp4)
[<img src="/docs/images/after_image" width="300">](/docs/videos/after.mp4)

아래 두개는 마찬가지로 적용 전후이고 위에 같은장소를 바로 옆에서 봤을 때입니다.

[<img src="/docs/images/before_image2" width="300">](/docs/videos/before2.mp4)
[<img src="/docs/images/after_image2" width="300">](/docs/videos/after2.mp4)


```c
std::vector<int> save_i_info; ///수직 성분 바뀌기 전에 한번 i정보를 다 저장해주는것.
std::vector<int> save_set_i;
save_set_i.clear();

int set_hori = 1;

for(int i=0; i<left_input_custom.cpoints.size(); i++)
{
    using namespace std;

    save_i_info.push_back(i);





    if( left_input_custom.infos[i].hori != left_input_custom.infos[i+1].hori || i+1 == left_input_custom.cpoints.size() ) ///변화하는 순간이므로. ring을 재 설정해주자.
//                cout<<"changed"<<endl;
    {
        int A[16];

        for(int array_n=0; array_n<16; array_n++)
        {
            A[array_n] = -1;
        }   ///@won 단순한 초기값 정렬
            ///여기서 array_n에는 링의 숫자가 그리고 대입값에는 i값을 넣어야 한다.

        for(int set_n=0; set_n < save_i_info.size(); set_n++)
        {
            A[ left_input_custom.infos[ save_i_info[set_n] ].ring ]
                    = save_i_info[set_n];

            left_input_custom.infos[save_i_info[set_n]].hori = set_hori;
        }

        set_hori++; ///@won 수평 성분 +해준다.

        for(int next_array_n = 0; next_array_n < 16; next_array_n++)
        {
            if(A[next_array_n] != -1)
            save_set_i.push_back( A[next_array_n] );
        }

        save_i_info.clear();
    }

}/// @won 여기를 나오면 save_set_i_push_back은 i가 나열되어있는데 그 값들은 뒤죽박죽이다 그냥 save_set_i 를 순서대로 값들을 넣으면 된다.
```
