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
    {
        int A[16];

        for(int array_n=0; array_n<16; array_n++)
        {
            A[array_n] = -1;
        }   

        for(int set_n=0; set_n < save_i_info.size(); set_n++)
        {
            A[ left_input_custom.infos[ save_i_info[set_n] ].ring ]
                    = save_i_info[set_n];

            left_input_custom.infos[save_i_info[set_n]].hori = set_hori;
        }

        set_hori++;

        for(int next_array_n = 0; next_array_n < 16; next_array_n++)
        {
            if(A[next_array_n] != -1)
            save_set_i.push_back( A[next_array_n] );
        }

        save_i_info.clear();
    }

}
```
이제 본래 Set의 목적인 ring x hori 배열로 정렬하는 부분입니다.
한 A[16]배열의 Data를 전부 -1로 초기화를 하고 Data가 존재하는 ring번째 인덱스만 저장을 합니다.
다음 hori도 마찬가지로 for문으로 적용합니다.
이걸 함으로써 Lidar의 단점인 인식 거리 범위 외의 데이터를 찾아낼 수 있게 됩니다.

```c
velodyne_msgs::custompoint new_my_custom_info;

for(int i=0; i < save_set_i.size(); i++)
{
    velodyne_msgs::cpoint set_point;
    velodyne_msgs::info set_info;
    pcl::PointXYZI pt;

    set_point.x = (C_R_Matrix[0][0]*left_input_custom.cpoints.at(save_set_i[i]).x) + (C_R_Matrix[0][1]*left_input_custom.cpoints.at(save_set_i[i]).y) +
                  (C_R_Matrix[0][2]*left_input_custom.cpoints.at(save_set_i[i]).z);

    set_point.y = (C_R_Matrix[1][0]*left_input_custom.cpoints.at(save_set_i[i]).x) + (C_R_Matrix[1][1]*left_input_custom.cpoints.at(save_set_i[i]).y) +
                  (C_R_Matrix[1][2]*left_input_custom.cpoints.at(save_set_i[i]).z);

    set_point.z = (C_R_Matrix[2][0]*left_input_custom.cpoints.at(save_set_i[i]).x) + (C_R_Matrix[2][1]*left_input_custom.cpoints.at(save_set_i[i]).y) +
                  (C_R_Matrix[2][2]*left_input_custom.cpoints.at(save_set_i[i]).z);
    set_point.intensity = left_input_custom.cpoints.at(save_set_i[i]).intensity;

    set_info = left_input_custom.infos.at( save_set_i[i] );

    /// @won Right Frame
    if(set_point.x <= L_PX && set_point.x >=L_MX && set_point.y <=L_PY && set_point.y >= L_MY && set_point.z <=L_PZ && set_point.z >=L_MZ )
    {}
    else
    {
        if(set_point.z <= -2.2)
        {}
        else {
            new_my_custom_info.infos.push_back(set_info);
            new_my_custom_info.cpoints.push_back(set_point);
        }
        pt.x = set_point.x;
        pt.y = set_point.y;
        pt.z = set_point.z;
        pt.intensity = set_point.intensity;
        pcl_output->push_back(pt);
    }

}
```
마지막으로 아까의 Rotation Matrix를 모든 point의 xyz에 적용을 하고 ring과 hori 정보, Data까지 저장을 합니다. 
이 과정중 if문이 있습니다.
이 if문에는 차에 위치한 point나 지면에 반사되는 물체가 생겼을때 땅 속으로 point가 생기는 것을 방지하기 위해 차량의 규격범위와 센서로부터 -2.2m 차량보다 조금 더 아래에 위치한 point는 제외하였습니다.
이것을 마지막으로 이 data들을 다음 Sync Center로 보내줍니다.

## 2. Sync Center & Sync

[vlpt_sync_center.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_sync_center.cpp)라는 파일을 기준으로 설명하겠습니다.

알고리즘을 설명하기에 앞서 subscribe하는 정보부터 설명드리겠습니다.
앞부분 코드를 보면
```c
sub_left = nh->subscribe(sub_center_left,1 , &vlp_::Left_VLP_CB,this );
sub_right = nh->subscribe(sub_center_right,1 , &vlp_::Right_VLP_CB,this );
sub_top = nh->subscribe(sub_center_top,1 , &vlp_::Top_VLP_CB,this );
sub_vehicle = nh->subscribe(sub_center_vehicle,10 , &vlp_::Vehicle_State_CB,this );
```
subscribe를 받는 변수가 있습니다. 이것은 위에서부터 각각 왼쪽,오른쪽,위쪽의 Set data와 차량데이터 Vehicle_State를 받습니다.
Set data는 10Hz로 0.1초마다 data를 받고 Vehicle_State data는 100Hz로 0.01초마다 데이터를 받습니다. 이러한 시간 차이로 Lidar기반으로 고속주행시 센서오차를 보정하였습니다.

Vehicle_State_CB라는 콜백함수를 먼저 보겠습니다.

```c

pharos_vlp_tilt::VehiclePosePtr vehicle(new pharos_vlp_tilt::VehiclePose);

bicycle_model(vehicle->x, vehicle->y, vehicle->theta ,input->state.velocity, input->state.wheel_angle, 0.01);

```
콜백함수는 data가 들어올때마다 실행되기때문에 0.01초마다 아래 설명하는 알고리즘이 실행됩니다.
처음으로 VehiclePos는 차량 정보를 다른 Node로 보내주기 위해 따로 Custom msg를 만들었습니다.
따라서 vehicle의 변수는 차량정보들을 저장할 변수입니다.
다음 bicycle_model모델 함수 입니다. 이 함수의 내부는

```c
void bicycle_model (double &x, double &y, double &theta,
                    const double &vel, const double &wheel_angle, const double &dt)
{
    double ratio = (steer_const - steer_bias) * 3 / M_PI * wheel_angle *steer_const + steer_bias;

    double d = vel*dt;
    double beta = d / L * tan(wheel_angle*steer_const/ratio);

    x = d * cos(theta);
    y = d * sin(theta);

    theta = beta;
    double x_dot = -d * sin(theta);
    double y_dot = d * cos(theta);
}
```
이렇게 되어있습니다. 이 함수의 인수는 저장할 공간인 변수와 차량데이터의 속도, 휠각도,움직인 시간을 입력하여 차량의 운동방정식을 이용하여 이동한 거리 휠 각도를 산출해냅니다.
```c
vehicle->stamp.data = input->header.stamp;
```
그 후 그 차량정보의 시간까지 저장을 해줍니다.

```c
if(vehicle_vec->vehicles.size() == 0)
{
    for(int i=0;i<vehicle_stack;i++)
    {
        vehicle_vec->vehicles.push_back(*vehicle);
    }
}

vehicle_vec->vehicles.push_back(*vehicle);
```
이 부분은 자율주행자동차가 처음 주행 시작하는 경우 전역변수인 vehicle_vec 즉 차량정보의 vector변수를 초기 차량정보로 가득 채운다는 의미입니다.
vehicle_stack은 임의로 정하는 것이지만 저는 25개로 정하였습니다.

에러메시지는 따로 설명하지 않고 넘어가겠습니다.

```c
if(vehicle_vec->vehicles.size() > vehicle_stack)
{
    isVehicleInit = true;

    vehicle_vec->vehicles.erase(vehicle_vec->vehicles.begin() + 0);
}
```
여기는 차량정보의 vector변수가 vehicle_stack 즉 25개가 넘지않게 조절해주는 역할을 합니다.

