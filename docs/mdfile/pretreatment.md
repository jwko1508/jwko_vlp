[목차](/README.md)          |           [Previous](/docs/mdfile/algorithm_abstract.md)
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
이렇게 되어있고, 아래 그림을 참고하였습니다.

<img src="/docs/images/bicycle_model" width="200">

이 함수의 인수는 저장할 공간인 변수와 차량데이터의 속도, 휠각도,움직인 시간을 입력하여 차량의 운동방정식을 이용하여 이동한 거리 휠 각도를 산출해냅니다.
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

```c
Eigen::Matrix2d rotation2Dmatrix;
rotation2Dmatrix << cos(vehicle_vec->vehicles.back().theta), -sin(vehicle_vec->vehicles.back().theta),
        sin(vehicle_vec->vehicles.back().theta), cos(vehicle_vec->vehicles.back().theta);

for(int i=0; i<vehicle_vec->vehicles.size(); i++)
{
    vehicle_vec->vehicles[i].x -= vehicle_vec->vehicles.back().x;

    Eigen::Vector2d position;
    position.x() = vehicle_vec->vehicles[i].x;
    position.y() = vehicle_vec->vehicles[i].y;

    position = rotation2Dmatrix * position;

    vehicle_vec->vehicles[i].x = position.x();
    vehicle_vec->vehicles[i].y = position.y();

    vehicle_vec->vehicles[i].theta -= vehicle_vec->vehicles.back().theta;
}
```

Rotation Matrix는 쌓아놓은 vehicle_vec 안의 정보들 중 가장 마지막 인덱스, 즉 가장 최근 정보의 좌표계 기준으로 모든 정보를 평행 이동해주는 회전 행렬입니다.
for문안의 내용은 모든정보를 평행이동시켜주는 과정입니다.

위 설명하였던 100hz 차량정보를 subscribe 될 때 실행되는 Vehicle_State_CB와 다르게 
왼쪽, 오른쪽, 가운데 Lidar 정보가 subscribe 될 때 Left_VLP_CB, Right_VLP_CB, Top_VLP_CB라는 콜백함수도 있습니다.

Left_VLP_CB, Right_VLP_CB는 단순히 왼쪽과 오른쪽의 정보가 들어왔는지 아닌지만 확인하기 위해 만들었습니다.

셋 중 가장 중요한 Top_VLP_CB 입니다. 이유는 모든 point 하나하나를 전부 가운데 Lidar의 마지막 point 기준의 시간으로 바꾸려고 하기 때문입니다.

```c
if(!isLeftInit || !isRightInit || !isVehicleInit)
{
    return;
}
```
처음은 왼쪽, 오른쪽, 차량정보가 subcribe 되어있지 않으면 return한다는 단순한 내용입니다.

```c
pharos_vlp_tilt::VehiclePoseArrayPtr store_vehicle(new pharos_vlp_tilt::VehiclePoseArray);

store_vehicle = vehicle_vec;

store_vehicle->header.stamp = input.header.stamp;

pub_for_sync.publish(store_vehicle);
```

위는 단순히 차량정보를 다른 변수에 저장을 합니다. 하지만 중요한 것은 가운데 Lidar가 들어온 시간을 header에 저장을 한다는 것입니다.
이 과정이 무엇보다 중요합니다.

여기까지 Sync Center에 관해 설명하였습니다.

다음은 [vlpt_sync_L.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_sync_L.cpp)라는 파일을 기준으로 설명하겠습니다.

처음 전역변수를 설명하지 않으면 이해를 못할 수 있기 때문에 이번에는 전역변수부터 설명하겠습니다.

```c
double diff_value;
bool isTimeLeft;
bool isErrorHori = false;
bool isLeftInit = false;

float L_velodyne_x , L_velodyne_y , L_velodyne_z;

std::vector<velodyne_msgs::custompoint> global_left;
```
diff_value, isTimeLeft 이 두개는 단순히 이 처리과정이 얼마나 걸리는지 시간을 측정하고 가시화시키기 위한 전역변수입니다.
isErrorHori는 제대로된 point 정보가 들어있지 않는 것을 편하게 구분하기 위하여 선언하였습니다.
isLeftInit는 Sync를 가운데 Lidar(Top_velodyne) 기준으로 하기 때문에 가운데 Lidar의 정보가 들어오기전에 왼쪽 Lidar(Left_velodyne)의 point 정보가 반드시 존재하여야 합니다. 따라서 이 왼쪽 라이다의 정보가 존재하는지 하지 않는지 구분하기 위해 선언하였습니다.
L_velodyne_x,y,z는 이 자율주행차의 차축, 즉 모든 frame의 parent frame과의 거리입니다.
global_left는 왼쪽 Lidar의 정보는 왼쪽 기준으로 저장되어있습니다. 이것을 차축 기준으로 평행이동하여 저장할 변수입니다. 


다음은 vlpt_sync_L.cpp 안의 Left_VLP_CB 콜백함수를 보겠습니다.

```c
velodyne_msgs::custompointPtr left(new velodyne_msgs::custompoint);
*left = input;

isLeftInit =true;

for (int i = 0; i < left->cpoints.size(); i++)
{
    left->cpoints[i].x += L_velodyne_x;
    left->cpoints[i].y += L_velodyne_y;
}
```
앞서 설명하였듯이 왼쪽 정보를 차축기준으로 평행이동하기 위한 과정입니다. 

이 부분 뒤 코드는 저장되는 point정보들의 묶음 수를 조절하기위한 코드이기 때문에 따로 설명하지 않겠습니다.

본격적으로 Vehicle_Top_CB 콜백함수를 보겠습니다.
이 콜백함수는 가운데 Lidar의 정보를 subscribe하는 

```c
pharos_vlp_tilt::VehiclePoseArrayPtr store_vehicle(new pharos_vlp_tilt::VehiclePoseArray);
velodyne_msgs::custompointPtr point_(new velodyne_msgs::custompoint);

store_vehicle = input;
```

우선 subscribe 된 정보를 가공하기위해 변수를 선언하여 저장하였습니다.

```c
double ltime = store_vehicle->header.stamp.toSec() - global_left[1].header.stamp.toSec();
*point_ = global_left[1];

if(ltime < 0)
{
    ltime = store_vehicle->header.stamp.toSec() - global_left[0].header.stamp.toSec();
    *point_ = global_left[0];
}
```

이 부분은 가운데와 왼쪽의 Lidar의 시간을 비교하여 ltime 이라는 변수에 저장을 하고 point_라는 변수에 왼쪽 정보를 저장하였습니다.
아래 if문을 추가한 이유는 혹시 데이터가 누락 되거나 가운데 Lidar의 정보보다 나중에 들어온 데이터를 골라내기 위해 추가하였습니다.

```c
for (int j = point_->cpoints.size() - 1; j >= 0; j--)
{
    int lt_hori = ltime * 18000;

    int num = (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) / 180;

    if(lt_hori < 0 && j == point_->cpoints.size() - 1 && isErrorHori == false)
    {
        isErrorHori = true;
        ROS_ERROR("Sync_L is error!!!!!!!!! : %d",lt_hori);
    }

    Eigen::Vector2d position;
    position.x() = point_->cpoints[j].x;
    position.y() = point_->cpoints[j].y;

    position.x() = position.x() + store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].x + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].x - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].x)/180 * (180 - (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) % 180);
    position.y() = position.y() + store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].y + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].y - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].y)/180 * (180 - (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) % 180);

//            std::cout<<position.x()<<std::endl;

    double point__theta;
    point__theta = store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].theta + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].theta - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].theta)/180 * (180 - (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) % 180);


    Eigen::Matrix2d rotation2Dmatrix;
    rotation2Dmatrix << cos(point__theta), -sin(point__theta),
            sin(point__theta), cos(point__theta);

    position = rotation2Dmatrix * position;

    point_->cpoints[j].x = position.x() - L_velodyne_x;
    point_->cpoints[j].y = position.y() - L_velodyne_y;
}
```

velodyne의 Lidar제품은 0.1초에 대략 1800개의 point 열을 같습니다.
이 point열이라는 것은 수평방향으로만 생각한 point의 갯수입니다.
따라서 1초에 18000개의 point를 갖습니다. 이 18000이라는 수를 ltime에 곱하게 되면 이 ltime이라는 시간동안 스캔된 point 열의 갯수가 구해집니다.
이 구해진 lt_hori과 선형보간법을 이용하여 차량이 이동하면서 생긴 Lidar의 오차를 보정합니다.
선형보간법에서 사용되는 기준 좌표는 앞에 store_vehicle에 저장된 차량 정보입니다.

이 차량정보의 이동거리, 차량 조향각을 이용하여 선형보간법 하여 각각 point하나하나에 맞는 오차를 보정하였습니다.

더 자세히 설명하기 위해 사진자료를 첨부하겠습니다.

[사진자료]

아래는 publish 내용과 가시화를 위한 과정이기 때문에 설명하지 않겠습니다.
