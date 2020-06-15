# 전처리과정

전처리 과정에는 Set 코드와 Sync 코드가 해당됩니다.

## 1. Set

vlpt_set_L.cpp라는 파일을 기준으로 LeftSetVertical이라는 콜백함수를 보겠습니다.

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

