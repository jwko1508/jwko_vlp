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
            std_pt->z = (pt_front->z + pt_back->z) / 2;/*pt_front->z > pt_back->z ?
                    pt_back->z : pt_front->z;*/
//            CAR_HEIGHT = std_pt->z;
        }


```
