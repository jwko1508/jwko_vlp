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
