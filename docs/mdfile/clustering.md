[목차](/README.md)          |           [Previous](/docs/mdfile/remove_ground.md)
# 물체 객체 구분

물체 객체 구분에서는 segmentation 코드, each 코드, combine 코드를 설명하겠습니다.

## 1. Segmentation

[vlpt_segmentation_L.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_segmentation_L.cpp)라는 파일을 기준으로 LeftSegmentation이라는 콜백함수를 보겠습니다.
```c
std::vector<int> save_i;
pharos_vlp_tilt::perfectarrayPtr left_input(new pharos_vlp_tilt::perfectarray);
left_input = raw_left_input;
pharos_vlp_tilt::perfectarrayPtr expand_my_msg(new pharos_vlp_tilt::perfectarray);
```
처음에 sav_i는 처리할 때 필요한 변수이고 그 다음 
subscribe 된 데이터를 left_input라는 변수에 저장하였습니다.
나머지 expand_my_msg 처리한 데이터를 저장할 변수입니다.

```c
for(int i=0; i < left_input->objects.size(); i++)
{
    save_i.push_back(i);    
    if(left_input->objects[i].info.hori != left_input->objects[i+1].info.hori || i+1 == left_input->objects.size() )
    {
        int full_data[vlp];
        for(int set_n =0; set_n < vlp; set_n++)
        {
            full_data[set_n] = -1;
        }
        for(int j =0; j < save_i.size(); j++)
        {
            full_data[left_input->objects[save_i[j]].info.laser] = save_i[j];
        }

        for(int num=0; num<vlp; num++ )
        {
            if(full_data[num] != -1 )
            {
                left_input->objects[ full_data[num] ].state.is_del = 0;
                left_input->objects[ full_data[num] ].state.is_infect = 0;
                expand_my_msg->objects.push_back(left_input->objects[ full_data[num] ]);
            }
            else
            {
                pharos_vlp_tilt::perfect pt;
                pt.state.is_del = 1;
                pt.state.is_infect = 0;
                expand_my_msg->objects.push_back(pt);
            }
        }
        save_i.clear();
    }
}

```

이 과정은 remove_ground 과정에 있는 point배열을 해주는 for문에서 지면제거 함수만 제거된 과정입니다. 그래서 따로 설명드리지 않겠습니다.
이 과정으로 point배열에서 센서가 인식하지 않은 빈공간의 데이터와 인식한 데이터가 확실하게 나눠지게 됩니다.
 

[목차](/README.md) | [Next](/docs/mdfile/data_combine.md)
