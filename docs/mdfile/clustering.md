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

이 과정은 remove_ground의 point배열을 해주는 for문에서 지면제거 함수만 제거된 과정입니다. 다른 점은 빈 공간데이터 뿐만 아니라 지면이라고 인식한 point까지 전부 빈 공간의 데이터라고 설정해주는 것입니다.
 

```c
std::vector<pharos_vlp_tilt::perfectarray> objects_vector;
objects_vector = Vlp16Segmentation(expand_my_msg);

```
여기서 계층적 군집화 알고리즘이 시작됩니다.
Vlp16Segmentation 이 함수가 계층적 군집화를 하는 함수입니다. 이 함수를 자세히 살펴보겠습니다.

```c
 std::vector<pharos_vlp_tilt::perfectarray> Vlp16Segmentation(pharos_vlp_tilt::perfectarrayPtr &input)
{

    std::vector<pharos_vlp_tilt::perfectarray> all_objects;
    for(int i=0; i < input->objects.size(); i++)
    {
        if(input->objects[i].state.is_infect == 0 && input->objects[i].state.is_del ==0 )
        {
            std::vector<int> init_one_i;
            std::vector<int> multiple_i;
            pharos_vlp_tilt::perfectarrayPtr pfa(new pharos_vlp_tilt::perfectarray);
            input->objects[i].state.is_infect = 1;
            pharos_vlp_tilt::perfectarrayPtr one_object(new pharos_vlp_tilt::perfectarray);

            one_object->objects.push_back(input->objects[i]);

            init_one_i.push_back(i); 
            multiple_i = DataInfection(init_one_i , input, one_object/*, info_hori_min, info_hori_max*/);
            while(multiple_i.size() != 0)
            {
                multiple_i = DataInfection(multiple_i,input, one_object/*, info_hori_min, info_hori_max*/);
            }

            if(one_object->objects.size() <= MIN_OBJECT_NUM)
            {}
            else
            {
                all_objects.push_back(*one_object);
            }
        }


    }
    return all_objects;

}
```

이 함수는 모든 점 하나하나에 대하여 주변점이 가까운지 판단하고 하나의 객체인지 결정합니다.
이 함수를 사용하기 이전 알고리즘에서 is_infect라는 상태정보의 모든 값을 0으로 초기화 해주었습니다. 이 is_infect가 0인 물체 point만을 이용하여 이 함수를 실행됩니다. 

처음 point부터 시작해봅시다. 처음 point는 기준점이기 때문에 일단 is_infect를 1로 변경해줍니다. 이 point를 저장하고 
주변의 point들이 가깝지 않은 포인트만 있거나 빈공간의 데이터만 있을 때 계층적 알고리즘을 그만둡니다. 이 과정을 계속하다보면 결과적으로 아래와 같이 묶이게 됩니다.

|15|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|---|-|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|**14**|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|4|4|4|4|
|**13**|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|4|4|4|
|**12**|-|-|-|-|-|-|-|-|-|-|3|-|3|3|3|-|-|-|4|-|
|**11**|-|-|-|-|-|-|-|-|-|-|-|3|3|3|-|-|-|4|-|4|
|**10**|-|-|-|-|-|-|-|-|-|-|-|-|3|3|-|-|-|-|4|-|
|**9**|-|-|-|-|-|-|-|-|-|-|-|-|3|3|-|-|-|-|-|-|
|**8**|-|-|-|-|-|-|-|-|-|-|-|3|3|-|-|-|-|-|-|-|
|**7**|-|-|-|-|-|-|2|-|2|-|-|-|3|3|-|-|-|-|-|-|
|**6**|-|-|-|-|2|2|2|2|2|2|-|3|3|3|-|-|-|-|-|-|
|**5**|-|-|-|-|-|2|2|2|2|2|-|3|3|3|-|-|-|-|-|-|
|**4**|-|-|-|-|-|-|2|-|2|-|-|3|3|3|-|-|-|-|-|-|
|**3**|-|-|1|-|-|-|-|-|-|-|-|3|3|3|-|-|-|-|-|-|
|**2**|-|1|1|-|-|-|-|-|-|-|-|3|3|3|-|-|-|-|-|-|
|**1**|1|1|1|-|-|-|-|-|-|-|-|-|3|-|-|-|-|-|-|-|
|**0**|1|1|1|1|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|

왼쪽 숫자는 Lidar의 채널이고 그 이외에는 같은 숫자끼리 하나의 객체라는 것을 표시합니다.

다음 DataInfection라는 함수가 중간에 존재합니다. 이 함수는 계층적 알고리즘의 조건을 설정하는 함수입니다.

```c
std::vector<int> DataInfection(std::vector<int> input_i , pharos_vlp_tilt::perfectarrayPtr &input_my_msg , pharos_vlp_tilt::perfectarrayPtr &object_output)
{
    std::vector<int> out_put_is; ///return 할 output i들

    for(int i=0; i < input_i.size(); i++)
    {
        int way[8] = {input_i[i]-vlp,   /// @won -> 요것들이 전부 input_my_msg에 해당되는 i값이다.
                      input_i[i]-vlp+1,
                      input_i[i]+1,
                      input_i[i]+vlp+1,
                      input_i[i]+vlp,
                      input_i[i]+vlp-1,
                      input_i[i]-1,
                      input_i[i]-vlp-1,
        };

        for(int n=0; n < 8; n++)
        {
            if(n==0)
            {
                if( input_i[i] < vlp )
                {
                    way[0] = (input_i[i] % vlp ) + (input_my_msg->objects.size() -vlp );

                    if (input_my_msg->objects[ way[0] ].state.is_infect == 0 && input_my_msg->objects[ way[0]].state.is_del ==0 )
                    {
                        if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[0]].point ) <= THRESHOLD  )
                        { 
                            input_my_msg->objects[ way[0] ].state.is_infect = 1;
                            out_put_is.push_back(way[0]);
                            object_output->objects.push_back( input_my_msg->objects[way[0]]);
                        }
                    }
                }
                else
                {
                    if (input_my_msg->objects[ way[0] ].state.is_infect == 0 && input_my_msg->objects[ way[0]].state.is_del ==0 )
                    {
                        if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[0]].point ) <= THRESHOLD  )
                        {
                            input_my_msg->objects[ way[0] ].state.is_infect = 1;
                            out_put_is.push_back(way[0]);
                            object_output->objects.push_back( input_my_msg->objects[way[0]]);
                        }
                    }

                }
            }
            else if(n==1)
            {
                if( input_i[i] % vlp == 15) {

                }
                else
                {
                    if( input_i[i] < vlp ) 
                    {
                        way[1] = (input_i[i] % vlp ) + (input_my_msg->objects.size() -vlp +1 );
                        if (input_my_msg->objects[way[1]].state.is_infect == 0 && input_my_msg->objects[way[1]].state.is_del == 0)
                        {
                            if (ReturnDistance(input_my_msg->objects[input_i[i]].point,input_my_msg->objects[way[1]].point) <= THRESHOLD /*+ 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point)*/ )
                            {
                                input_my_msg->objects[ way[1] ].state.is_infect = 1;
                                out_put_is.push_back(way[1]);
                                object_output->objects.push_back( input_my_msg->objects[way[1]]);
                            }
                        }
                    }
                    else 
                    {
                        if (input_my_msg->objects[way[1]].state.is_infect == 0 && input_my_msg->objects[way[1]].state.is_del == 0)
                        {
                            if (ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[way[1]].point) <= THRESHOLD )
                            {
                                input_my_msg->objects[ way[1] ].state.is_infect = 1;
                                out_put_is.push_back(way[1]);
                                object_output->objects.push_back( input_my_msg->objects[way[1]]);
                            }
                        }
                    }
                }
            }
            else if(n==2)
            {
                if( input_i[i] % vlp == 15)
                {

                }
                else
                {
                    if (input_my_msg->objects[way[2]].state.is_infect == 0 && input_my_msg->objects[way[2]].state.is_del == 0)
                    {
                        if (ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[way[2]].point) <= THRESHOLD )
                        {
                            input_my_msg->objects[ way[2] ].state.is_infect = 1;
                            out_put_is.push_back(way[2]);
                            object_output->objects.push_back( input_my_msg->objects[way[2]]);
                        }
                    }
                }
            }
            else if(n==3)
            {
                if( input_i[i] % vlp == 15 )
                {

                }

                else
                {
                    if( input_i[i]+vlp >= input_my_msg->objects.size() )
                    {
                        way[3] = (input_i[i] % vlp) + 1;
                        if (input_my_msg->objects[ way[3] ].state.is_infect == 0 && input_my_msg->objects[ way[3]].state.is_del ==0 )
                        {
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[3]].point ) <= THRESHOLD  )
                            {
                                input_my_msg->objects[ way[3] ].state.is_infect = 1;
                                out_put_is.push_back(way[3]);
                                object_output->objects.push_back( input_my_msg->objects[way[3]]);
                            }
                        }
                    }
                    else
                    {
                        if (input_my_msg->objects[way[3]].state.is_infect == 0 && input_my_msg->objects[way[3]].state.is_del == 0)
                        {
                            if (ReturnDistance(input_my_msg->objects[input_i[i]].point,
                                               input_my_msg->objects[way[3]].point) <=
                                THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point)) {
                                input_my_msg->objects[way[3]].state.is_infect = 1;
                                out_put_is.push_back(way[3]);
                                object_output->objects.push_back( input_my_msg->objects[way[3]]);
                            }
                        }
                    }
                }
            }
            else if(n==4)
            {
                if( input_i[i]+vlp >= input_my_msg->objects.size() )
                {
                    way[4] =input_i[i] % vlp;
                    if (input_my_msg->objects[ way[4] ].state.is_infect == 0 && input_my_msg->objects[ way[4]].state.is_del ==0 )
                    {
                        if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[4]].point ) <= THRESHOLD  )
                        {
                            input_my_msg->objects[ way[4] ].state.is_infect = 1;
                            out_put_is.push_back(way[4]);
                            object_output->objects.push_back( input_my_msg->objects[way[4]]);
                        }
                    }
                }

                else
                {
                    if (input_my_msg->objects[ way[4] ].state.is_infect == 0 && input_my_msg->objects[ way[4]].state.is_del ==0 )
                    {
                        if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[4]].point ) <= THRESHOLD  )
                        {
                            input_my_msg->objects[ way[4] ].state.is_infect = 1;
                            out_put_is.push_back(way[4]);
                            object_output->objects.push_back( input_my_msg->objects[way[4]]);
                        }
                    }
                }
            }
            else if(n==5)
            {
                if( input_i[i] % vlp == 0 ) 
                {}
                else
                {
                    if( input_i[i]+vlp >= input_my_msg->objects.size() )
                    {
                        way[5] = (input_i[i] % vlp) -1;
                        if (input_my_msg->objects[ way[5] ].state.is_infect == 0 && input_my_msg->objects[ way[5]].state.is_del ==0 )
                        {
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[5]].point ) <= THRESHOLD    )
                            {
                                input_my_msg->objects[ way[5] ].state.is_infect = 1;
                                out_put_is.push_back(way[5]);
                                object_output->objects.push_back( input_my_msg->objects[way[5]]);
                            }
                        }
                    }
                    else
                    {
                        if (input_my_msg->objects[ way[5] ].state.is_infect == 0 && input_my_msg->objects[ way[5]].state.is_del ==0 )
                        {
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[5]].point ) <= THRESHOLD  )
                            {
                                input_my_msg->objects[ way[5] ].state.is_infect = 1;
                                out_put_is.push_back(way[5]);
                                object_output->objects.push_back( input_my_msg->objects[way[5]]);
                            }
                        }
                    }
                }



            }
            else if(n==6)
            {
                if(input_i[i] % vlp ==0 )
                {}
                else
                {
                    if (input_my_msg->objects[ way[6] ].state.is_infect == 0 && input_my_msg->objects[ way[6]].state.is_del ==0 )
                    {
                        if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[6]].point ) <= THRESHOLD )
                        {
                            input_my_msg->objects[ way[6] ].state.is_infect = 1;
                            out_put_is.push_back(way[6]);
                            object_output->objects.push_back( input_my_msg->objects[way[6]]);
                        }
                    }
                }
            }
            else
            {
                if(input_i[i] % vlp == 0 )
                {}
                else
                {
                    if( input_i[i] < 16 ) 
                    {
                        way[7] = (input_i[i] % vlp ) + (input_my_msg->objects.size() -vlp ) -1;
                        if (input_my_msg->objects[ way[7] ].state.is_infect == 0 && input_my_msg->objects[ way[7]].state.is_del ==0 )
                        {
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[7]].point ) <= THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point) )
                            {
                                input_my_msg->objects[ way[7] ].state.is_infect = 1;
                                out_put_is.push_back(way[7]);
                                object_output->objects.push_back( input_my_msg->objects[way[7]]);
                            }
                        }
                    }
                    else
                    {
                        if (input_my_msg->objects[ way[7] ].state.is_infect == 0 && input_my_msg->objects[ way[7]].state.is_del ==0 )
                        {
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[7]].point ) <= THRESHOLD  )
                            {
                                input_my_msg->objects[ way[7] ].state.is_infect = 1;
                                out_put_is.push_back(way[7]);
                                object_output->objects.push_back( input_my_msg->objects[way[7]]);
                            }
                        }
                    }
                }
            }

        }
    }
    return out_put_is;



}
```
여기의 함수 if문은 각각 계층적 알고리즘을 상황별로 조건을 주어 작성하였습니다.
엄청나게 복잡하게 보이지만 사실 하나하나 알고보면 엄청 간단한 방식입니다.
이렇게 if문 조건이 나눠진 이유에 대해 설명하기 이전에 원리에 대해 시각화 자료로 보겠습니다.

|i-16+1|i+1|i+16+1|
|---|---|---|
|**i-16**|<div class="text-purple">i<a href="#" class="text-inherit"></a></div>|**i+16**|
|**i-16-1**|**i-1**|**i+16-1**|

[목차](/README.md) | [Next](/docs/mdfile/data_combine.md)
