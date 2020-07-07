[목차](/README.md)          |           [Previous](/docs/mdfile/pretreatment.md)
# 지면제거 알고리즘

지면제거 알고리즘에서는 remove_ground 코드와 filter 코드를 설명하겠습니다.

## 1. Remove Ground

[vlpt_remove_ground_L.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_remove_ground_L.cpp)라는 파일을 기준으로 RemoveGround이라는 콜백함수를 보겠습니다.

```c
std::vector<pharos_vlp_tilt::perfectarray> vec_col(16);
///
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object_data(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground_data(new pcl::PointCloud<pcl::PointXYZI>);
///

pharos_vlp_tilt::perfectarrayPtr point_bank (new pharos_vlp_tilt::perfectarray);
std::vector<int> index_bank;
pharos_vlp_tilt::perfectPtr point_ (new pharos_vlp_tilt::perfect);
pharos_vlp_tilt::infoPtr info_ (new pharos_vlp_tilt::info);
pharos_vlp_tilt::perfectarrayPtr expand_bank(new pharos_vlp_tilt::perfectarray);
std::vector<PointType> property_bank;
```

처음은 마찬가지로 subscribe 된 데이터를 가공하기 위한 변수를 선언해주었습니다. 자세한 설명은 아래에 같이 설명하겠습니다.

```c
for (int index = 0; index < input->cpoints.size(); index++)
{

    point_->point.x = input->cpoints[index].x;
    point_->point.y = input->cpoints[index].y;
    point_->point.z = input->cpoints[index].z;
    point_->point.intensity = input->cpoints[index].intensity;

    point_->info.hori = hori;
    point_->info.laser = input->infos[index].ring;

    point_->state.is_ground = 1;

    point_bank->objects.push_back(*point_);
    index_bank.push_back(index);



    if(input->infos[index].hori != input->infos[index + 1].hori || index + 1 == input->cpoints.size())
    {
        int full_data[16];

        for(int set_n =0; set_n < 16; set_n++)
        {
            full_data[set_n] = -1;
        }

        DoDistinction_row(index_bank, point_bank);

        for(int j =0; j < index_bank.size(); j++)
        {
            full_data[point_bank->objects[index_bank[j]].info.laser] = index_bank[j];
        }

        for(int num=0; num< 16; num++ )
        {
            if(full_data[num] != -1 )
            {
                point_bank->objects[ full_data[num] ].state.is_del = 0;
                point_bank->objects[ full_data[num] ].state.is_infect = 0;
                expand_bank->objects.push_back(point_bank->objects[ full_data[num] ]);
            }
            else
            {
                pharos_vlp_tilt::perfect pt;
                pt.state.is_del = 1;
                pt.state.is_infect = 0;
                pt.info.hori = hori;
                expand_bank->objects.push_back(pt);
            }
        }

        index_bank.clear();
        hori++;
    }
}
```

이부분은 복잡한 것처럼 되어있지만 자세히 보면 Set에서의 코드와 비슷합니다. Set에서는 섞여있는 ring정보를 바르게 배열하기 위하여 사용하였습니다. 하지만 Remove Ground에서는 빈공간의 배열을 표시하기 위해 사용하였습니다.

하나의 point열을 생각해보겠습니다.
만약 ring 정보가 1,3,5,6,7인 point가 존재한다고하면 배열되어있는 정보는 아래와 같을 것입니다.

<span style="color:red">index</span>|<span style="color:red">0</span>|<span style="color:red">1</span>|<span style="color:red">2</span>|<span style="color:red">3</span>|<span style="color:red">4</span>|
---|---|---|---|---|---|
<span style="color:red">data</span>|1|3|5|6|7|

이렇게 point마다 ring정보를 표시해 두었기 때문에 빈공간의 데이터가 몇번째 ring인지는 금방 찾을 수 있습니다.
따라서 이 for문은 빈공간의 데이터를 아래처럼 표시합니다.
<span style="color:red">index</span>|<span style="color:red">0</span>|<span style="color:red">1</span>|<span style="color:red">2</span>|<span style="color:red">3</span>|<span style="color:red">4</span>|<span style="color:red">5</span>|<span style="color:red">6</span>|<span style="color:red">7</span>|<span style="color:red">8</span>|<span style="color:red">9</span>|<span style="color:red">10</span>|<span style="color:red">11</span>|<span style="color:red">12</span>|<span style="color:red">13</span>|<span style="color:red">14</span>|<span style="color:red">15</span>|
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
<span style="color:red">data</span>|0|1|-1|3|-1|5|6|7|-1|-1|-1|-1|-1|-1|-1|-1|

Lidar는 16채널을 사용하였기 때문에 나머지 빈 공간을 16번째 인덱스까지 꽉 채웠습니다.
-1인 숫자에 해당하는 ring에 구조체 형식으로 is_del이라는 msg 구조체를 사용하여 저장을 합니다.
부가적으로 나중 클러스터링에서 계층적 군집화를 위해 is_infect라는 msg 구조체를 사용하여 모두 0이라는 수치를 저장합니다.
이 is_del, is_fect와 위에는 나와있지 않지만 is_ground를 편의상 상태(status)라고 하겠습니다.
이 과정을 for문을 통해 모든 point열에 적용합니다.

그리고 중간에 보시면 DoDistinction_row이라는 함수를 볼수 있습니다. 
이 함수는 아래와 같습니다.
```c
void DoDistinction_row(std::vector<int> num_vec , pharos_vlp_tilt::perfectarrayPtr& pta) 
{

    for (unsigned int index = 0; index < num_vec.size(); index++)
    {

        if (index == 0)
        {
            pharos_vlp_tilt::point init_point;
            init_point.x = 0, init_point.y = 0, init_point.z = CAR_HEIGHT_;


            double alpha = asinf((pta->objects[num_vec[index]].point.z - init_point.z)
                                 / distance_two(init_point, pta->objects[num_vec[index]].point));
            alpha = alpha * 180.0 / M_PI;

            if (alpha >= init_ALPHA_MAX_)
            {

                
                pta->objects[num_vec[index]].state.is_ground = 1;

            }

            else
            {
                pta->objects[num_vec[index]].state.is_ground = 0;

            }


        }

        else
        {


            if (pta->objects[num_vec[index]].point.z > -0.2)
            {
                pta->objects[num_vec[index]].state.is_ground = 1;
                continue;
            }

            if (distance_two_f(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point) == true)
            {
                pta->objects[num_vec[index]].state.is_ground = 1;
                continue;

            }


            double alpha = asinf((pta->objects[num_vec[index]].point.z - pta->objects[num_vec[index - 1]].point.z)
                                 / distance_two(pta->objects[num_vec[index - 1]].point,
                                                pta->objects[num_vec[index]].point));
            alpha = alpha * 180.0 / M_PI;

            if (alpha >= ALPHA_MAX_)
            {
                pta->objects[num_vec[index]].state.is_ground = 1;
                continue;

            }

            if (pta->objects[num_vec[index - 1]].state.is_ground == 0)
            {
                if (alpha < SET_GROUND_DEG)
                {
                    pta->objects[num_vec[index]].state.is_ground = 0;


                }

                else
                {
                    pta->objects[num_vec[index]].state.is_ground = 1;
                }


            }

            else
            {

                if (pta->objects[num_vec[index]].point.z > pta->objects[num_vec[index - 1]].point.z + 0.2)
                {
                    pta->objects[num_vec[index]].state.is_ground = 1;
                    continue;
                }

                else if (alpha < 1.5 && alpha > -1.5 && pta->objects[num_vec[index]].point.z < -1.2 &&
                           pta->objects[num_vec[index]].point.z < -1.2)
                {
                    pta->objects[num_vec[index]].state.is_ground = 0;


                }

                else if (pta->objects[num_vec[index]].point.z < pta->objects[num_vec[index - 1]].point.z &&
                           pta->objects[num_vec[index]].point.z < -1.2)
                {
                    pta->objects[num_vec[index]].state.is_ground = 0;

                }


            }
        }
    }
}
```
이 함수는 하나의 point열에서 이전 index point와 비교 대상 pointf를 높이와 기울기로 비교하여 지면인지 물체인지 판단합니다. if문의 조건은
지면의 특징을 생각하여 정하였습니다.

```c
col_same_state(expand_bank, property_bank); 

std::vector<int> save_ground;

save_ground = check_state(expand_bank, property_bank); 

pharos_vlp_tilt::vector_perfect_arrayPtr least_sol_full(new pharos_vlp_tilt::vector_perfect_array);

least_square_method(expand_bank, property_bank, least_sol_full);


check_deviation(expand_bank, property_bank, least_sol_full, save_ground); 
```

위 함수들은 너무 복잡하기 때문에 함수로 만들었습니다. 하나씩 간단히 설명 드리겠습니다.

col_same_state부터 설명드리겠습니다.
기존 높이와 기울기의 지면제거 알고리즘은 16채널이라는 Lidar 특성상 전방 먼거리의 물체는 제대로 판단하기 힘들다는 한계점이 있습니다.
그래서 실제로는 물체이지만 높이와 기울기의 알고리즘에서 지면으로 판단한 point를 제대로 물체로 인식해야했습니다.
그래서 저는 point열이 아니라 같은 ring에 해당하는 point행으로 비교하려고 하였습니다.
따라서 이 함수는 현재 포인트에서 16번째 이후의 index를 비교하였습니다.
비교한후 계층적 알고리즘을 이용하여 인접하고 is_ground 상태정보가 같은 point만 vector로 묶어놓았습니다.

[사진 추가예정]

다음은 check_state 함수입니다.
이 함수는 말그대로 위 col_same_state로 묶어놓은 vector들이 지면인지 물체인지 판단하는 함수입니다. 내용은 단순히 is_ground 상태정보가 지면인 vector를 골라냅니다.
여기서 활용한 지면의 특성은 보통 1자로 point가 스캔 되어있고 왠만해선 길이가 길다는 것입니다. 또한 이 전체 vector의 첫점과 끝점의 기울기를 계산하여 이용하였습니다.
이 특성들로 조건을 생각하여 상태정보를 저장하였습니다.

지금까지의 함수는 높이와 기울기의 보완을 목적으로 만들었으며
아래부터는 거의 의미 없지만 혹시모를 상황에 대비하여 만들었습니다.

least_square_method 함수입니다.
이 함수는 단순하지만 작성하기 어려웠던 함수였습니다. 즉 간단히 말하면 최소제곱법을 이용하여 회귀선을 만들었습니다.
이번 목적은 물체로 잘못 판단된 지면 상태 point를 골라내기 위해서입니다.
따라서 최소제곱법을 이용하여 z축은 실제값 그대로 유지하고 x y축 실제값만 이용하여 회귀선을 만들었습니다.

그 후 check_deviation이라는 함수를 이용하였습니다.
이 함수로 위 최소제곱법으로 나열한 vector와 실제 vector의 값이 차이가 적다면 땅이라고 조건을 추가하였습니다.

이 함수 이후 코드는 가시화와 publish를 위한 과정입니다.

## 2. Map Filter

이번에는 
[vlpt_filter_L.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_filter_L.cpp)라는 파일을 기준으로 RemoveGround이라는 콜백함수를 보겠습니다.





[목차](/README.md) | [Next](/docs/mdfile/clustering.md)
