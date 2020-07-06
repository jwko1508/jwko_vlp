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
1|2|3|4|5
---|---|---|---|---|
1|3|5|6|7|

이렇게 point마다 ring정보를 표시해 두었기 때문에 빈공간의 데이터가 몇번째 ring인지는 금방 찾을 수 있습니다.
따라서 이 for문은 빈공간의 데이터를 아래처럼 표시합니다.
1|2|3|4|5|6|7|
---|---|---|---|---|---|
1|-1|3|-1|5|6|7|

-1인 숫자에 해당하는 ring에 구조체 형식으로 is_del이라는 msg 구조체를 사용하여 저장을 합니다.
부가적으로 나중 클러스터링에서 계층적 군집화를 위해 is_infect라는 msg 구조체를 사용하여 모두 0이라는 수치를 저장합니다.


[목차](/README.md) | [Next](/docs/mdfile/clustering.md)
