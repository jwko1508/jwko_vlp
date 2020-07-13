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
            multiple_i = DataInfection(init_one_i , input, one_object);
            while(multiple_i.size() != 0)
            {
                multiple_i = DataInfection(multiple_i,input, one_object);
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
|**i-16**|***i***|**i+16**|
|**i-16-1**|**i-1**|**i+16-1**|

여기서 기준이 되는 것은 i라는 임의의 index입니다. 원리는 다시 한번 말하면 16채널의 배열의 한 point인 i 와 주변 8개의 point의 거리를 비교하여 임의의 threthhold 값을 넘지 않는다면 한 객체로 보는것 입니다.

|15|1|2|2|2|2|2|2|2|2|2|2|2|2|2|2|2|2|2|~|3|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|**14**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**13**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**12**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**11**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**10**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**9**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**8**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**7**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**6**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**5**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**4**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**3**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**2**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**1**|8|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|~|4|
|**0**|7|6|6|6|6|6|6|6|6|6|6|6|6|6|6|6|6|6|~|5|

위 그림은 if문 조건에 대한 설명을 하기위해 시각화 자료를 만들었습니다. 여기서 3, 4, 5 번은 이 배열의 가잘 끝열을 의미합니다.
이렇게 1 ~ 8 번처럼 가장자리에 존재하는 point는 주변 점을 온전히 찾을 수 없습니다. 그렇기 때문에 Lidar가 원형으로 스캔한다는 점을 이용하여 처음 열을 마지막 열과 비교하는 작업을 if문으로 하였습니다. 또한 가장 위의 point행과 아래의 point행은 연결되어 있지 않기 때문에 상황에 맞는 조건을 작성하였습니다.

<div>
  <img width="500" src="/docs/images/segmentation_after">
</div>
위 사진은 알고리즘 개요에서 보았던 사진으로 각 물체마다 intensity값을 다르게 주어 표시하였습니다.

이제 하나의 객체라고 인식된 것들을 군집화하였으니 each의 코드로 넘어가겠습니다.

## 2. Each

[vlpt_each_L.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_each_L.cpp)라는 파일을 기준으로 EachCombineObject이라는 콜백함수를 보겠습니다.

```c
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > vec_listed_pointcloud;
vec_listed_pointcloud.reserve(20000);
std::vector<CombineType> vec_listed_combine;
vec_listed_combine.reserve(10000);
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > vec_infos;
vec_infos.reserve(10000);
```

처음은 역시 가공할 데이터를 저장할 변수 선언입니다. 이번에는 subscribe된 데이터를 통째로 저장하지 않고 나눠서 저장을 할 예정입니다.

```c
for(unsigned int i=0; i < input->one.size(); i++)
{
    ListSeqeunce( input->one[i] ,vec_listed_pointcloud, vec_listed_combine,vec_infos );

}
```
위에서 설명했듯이 데이터를 나눠서 저장하기 위한 함수입니다. 함수를 자세히 보겠습니다.

```c
void ListSeqeunce(pharos_vlp_tilt::perfectarray& one  , std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& pointvector
            , std::vector<CombineType>& combinevector ,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& infooo    )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    CombineType object_pose_info;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_and_hori (new pcl::PointCloud<pcl::PointXYZI>);

    double center_x(0) ,center_y(0), center_z(0);
    pcl::PointXYZI pt;
    pcl::PointXYZI infopt;

    int info_min = 10000;
    int info_max = -1;


    for(unsigned int i=0; i < one.objects.size(); i++)
    {
        center_x += one.objects[i].point.x ,pt.x = one.objects[i].point.x;
        center_y += one.objects[i].point.y ,pt.y = one.objects[i].point.y;
        center_z += one.objects[i].point.z ,pt.z = one.objects[i].point.z;
        ///!!!!
        infopt.x = one.objects[i].info.hori;
        infopt.y = one.objects[i].info.laser;
        laser_and_hori->push_back(infopt);


        object_cloud->push_back(pt);

        if(i == 0)
        {
            object_pose_info.min_object.x = one.objects[i].point.x;
            object_pose_info.min_object.y = one.objects[i].point.y;
            object_pose_info.min_object.z = one.objects[i].point.z;
            object_pose_info.max_object.x = one.objects[i].point.x;
            object_pose_info.max_object.y = one.objects[i].point.y;
            object_pose_info.max_object.z = one.objects[i].point.z;

        }

        else
        {

            object_pose_info.min_object.x = (object_pose_info.min_object.x >= one.objects[i].point.x) ? one.objects[i].point.x : object_pose_info.min_object.x;
            object_pose_info.min_object.y = (object_pose_info.min_object.y >= one.objects[i].point.y) ? one.objects[i].point.y : object_pose_info.min_object.y;
            object_pose_info.min_object.z = (object_pose_info.min_object.z >= one.objects[i].point.z) ? one.objects[i].point.z : object_pose_info.min_object.z;
            object_pose_info.max_object.x = (object_pose_info.max_object.x <= one.objects[i].point.x) ? one.objects[i].point.x : object_pose_info.max_object.x;
            object_pose_info.max_object.y = (object_pose_info.max_object.y <= one.objects[i].point.y) ? one.objects[i].point.y : object_pose_info.max_object.y;
            object_pose_info.max_object.z = (object_pose_info.max_object.z <= one.objects[i].point.z) ? one.objects[i].point.z : object_pose_info.max_object.z;
            info_min = (info_min >= one.objects[i].info.hori) ? one.objects[i].info.hori : info_min;
            info_max = (info_max <= one.objects[i].info.hori) ? one.objects[i].info.hori : info_max;
            object_pose_info.min_hori = info_min;
            object_pose_info.max_hori = info_max;
            object_pose_info.min_center = (info_min == one.objects[i].info.hori) ? PointtoCenter(one.objects[i].point) : object_pose_info.min_center;
            object_pose_info.max_center = (info_max == one.objects[i].info.hori) ? PointtoCenter(one.objects[i].point) : object_pose_info.max_center;
        }



    }

    object_pose_info.center_x = center_x/one.objects.size();
    object_pose_info.center_y = center_y/one.objects.size();
    object_pose_info.center_z = center_z/one.objects.size();


    pointvector.push_back(object_cloud);
    combinevector.push_back(object_pose_info);
    infooo.push_back(laser_and_hori);

}
```
이 함수는 비교적 간단하여 for문 안쪽을 살펴보겠습니다.
그 전에 변수에 대해 헷갈릴 수 있어 간단히 설명하겠습니다. 일단 one은 이전 콜백함수에서 subscribe된 input 데이터이고, 나머지 pointvector, combinevector, infooo는 아까 가공할 데이터를 저장할 변수입니다. 다음 바로 for문입니다.

```c
center_x += one.objects[i].point.x ,pt.x = one.objects[i].point.x;
center_y += one.objects[i].point.y ,pt.y = one.objects[i].point.y;
center_z += one.objects[i].point.z ,pt.z = one.objects[i].point.z;
///!!!!
infopt.x = one.objects[i].info.hori;
infopt.y = one.objects[i].info.laser;
laser_and_hori->push_back(infopt);
```
처음 부분은 중심점을 구하기 위해 center_x,y,z라는 변수에 한 객체의 point x,y,z 좌표를 전부 더하여 저장합니다.
그리고 x,y,z 좌표도 따로 저장하고, set 코드부터 계속 설명하였던 hori, laser(ring)정보를 따로 저장합니다.

```c
if(i == 0)
{
    object_pose_info.min_object.x = one.objects[i].point.x;
    object_pose_info.min_object.y = one.objects[i].point.y;
    object_pose_info.min_object.z = one.objects[i].point.z;
    object_pose_info.max_object.x = one.objects[i].point.x;
    object_pose_info.max_object.y = one.objects[i].point.y;
    object_pose_info.max_object.z = one.objects[i].point.z;

}

else
{

    object_pose_info.min_object.x = (object_pose_info.min_object.x >= one.objects[i].point.x) ? one.objects[i].point.x : object_pose_info.min_object.x;
    object_pose_info.min_object.y = (object_pose_info.min_object.y >= one.objects[i].point.y) ? one.objects[i].point.y : object_pose_info.min_object.y;
    object_pose_info.min_object.z = (object_pose_info.min_object.z >= one.objects[i].point.z) ? one.objects[i].point.z : object_pose_info.min_object.z;
    object_pose_info.max_object.x = (object_pose_info.max_object.x <= one.objects[i].point.x) ? one.objects[i].point.x : object_pose_info.max_object.x;
    object_pose_info.max_object.y = (object_pose_info.max_object.y <= one.objects[i].point.y) ? one.objects[i].point.y : object_pose_info.max_object.y;
    object_pose_info.max_object.z = (object_pose_info.max_object.z <= one.objects[i].point.z) ? one.objects[i].point.z : object_pose_info.max_object.z;
    info_min = (info_min >= one.objects[i].info.hori) ? one.objects[i].info.hori : info_min;
    info_max = (info_max <= one.objects[i].info.hori) ? one.objects[i].info.hori : info_max;
    object_pose_info.min_hori = info_min;
    object_pose_info.max_hori = info_max;
    object_pose_info.min_center = (info_min == one.objects[i].info.hori) ? PointtoCenter(one.objects[i].point) : object_pose_info.min_center;
    object_pose_info.max_center = (info_max == one.objects[i].info.hori) ? PointtoCenter(one.objects[i].point) : object_pose_info.max_center;
}
```
다음 if문은 한 객체에서 가장 큰 max_x,y,z , max_hori값과 가장 작은 min_x,y,z , min_hori값들을 비교하여 저장합니다..
이 과정을 하나의 객체의 point수 만큼 반복하여 아까 max와 min값을 결정하고 center_x,y,z 저장된 값을 객체의 point 수만큼 나눠 중심점을 찾아 저장해줍니다.

이전의 계층적 군집화로는 객체를 하나라고 인식하는 것이 불완전하였기 때문에 위 방법처럼 K-평균 군집화 알고리즘을 사용할 데이터들을 정리하였었습니다. 하지만 저는 다른 방법으로 사용해보았습니다.
우선 z축 방향으로 point를 2차원적으로 보았습니다.  

[사진추가예정]

위그림 처럼 xy영역을 사각형처럼 생각하고 이 사각형이 겹쳐져 있다면 한 객체로써 저장을 하였습니다. 이 과정이 CombineObject라는 함수에 담겨져 있습니다.

지금까지 velodyne driver부터 설명한 모든 과정이 하나의 Lidar에서 처리되는 알고리즘이고 다음 코드에서는 이 다중 Lidar의 정보를 합치는 역할을 합니다.

## 3. Combine
 
[vlpt_LRT_Combine.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_LRT_Combine.cpp)라는 파일을 기준으로 합니다.

하지만 이 코드는 Each 코드와 별반 다를게 없습니다.
다른 것이 있다면 Lidar 3개의 정보를 subscribe 받아 정보를 합치는 것입니다.
그러므로 이 코드는 따로 설명하지않고 넘어가겠습니다.


[목차](/README.md) | [Next](/docs/mdfile/data_combine.md)
