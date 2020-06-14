# 알고리즘 개요

본격적으로 알고리즘을 설명하기 전에 간단한 순서와 내용을 설명드릴려고합니다.

## 1. 전체적인 순서도

가장 먼저 전체적인 순서에 대해 설명 드리겠습니다.
순서는 아래와 같습니다.

![순서도](/docs/images/AnyConv.com__Lidar_algorithm.png)

대략 이런 순서이며 빨간색은 Sensor의 Raw Data이며 노란색은 다른 파트를 나타내고, 초록색은 Lidar 코드입니다.
이 Lidar코드는 PointCloud Library를 이용하여 Lidar 데이터를 가공했습니다.
또한 Lidar 데이터만 이용하는 것이 아니라 차량데이터, GPS데이터, MAP데이터까지 이용하여 최적화를 하였습니다.

p.s. 이 순서도의 이름은 순전히 주관적인 저의 생각으로 붙힌 이름입니다.

## 2. 간단한 내용 요약

이제 순서도에서 초록색 부분의 내용을 간략히 말씀드리겠습니다.

1) Velodyne Driver
Driver는 원래 오픈소스로 제공되지만 안에 소스를 건드려서 custom msgs로 바꾸었습니다.
하지만 이 Driver가 제가 작성한 코드가 아니기 때문에 정확히 설명드릴 수 없습니다.
때문에 이 글에서만 간단하게 Custom msgs가 어떻게 구성 되어있는지만 설명 드리겠습니다.
구성은 아래와 같습니다.

```c

Header header
time stamp
info[] infos
cpoint[] cpoints

```
이 구성은 custompoint.msg 파일의 구성입니다.
이 파일 구성안에 info와 cpoint가 더 보이실겁니다.
이것또한 custom msgs로 다음과 같습니다.

```c

int32 ring                          float64 x
int32 hori                          float64 y
                                    float64 z
                                    float64 intensity

```



