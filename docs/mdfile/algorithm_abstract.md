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

##### 1. Velodyne Driver
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

int32 ring                  | float64 x
int32 hori                  | float64 y
                            | float64 z
                            | float64 intensity

```

왼쪽이 info.msg 파일이고 오른쪽이 cpoint.msg 파일입니다.
설명을 드리자면 info의 ring은 Vertical Resolution 방향 Point의 번호입니다.
간단히 말씀드리면 만약 16채널 Lidar라면 이 ring의 숫자 범위는 0부터 15까지가 될것입니다.
마찬가지로 hori는 Horizontal Resolution 방향의 Point 번호입니다.
이 번호는 그때그때마다 달라지지만 보통 16채널 라이다가 600rpm에 0.2°의 Resolution을 갖고 있을때,
360°/0.2°를 하여 총 1800개의 보통의 열이 있습니다.
하지만 Lidar가 한바퀴보다 많이 돌기 때문에 1800개보다 조금더 많습니다.
아래는 왼쪽부터 순서대로 ring와 hori를 보기쉽게 가시화한 자료입니다.

<div>
  <img width="400" src="/docs/images/vertical">
  <img width="400" src="/docs/images/horizon">
  
  
</div>

다음은 cpoint.msg 파일입니다.
이 파일은 말 그대로 한 Point의 xyz좌표와 intensity값을 나타냅니다.

이 과정에서 적용한 것은 Ring과 Hori를 적용해 주었단 것입니다.
이것에 대한 이점은 원래 Sensor의 Datasheet를 보면 알 수 있습니다.

Laser ID|Vertical angle VLP-16|
---|---|
0|-15°|
1|1°|
2|-13°|
3|3°|
4|-11°|
5|5°|
6|-9°|
7|7°|
8|-7°|
9|9°|
10|-5°|
11|11°|
12|-3°|
13|13°|
14|-1°|
15|15°|

이 표를 보면 원래 Driver에서 인식하는 Laser ID입니다.
하지만 저는 Ring을 적용해주면서 이 뒤죽박죽의 Data를 올바르게 오름차순으로 정렬해주었고
같이 Hori를 적용하여 이게 몇번째 열인지 정리 해주었습니다.

#### 2. Set
이제부터는 나중에 따로 자세히 설명할 예정이므로 간단하게 소개만 해드리겠습니다.

Set에서는 가시화적인 자료가 딱히 없고 이 포인트들을 ring X hori 배열로 바꾸어 처리하기 편하게 정렬하는 역할을 합니다.
또한 정렬하기 이전에는 이미 있을지도 모르지만 제가 작성한 과속방지턱이나 흔들림 외란에 대비한 알고리즘도 적용하였습니다.

#### 3. Sync

Sync에서는 자율주행차량이 고속주행을 하게 되면 Lidar가 스캔하는데 걸리는 시간 0.1초 동안 거리를 이동합니다.
이러한 고속주행으로 인한 오차가 발생하게 되어 차량 데이터 Vehicle State를 이용하여 오차를 보정합니다.
아래는 보정을 하기 전과 보정을 한 후의 사진입니다.







