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

#### 3. Sync Center & Sync

Sync에서는 자율주행차량이 고속주행을 하게 되면 Lidar가 스캔하는데 걸리는 시간 0.1초 동안 거리를 이동합니다.
이러한 고속주행으로 인한 오차가 발생하게 되어 차량 데이터 Vehicle State를 이용하여 오차를 보정합니다.

아래는 보정을 하기 전과 보정을 한 후의 사진입니다.
<div>
  <img width="400" src="/docs/images/sync_before">
  <img width="400" src="/docs/images/sync_after">
</div>

#### 4. Remove Ground

Remove Ground에서는 기본적으로 오픈소스로 나와있는 높이와 기울기를 이용한 지면제거를 썼습니다.
하지만 이는 32채널이상의 많은 채널의 Lidar는 상관이 없지만 16채널 정도의 Lidar라면 멀리 스캔하는 채널일 수록 채널간의 간격은 점점 넓어집니다.
따라서 기존 ring X hori 배열이 위에서 아래로 vertical 방향으로 인덱스 순서가 매겨져있다면 저는 왼쪽에서 오른쪽으로 horizontal 방향 즉 회전방향 기준으로 인덱스 순서를 재배열한 후 지면제거를 하였습니다.

아래는 Raw Data와 Object, Ground의 사진입니다.
<div>
  <img width="500" src="/docs/images/raw_data">
  <img width="500" src="/docs/images/object">
  <img width="500" src="/docs/images/ground">
</div>

#### 5. Map Filter

이번에는 Map Filter입니다.
이 코드도 직접 작성하지는 않았지만 이해를 하여 적용하였습니다.
이름에서와 같이 Map Data와 GPS Data를 이용하여 map에서 도로영역 이외에는 잘라내었습니다.
이 부분은 최적화에 많은 영역을 주어 많이 활용하였습니다.

filter 후 사진입니다.
<div>
  <img width="500" src="/docs/images/map_filter_after">
</div>
위 Remove Ground 단계 Object에서 도로영역 부분만 골라내어진 데이터가 되어 훨씬 적은양의 데이터를 처리하게 되었습니다.

#### 6. Segmentatiom

Segmentation입니다.
이 코드 또한 기존 Clustering 기법을 베이스로 작성하였습니다.
뜻은 분할이고 Point들을 거리 기준으로 '이 Point들이 서로 가깝다면 하나의 집합이고 물체이다'라고 하였습니다.

Segmentation후 사진입니다.
<div>
  <img width="500" src="/docs/images/segmentation_after">
</div>
기존 그냥 Point들로만 이루어져있던 Data가 집합을 이루어 같은 물체의 Point라면 같은 intensity값으로 설정하였습니다.

#### 7. Combine

Combine에서는 집합 Data들의 병합 및 다른 Lidar와의 병합을 하고있습니다.
이것을 설명하기 이전에 Segmentation에서 하나의 물체를 판단하여 집합으로 묶었지만 완벽히 처리되지 않는다는 단점이 있습니다.
그래서 이러한 집합 Data들을 다시한번 범위를 주어 병합을 합니다.
또한 16채널 Lidar이기 때문에 1개로는 정확성이 떨어집니다.
이러한 이유로 Lidar를 여러개 사용하여 그 Data들도 병합하는 역할도 하고있습니다.

이 사진은 Segmentation과 유사하기 때문에 생략하겠습니다.

#### 8. Messages For Other Part

이것또한 이름 그대로 다른 Part로 보내주기 위해 Data를 마지막으로 처리하고 보내는 역할을 합니다.


## 2. 정리

여기에서는 간단하게 설명을 했지만 다음에 코드를 자료로 첨부하여 어떤 역할을 하는지 자세하게 설명을 하겠습니다.
