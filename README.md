# Lanelet2 HD Map 제작 연습

## 1. 프로젝트 개요

이 저장소는 JOSM을 이용해 Lanelet2 기반 HD Map을 제작하는 연습 과정을 기록하기 위한 저장소입니다.

현재는 국토지리정보원에서 제공하는 K-City 지도 파일의 일부 구간을 이용해 Lanelet2 구조를 연습하고 있으며, 동국대학교 만해광장 지도를 기반으로 간단한 Lanelet2 HD Map을 제작해보았습니다.

최종 목표는 지금까지의 연습 과정을 바탕으로 K-City 전체 HD Map을 제작하고, 이를 ROS2 기반 자율주행 시스템에서 활용하는 것입니다.

---

## 2. 프로젝트 목표

이 프로젝트의 주요 목표는 다음과 같습니다.

* JOSM을 이용한 HD Map 편집 과정 학습
* Lanelet2 지도 구조 이해
* 도로의 좌우 경계선을 이용한 lanelet relation 생성
* `left`, `right`, `type=lanelet`, `subtype=road` 등 Lanelet2 기본 태그 분석
* K-City 일부 구간을 이용한 Lanelet2 태그 적용 연습
* 동국대학교 만해광장 간단한 HD Map 제작
* 향후 K-City 전체 HD Map 제작 과정 정리
* ROS2 및 RViz에서 Lanelet2 지도 시각화 준비

---

## 3. 현재까지 진행한 작업

### 3.1 K-City 일부 구간 Lanelet2 연습

K-City 전체 지도 파일을 바로 사용하기 전에, 극히 일부 구간만 추출하여 Lanelet2 구조를 연습했습니다.

이 과정에서 다음 내용을 연습했습니다.

* 도로 좌측 boundary way 생성
* 도로 우측 boundary way 생성
* 좌우 boundary way를 이용한 lanelet relation 생성
* relation member의 role을 `left`, `right`로 지정
* Lanelet2 형식의 OSM XML 구조 확인

사용 파일:

* `partial_lanelet2.osm`

![K-City 일부 구간 RViz 시각화](images/kcity_partial_rviz.png)

---

### 3.2 동국대학교 만해광장 HD Map 제작 연습

JOSM에서 동국대학교 만해광장 지도를 불러와 간단한 Lanelet2 HD Map을 제작했습니다.

이 과정에서 다음 내용을 연습했습니다.

* JOSM에서 실제 공간 지도 불러오기
* 도로 형태를 기준으로 boundary way 생성
* 좌우 boundary를 이용한 lanelet relation 생성
* Lanelet2 기본 태그 적용
* OSM 파일로 저장 후 추후 ROS2에서 사용할 수 있는 형태로 정리

사용 파일:

* `manhae_practice1.osm`
* `manhae_practice2.osm`

![동국대학교 만해광장 RViz 시각화](images/manhae1_rviz.png)

---

## 4. Lanelet2 기본 구조

Lanelet2에서 하나의 주행 가능 도로 구간은 보통 하나의 선이 아니라, 좌우 경계선으로 둘러싸인 영역으로 표현됩니다.

기본 구조는 다음과 같습니다.

* 왼쪽 경계선 way
* 오른쪽 경계선 way
* lanelet relation

lanelet relation의 기본 태그 예시는 다음과 같습니다.

```xml
<relation id="...">
  <member type="way" ref="..." role="left"/>
  <member type="way" ref="..." role="right"/>
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
</relation>
```

여기서 `left`와 `right`는 차량 진행 방향을 기준으로 구분합니다.

---



## 5. 사용 도구

* JOSM
* Lanelet2
* OSM XML
* ROS2 Humble
* RViz2
* Python

