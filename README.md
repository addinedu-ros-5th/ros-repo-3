# 물류 창고 출고 자동화 시스템

### 1. 프로젝트 개요

#### 1-1. 주제 소개
자율 주행 로봇을 활용한 물류 창고의 출고 과정을 자동화

#### 1-2. 기술 스택
|분류|기술|
|:---|:---|
|개발 환경|<img alt="Linux" src="https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=Linux&logoColor=white"/> <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white"/> <img alt="ROS2" src="https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ROS&logoColor=white"/>|
|언어|<img alt="C++" src="https://img.shields.io/badge/C++-D26383?style=for-the-badge&logo=cplusplus&logoColor=white"/> <img alt="Python" src="https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54" />|
|딥러닝 및 영상처리|<img alt="OpenCV" src="https://img.shields.io/badge/opencv-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img alt="Yolov8" src="https://img.shields.io/badge/Yolov8-8A2BE2?style=for-the-badge">|
|GUI|<img alt="PyQt5" src="https://img.shields.io/badge/PyQt-41CD52?style=for-the-badge&logo=Qt&logoColor=white"/>|
|데이터베이스|<img alt="MySQL" src="https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=MySQL&logoColor=white"/>|
|서버|<img alt="Crow" src="https://img.shields.io/badge/Crow-24404F?style=for-the-badge&logo=Crow&logoColor=white"/> <img alt="Socket" src="https://img.shields.io/badge/socket-blue?style=for-the-badge&logo=socket&logoColor=white"/>|
|협업|<img src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white"/> <img src="https://img.shields.io/badge/confluence-%23172BF4.svg?style=for-the-badge&logo=confluence&logoColor=white"/> <img src="https://img.shields.io/badge/Slack-4A154B?style=for-the-badge&logo=Slack&logoColor=white"/> <img src="https://img.shields.io/badge/github-181717?style=for-the-badge&logo=github&logoColor=white"/>|

#### 1-3. 팀원 소개
|이름|직책|담당|
|:---|:---|:---|
|조성현|팀장|통신 서버, 로봇 작업 관리, 고객 GUI, 작업자 GUI|
|강지연|팀원|모델 구축, AI서버, 관제 GUI|
|김요한|팀원|하드웨어, 맵 제작, 관제 GUI|
|이현복|팀원|SLAM, 경로 탐색 알고리즘 설계, Domain Bridge, 관제 GUI|

### 2. 프로젝트 설계

#### 2-1. 주요 기능
![Main Function](https://github.com/user-attachments/assets/e43d0160-6ed8-4fd2-b38a-1516a993b127)

#### 2-2. 시스템 구성
![System](https://github.com/user-attachments/assets/d2752056-4b50-499a-9d1c-7f521107dd8c)

#### 2-3. 로봇 상태도
![State](https://github.com/user-attachments/assets/93ea4af7-e49c-48cb-9e51-911f440cdc19)

#### 2-4. 시나리오
![Scenario](https://github.com/user-attachments/assets/ed460258-0664-4e46-8f7f-e30f44fe4cb3)

#### 2-5. 맵 구성
![Map](https://github.com/user-attachments/assets/b9aeeaaf-e9d5-4072-9365-55e07e4e6ecb)

### 3. 프로젝트 구현

#### 3-1. Customer GUI (고객 GUI)
[![Customer GUI](https://github.com/user-attachments/assets/30b8a890-67cb-4fc9-a86b-bf05259bed20)](https://youtube.com/shorts/9fw4idn_-Tk)

#### 3-2. Outbound Delivery Worker GUI (작업자 GUI)
[![Worker GUI](https://github.com/user-attachments/assets/e4d9f1f3-2f96-433b-a919-a25abbc0af27)](https://youtube.com/shorts/qxyMQt97Qd8)

#### 3-3. SLAM
![Slam](https://github.com/user-attachments/assets/58ca4252-34d6-4b2a-8d2b-6a2246fca718)

#### 3-4. Path Planning(A*)
![AStar](https://github.com/user-attachments/assets/0f2be15f-e3cf-4766-9a2e-f88a2d03861f)

#### 3-5. ROS Domain Bridge
![Domain Bridge](https://github.com/user-attachments/assets/19c66b60-53ac-4bd8-b331-01bf95405efa)

#### 3-6. Robot & Box Model
![Detection](https://github.com/user-attachments/assets/63f23e76-42bb-4bbc-afdb-2a27ee90a1c0)

##### 3-6-1. Detection
[![Camera](https://github.com/user-attachments/assets/db3dbbda-f09d-4950-8051-a091096d1af3)](https://youtube.com/shorts/RrZ8wz116vg)

#### 3-7. Outbound Delivery Management GUI (관제 GUI)
[![ODM GUI](https://github.com/user-attachments/assets/83defa56-85db-48b3-a12a-693bc0dce509)](https://youtu.be/g-ZS3oF_-eY)

#### 3-8. 동작 시나리오 예시
![Movement](https://github.com/user-attachments/assets/a305dd58-9844-438c-8d9a-7b023b2cf77b)

### 4. 프로젝트 시연
[![Video](https://github.com/user-attachments/assets/c47519ab-30b5-42fa-8d59-e38724e7e5f5)](https://youtu.be/1v3poAZRnrM)
