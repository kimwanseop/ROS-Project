![Screenshot from 2025-02-06 16-01-29](https://github.com/user-attachments/assets/077491d6-eea7-4a83-9a59-08c4317cf4e4)

# 1. Project Overview
## 1-1 Objective
![image](https://github.com/user-attachments/assets/37198db5-409f-43b7-a27a-d6a5de065ba6)

# 2. Plan
## 2-1 Operational Scenario
![image](https://github.com/user-attachments/assets/aaa5ce31-1c59-4270-a637-3f130968257c)

## 2-2 System Architecture
![image](https://github.com/user-attachments/assets/b1a0c2f4-2e7e-4d6a-8e67-d65b65ea59fd)

## 2-3 Status Diagram
![image](https://github.com/user-attachments/assets/ae10dce4-90e6-4307-a8c5-0809e2ab10f8)

# 3. Main Functions
## 3-1 Path Navigation Algorithm
![image](https://github.com/user-attachments/assets/82d582a5-bc3f-41de-820a-12d4d71758d2)

## 3-2 Vehicle Driving Algorithm
![Screenshot from 2025-02-10 16-37-54](https://github.com/user-attachments/assets/aa03fa29-059b-4dc4-ab47-c9e1b6fb73a5)

## 3-3 Vehicle Management System
![Screenshot from 2025-02-10 16-38-28](https://github.com/user-attachments/assets/a32f0f59-ea6c-4785-b7da-6b99f0672d34)

## 3-4 Car Classification Algorithm
![damage](https://github.com/user-attachments/assets/945d57c7-ddc1-4aba-a809-16e7cb7c5237) ![perfect](https://github.com/user-attachments/assets/a6c8c360-abe0-49e3-b53c-44985b33ad25) <br>
**1. 차량 손상, 정상 이미지 (car_damage 696장, car_perfect 724장) 데이터 분류** <br>
**2. 데이터 전처리 (데이터셋 80% 학습 / 20% 테스트로 분할)** <br>
**3. Timm 라이브러리 사용하여 ResNet-18 (사전학습된 모델) 로딩** <br>
**4. OpenCV 실시간 자동차 손상 여부 자동 분류 가능**

# 4. Conclusion
## Responsibility & Stacks
|이름|담당 업무|
|:---:|---|
|**이상범(팀장)**| * GUI 화면 설계 <br> * Client / Manage UI 개발 <br> * DB 구성 및 관리 (MySQL, Github, Docker) <br> * YOLO Face 활용한 얼굴인증 모델 개발|
|**윤민섭(팀원)**| * 도로 환경 설계 <br> * 이미지 UDP 통신 환경 구축 <br> * 인지, 판단, 제어 알고리즘 개발|
|**윤희태(팀원)**| * SLAM 을 활용한 최단 경로 생성 알고리즘 개발|
|**김완섭(팀원)**| * YOLO v8n - Detection 활용한 '객체 박스 감지' 개발 <br> * YOLO v8n - Segmentation 활용한 '도로 영역 감지' 개발 <br> * 차량 상태 점검 알고리즘 개발 <br> * PPT 자료 준비 <br> * Github 정리|

|구분|상세|
|------|----------------------|
|개발환경|![ROS](https://img.shields.io/badge/ROS2(JAZZY)-%230A0FF9.svg?style=for-the-badge&logo=ROS2(JAZZY)&logoColor=white) ![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white) ![Ubuntu](https://img.shields.io/badge/Ubuntu24.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)|
|개발언어|<img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white"> ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white) |
|라이브러리 & 툴|<img src="https://github.com/user-attachments/assets/5f8d52f1-1b12-4075-a59d-a641c01ad558" style="width: 150x; height: 25px;"> <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV&logoColor=white" /> ![PyTorch](https://img.shields.io/badge/PyTorch-%23EE4C2C.svg?style=for-the-badge&logo=PyTorch&logoColor=white) <img src="https://img.shields.io/badge/PyQt5-41CD52?style=for-the-badge&logo=Qt&logoColor=white"> <img src="https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=MySQL&logoColor=white">|
|협업관리|<img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=GitHub&logoColor=white"/> <img src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white"/> <img src="https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=Confluence&logoColor=white"/>|
|하드웨어|<img src="https://img.shields.io/badge/Raspberry Pi-A22846?style=for-the-badge&logo=Raspberry Pi&logoColor=white">
