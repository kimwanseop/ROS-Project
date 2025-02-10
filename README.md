![Screenshot from 2025-02-06 16-01-29](https://github.com/user-attachments/assets/077491d6-eea7-4a83-9a59-08c4317cf4e4)

# 1. Project Overview
## 1-1 Proposed Methods
![Screenshot from 2025-02-06 16-11-22](https://github.com/user-attachments/assets/0d99c3e1-b1d6-4a85-be17-7f9ac53fd41e)

## 1-2 Objective
![Screenshot from 2025-02-06 16-12-17](https://github.com/user-attachments/assets/3ff51152-cf4b-4e67-8f0b-28de3c46896c)

# 2. Plan
## 2-1 System Requirements
|카테고리|Function (기능)|Description (구체화)|우선순위|
|:---:|:---:|---|:---:|
|**차량 관리 서비스**|**차량 호출**|* 서비스 지역 내 차량 호출 시 사용자 위치로 차량 이동|**1**|
||**차량 반납**|* 서비스 지역 내 차량 반납 시 차고지로 차량 이동|**1**|
||**소지품 확인**|* 차량 반납 시 차량 내에 소지품이 발견되면 알림|**2**|
||**결제 기능**|* 카드 결제 시스템을 통한 편의성 향상|**1**|
|||* 서비스 지역 외 호출 및 반납 시 추가 요금 부여|**2**|
|||* 차량 렌트 시간을 측정하여 요금 계산|**2**|
|||* 사용자의 배터리 충전량을 고려하여 요금 계산|**3**|
||**차량 파손 여부 확인**|* 차량 센터 도착 시 카메라를 통한 외관 상태 확인|**2**|
||**배터리 충전**|* 차량 센터 도착 시 배터리 잔량 확인|**2**|
|||* 배터리 잔량 50% 미만 시 충전 진행|**3**|
|**고객 관리 서비스**|**사용자 등록**|* 서비스 제공을 위한 사용자 정보 등록|**1**|
|||* 운전 면허증 등록|**3**|
||**VIP 서비스**|* 일정 기간 동안 계산된 요급을 합산|**4**|
|||* 결제 총액에 따른 할인 서비스 제공|**4**|
||**탑승자 확인**|* 등록된 카드 인식을 통한 차량 대여자 확인|**1**|
|||* 얼굴 인식을 통한 차량 대여자 확인|**2**|
|||* 차량 대여자 운전 면허증 확인|**3**|
|**관제**|**차량 위치 추적**|* SLAM을 이용한 차량 위치 추적|**1**|
||**시스템 보안**|* 아이디, 비밀번호를 통한 관리자 로그인|**1**|
||**이용 시간 추적**|* 차량 탑승 / 반납 / 서비스 전체 이용 시간 기록|**2**|
||**이용 요금 기록**|* 서비스 사용 요금 기록 / 전체 요금 합산|**3**|
||**사고 발생 추적**|* 차량 사고 발생 시각, 위치 기록|**4**|
|||* 차량 사고 영상 기록|**5**|
|||* 차량 대여자 운전 면허증 확인|**3**|
|**주행**|**주행 모드 변경**|* 주행 모드 자율 / 수동 모드로 변경|**1**|
||**수동 주행**|* 핸들(키보드 좌/우) 조작을 통한 조향 수행 <br> * 핸들(키보드 상/하) 조작을 통한 감가속 수행|**1**|
||**자율 주행**|* 목적지 까지 차선 유지 주행 <br> * 정적 장애물 인식시 회피 주행 <br> * 동적 장애물 인식시 일시 정지|**1**|
||**신호 대응**|* 신호등 인식 시 정지 및 출발 <br> * 제한 속도에 따른 속도 조절 <br> * 어린이 보호구역 진입 시 감속|**1**|
||**졸음 운전 대응**|* 졸음인식 시 졸음 경고음 알림 (3초 이내)|**1**|
|||* 졸음인식 시 시트 움직임 (반복 제동으로 대체) (5초 이내)|**2**|
|||* 졸음인식 시 비상등 점등 및 갓길 주차 (10초 이내)|**3**|
||**차량 사고 대응**|* 사고 발생 시 즉시 서비스 종료 <br> * 사고 발생 시 센터로 현재 위치 발송 <br> * 사고 발생 시 센터로 사고 알림 발송|**4**|
|||* 사고 발생 시 차량 주행 영상 녹화 및 저장|**5**|

## 2-2 System Architecture
![Screenshot from 2025-02-10 16-30-06](https://github.com/user-attachments/assets/255a2822-a079-496f-8560-4d76ba028e86)

## 2-3 Operational Scenario
![Screenshot from 2025-02-10 16-32-53](https://github.com/user-attachments/assets/57702547-ba5f-413d-8707-3979734ee67b)

# 3. Main Functions
## 3-1 Path Navigation Algorithm
![Screenshot from 2025-02-10 16-37-17](https://github.com/user-attachments/assets/0fe8fdab-8700-4cf6-8612-1b8d2ba74274)

## 3-2 Vehicle Driving Algorithm
![Screenshot from 2025-02-10 16-37-54](https://github.com/user-attachments/assets/aa03fa29-059b-4dc4-ab47-c9e1b6fb73a5)

## 3-3 Vehicle Management Algorithm
![Screenshot from 2025-02-10 16-38-28](https://github.com/user-attachments/assets/a32f0f59-ea6c-4785-b7da-6b99f0672d34)

# 4. Conclusion
## Responsibility & Stacks
|이름|담당 업무|
|:---:|---|
|**이상범(팀장)**| * GUI 화면 설계 <br> * Client / Manage UI 개발 <br> * DB 구성 및 관리 (Github, Docker)|
|**윤민섭(팀원)**| * 도로 환경 설계 <br> * 인지, 판단, 제어 알고리즘 개발|
|**윤희태(팀원)**| * SLAM, Navigation 개발|
|**김완섭(팀원)**| * YOLO (Detection, Segmentation) 개발 <br> * PPT 자료 준비 <br> * Github 정리|

|구분|상세|
|------|----------------------|
|개발환경|![ROS](https://img.shields.io/badge/ROS2(JAZZY)-%230A0FF9.svg?style=for-the-badge&logo=ROS2(JAZZY)&logoColor=white) ![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white) ![Ubuntu](https://img.shields.io/badge/Ubuntu24.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)|
|개발언어|<img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white"> ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white) |
|라이브러리 & 툴|<img src="https://github.com/user-attachments/assets/5f8d52f1-1b12-4075-a59d-a641c01ad558" style="width: 150x; height: 25px;"> <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV&logoColor=white" /> ![PyTorch](https://img.shields.io/badge/PyTorch-%23EE4C2C.svg?style=for-the-badge&logo=PyTorch&logoColor=white) <img src="https://img.shields.io/badge/PyQt5-41CD52?style=for-the-badge&logo=Qt&logoColor=white"> <img src="https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=MySQL&logoColor=white">|
|협업관리|<img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=GitHub&logoColor=white"/> <img src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white"/> <img src="https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=Confluence&logoColor=white"/>|
|하드웨어|<img src="https://img.shields.io/badge/Raspberry Pi-A22846?style=for-the-badge&logo=Raspberry Pi&logoColor=white">
