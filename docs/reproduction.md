# 실행 준비와 현재 확인 상태

## 준비물과 구성

STM32F103RB 3개, 마이크 3개, Flex 센서 5개, MPU6050, 방향 LED와 16×2 I2C LCD가 필요합니다. 공개한 A/B 설정을 STM32CubeIDE/CubeMX 환경에 연결하고 STM32F1 HAL·CMSIS·startup·링커 설정을 맞춰야 합니다. C 참조 코드의 LCD 함수는 파일 안에 있으며, 별도 `main.h`와 독립 프로젝트 설정의 복원이 필요합니다.

선별 코드·헤더·설정은 구조를 읽기 위한 공개 범위입니다. IDE build output과 SDK 전체를 포함한 완전한 즉시 실행 배포본으로 확인한 것은 아닙니다.

## 데이터 수집과 학습

1. 수집용 B firmware의 센서 입력과 UART 출력을 확인합니다.
2. `tools/collect_data.py`의 실제 COM 포트·baud rate·출력 위치를 맞춥니다.
3. CSV의 Flex/accel/gyro 열과 윈도우 길이를 특징 추출 입력과 대조합니다. 공개 예시 CSV는 gyro 열이 없으므로 그대로 185차원 입력으로 쓸 수 없습니다. CSV에서 features.npy/labels.npy를 만든 최종 원본 조합은 별도 확인 대상입니다.
4. 검증한 실제 특징 데이터를 features.npy/labels.npy로 준비하고 `tools/train_gesture_all.py`를 실행해 표준화와 선형 모델을 학습합니다.
5. 같은 특징 순서·scale로 C 헤더를 만들고 PC와 MCU의 동일 입력 점수를 비교합니다.

공개 예시 CSV만으로 보고서의 20클래스/800개 학습을 재현할 수 있는 것은 아닙니다. 원본 코드의 경로 변수는 각 환경에 맞춰 변경해야 합니다. 이번 정리에서 모델 파일을 새로 로드하거나 학습을 실행하지 않았습니다.

## 통합 전 맞춰야 하는 항목

| 항목 | 보존 자료의 확인 지점 |
|---|---|
| 특징 순서·통계량 | 보고서 PC 예시의 구간별 Flex→IMU→norm과 B의 IMU 구간→norm 구간→Flex 구간 순서, IMU std/variance 차이 |
| 정수 scale | B의 FEAT_QSCALE=512와 헤더의 1000배 표기, weight/bias scale |
| clock | 보고서 72 MHz 공통 표기와 A/B/C 소스 설정의 차이 |
| 통신 주기 | B의 센서 취득과 blocking SPI, C의 polling 주기를 함께 확인 |
| C 설정 | main.h·프로젝트 설정·GPIO/LCD 배선·SPI 두 경로 확인 |

## 하드웨어 확인 순서

A의 ADC/DMA·callback·프레임 간격 → B의 실제 센서 윈도우 → PC/MCU 특징과 점수 일치 → A/B–C 코드와 모드 전달 → LED/LCD 상태 순으로 확인할 수 있습니다. DMA 비교를 재측정한다면 ADC polling과 timer-only 조건을 따로 두어 구간을 분리합니다.

이 순서는 재현 계획입니다. 이번 공개 정리에서는 소스/버전 대조와 문서 경로 확인을 수행했으며, 펌웨어 빌드·업로드·장치 재시험·모델 재학습은 수행하지 않았습니다.
