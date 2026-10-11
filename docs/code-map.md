# 코드 안내와 원본 선택

보존 저장소의 기준 커밋은 `c8618b1caef9cdf128ed20f62692d63818d9dd61`입니다. [source-manifest.json](source-manifest.json)에 원본 경로·hash와 복원 여부를 기록했습니다.

| 공개 경로 | 기능 | 선택 기준 |
|---|---|---|
| [A main.c](../firmware/board_a/Core/Src/main.c) | ADC scan/DMA, 완료 구간 처리, 방향 판정·SPI | ADC/DMA 시작·callback·process_frame 경로가 실제 남은 소스 |
| [B main.c](../firmware/board_b/Core/Src/main.c) | Flex·IMU 취득, 특징·분류, SPI2 | 센서별 구현·모델 헤더와 함께 보존된 프로젝트 |
| [B_FeatureExtract](../firmware/board_b/Core/Src/main.c) | main.c 안의 185차원 윈도우 특징 계산 | 학습 입력과 비교해야 하는 순서·통계량·scale |
| [수집용 B main.c](../firmware/board_b_collection/Core/Src/main.c) | 학습용 센서 윈도우 출력 | 추론용 B와 데이터 수집 경로를 구분 |
| [train_gesture_all.py](../tools/train_gesture_all.py) | features.npy/labels.npy 로드·표준화·실제 모델 학습 | `csv/제출용`의 실제 `fit` 경로 |
| [collect_data.py](../tools/collect_data.py) | 시리얼 센서 윈도우 수집 | 제출용 수집 도구 |
| [C reference_main.c](../firmware/board_c/reference_main.c) | SPI 두 경로, LED·LCD 표시 | 원보고서 pp74–92에서 복원한 참조 코드 |

## A의 버전과 C의 복원

`final` 폴더명만으로 선택하지 않았습니다. `A_board_final_1/A_board_draft`는 `HAL_ADC_Start_DMA`, half/full callback, `process_frame`과 전방/후방 에너지 조건이 함께 남아 있어 공개 기준으로 선택했습니다. `Desktop/잡/A_board_final_2/A_board_draft`도 DMA가 동작하는 후보지만 전방 에너지 필터까지 설명하는 데는 전자가 적합합니다.

C보드의 독립 CubeIDE 프로젝트는 확보하지 못했습니다. 보고서에서 페이지 구분을 제거해 `reference_main.c`로 복원했으며, LCD 제어 함수는 이 코드 안에 있습니다. `main.h`, `.ioc`, HAL/CMSIS·startup·linker 등의 독립 프로젝트 설정은 완비되지 않았습니다. PDF 첫 주석의 잘린 comment opener와 말미 `assert_failed`의 중괄호·`#endif`를 복원한 부분은 코드 주석과 manifest에 표시했습니다. 기능 로직을 새로 설계한 것은 아니며, 이 파일을 바로 빌드·실행할 수 있다고 주장하지 않습니다.

## 학습 자료의 선택

`csv/train_gesture_all.py`의 시험용 임의 예측·무작위 모델 경로와 더미 생성물은 제외했습니다. 공개하는 학습 코드는 `csv/제출용`의 실제 NumPy 데이터 로드·표준화·`LogisticRegression.fit`을 확인한 버전입니다. 원본의 `02_extract_features.py`는 기본 입력이 dummy 데이터 폴더라 공개 파이프라인에 포함하지 않았습니다. 실제 CSV→features.npy/labels.npy의 최종 생성 조합은 확정되지 않았습니다.

[example_sensor_window.csv](../data/example_sensor_window.csv)는 `bboard_csv/COME_003.csv`의 실제 보존 예시입니다. 이 CSV에는 Flex와 accel 열만 있고 gyro 열이 없어, 보고서의 185차원 PC 특징 입력을 그대로 만족하지 않습니다. 전체 20클래스/800개 데이터셋을 완비한 것으로 설명하지 않습니다. `feature.c`의 보조 함수는 순간 Flex/IMU 값 11개를 반환하는 함수이며 main.c의 185차원 윈도우 추출과 구분합니다.

## 코드 변경 범위

A/B와 Python 파일은 원본을 선별해 복사했습니다. 이 공개 정리에서 특징 순서·scale을 임의로 맞추거나 모델을 재학습하지 않았습니다. C는 보고서 추출에 따른 참조 복원이며 출처·수정 범위를 따로 기록했습니다.
