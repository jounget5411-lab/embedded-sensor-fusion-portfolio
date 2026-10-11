import serial
import csv
import time
import os

# ==== [설정] 본인 환경에 맞게 수정하세요 ====
PORT = "COM12"          # 장치 관리자에서 확인한 포트 번호 (예: COM3, COM7 등)
BAUD = 115200           # STM32 UART 설정과 동일해야 함
OUT_DIR = "bboard_csv"  # CSV 파일이 저장될 폴더 이름

GESTURE_NAME = "COME"   # <-- 여기만 바꾸면 파일 이름이 COME_001.csv 이런 식으로 저장됨
# ===========================================

def main():
    # 저장할 폴더가 없으면 생성
    os.makedirs(OUT_DIR, exist_ok=True)

    # 시리얼 포트 연결 시도
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"[연결 성공] Port: {PORT}, Baud: {BAUD}")
        print("데이터 수집 대기 중... (Ctrl+C를 누르면 종료됩니다)")
    except serial.SerialException:
        print(f"[오류] {PORT} 포트를 열 수 없습니다. 포트 번호를 확인하거나 다른 프로그램이 사용 중인지 확인하세요.")
        return

    sample_idx = 0
    in_window = False
    rows = []

    while True:
        try:
            # STM32에서 한 줄 읽기
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            else:
                time.sleep(0.01)
                continue
        except KeyboardInterrupt:
            print("\n[종료] 사용자가 중단했습니다.")
            break
        
        if not line:
            continue

        # 1. 윈도우 시작 신호 감지
        if line == "BEGIN_WINDOW":
            in_window = True
            rows = []
            print("\n>>> 제스처 감지됨! 데이터 기록 시작...")
            continue

        # 2. 윈도우 종료 신호 감지 -> 파일 저장
        if line == "END_WINDOW":
            in_window = False
            sample_idx += 1
            
            # 파일명: COME_001.csv, COME_002.csv ...
            filename = f"{GESTURE_NAME}_{sample_idx:03d}.csv"
            path = os.path.join(OUT_DIR, filename)

            # CSV 쓰기
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                # 헤더(첫 줄) 작성
                writer.writerow(["frame", "flex0", "flex1", "flex2", "flex3", "flex4", "Ax", "Ay", "Az"])
                # 데이터 작성
                writer.writerows(rows)

            print(f"[저장 완료] {filename} (데이터 길이: {len(rows)} rows)")
            print("다음 제스처 대기 중...")
            continue

        # 3. 실제 데이터 파싱 및 임시 저장
        if in_window:
            # 데이터 포맷: frame,flex0,flex1,flex2,flex3,flex4,Ax,Ay,Az
            parts = line.split(",")
            
            # 데이터 개수가 맞는지 확인 (총 9개 데이터가 와야 함)
            if len(parts) < 9: 
                continue

            try:
                # 문자열을 숫자로 변환
                frame = int(parts[0])
                flex = [float(x) for x in parts[1:6]]
                imu  = [float(x) for x in parts[6:9]]
                
                # 리스트에 추가
                rows.append([frame] + flex + imu)
            except ValueError:
                continue

if __name__ == "__main__":
    main()
