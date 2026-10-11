# train_gesture_all.py
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, confusion_matrix

FEATURES_FILE = "features.npy"
LABELS_FILE   = "labels.npy"
MODEL_NPZ     = "gesture_model_all.npz"   # W, b, mean, std, classes 저장용

def main():
    # 1) 데이터 로드
    X = np.load(FEATURES_FILE)   # (N, 185)
    y = np.load(LABELS_FILE)     # (N, )

    print("X shape:", X.shape)
    print("y shape:", y.shape)
    print("클래스 목록:", np.unique(y))

    # 2) train / val 분할
    X_train, X_val, y_train, y_val = train_test_split(
        X, y,
        test_size=0.2,
        random_state=42,
        stratify=y
    )

    # 3) 표준화 (각 feature별 mean, std 저장해야 나중에 STM32에서 동일 처리 가능)
    scaler = StandardScaler()
    X_train_std = scaler.fit_transform(X_train)
    X_val_std   = scaler.transform(X_val)

    # 4) 다중 클래스 로지스틱 회귀 (소프트맥스)
    clf = LogisticRegression(
        max_iter=2000,
        multi_class="multinomial",
        solver="lbfgs"
    )
    clf.fit(X_train_std, y_train)

    # 5) 검증 성능 출력
    y_pred = clf.predict(X_val_std)
    print("\n=== Classification Report ===")
    print(classification_report(y_val, y_pred))

    print("=== Confusion Matrix ===")
    print(confusion_matrix(y_val, y_pred))

    # 6) 파라미터 저장 (STM32에서 쓸 핵심 값들)
    W = clf.coef_.astype(np.float32)        # (n_classes, 185)
    b = clf.intercept_.astype(np.float32)   # (n_classes,)
    mean = scaler.mean_.astype(np.float32)  # (185,)
    scale = scaler.scale_.astype(np.float32)# (185,)
    classes = clf.classes_                  # (n_classes,)

    np.savez(
        MODEL_NPZ,
        W=W,
        b=b,
        mean=mean,
        scale=scale,
        classes=classes
    )

    print(f"\n모델 파라미터 저장 완료 -> {MODEL_NPZ}")
    print("W shape:", W.shape)
    print("b shape:", b.shape)
    print("mean/scale shape:", mean.shape)

if __name__ == "__main__":
    main()
