#!/bin/bash
# 看门狗脚本 - 进程崩溃时自动重启

PROGRAM="./build/standard_mpc"
CONFIG="configs/standard3.yaml"
LOG_DIR="logs"
MAX_RESTART=100  # 最大重启次数
RESTART_COUNT=0

# Ctrl+C 时优雅退出，不重启
cleanup() {
    echo "[Watchdog][$(date)] Caught Ctrl+C, exiting gracefully"
    exit 0
}
trap cleanup SIGINT

mkdir -p "$LOG_DIR"

echo "[Watchdog] Starting at $(date)"
echo "[Watchdog] Program: $PROGRAM $CONFIG"

while [ $RESTART_COUNT -lt $MAX_RESTART ]; do
    TIMESTAMP=$(date "+%Y-%m-%d_%H-%M-%S")
    LOGFILE="$LOG_DIR/${TIMESTAMP}.log"

    echo "[Watchdog][$(date)] Starting program (attempt $((RESTART_COUNT+1))/$MAX_RESTART)"

    # 运行程序
    $PROGRAM "$CONFIG" 2>&1 | tee "$LOGFILE"

    EXIT_CODE=$?

    # 检查是否是因为 Ctrl+C 退出（SIGINT=130）
    if [ $EXIT_CODE -eq 130 ] || [ $EXIT_CODE -eq 143 ]; then
        echo "[Watchdog][$(date)] Program interrupted, exiting"
        break
    fi

    if [ $EXIT_CODE -eq 0 ]; then
        echo "[Watchdog][$(date)] Program exited normally"
        break
    else
        echo "[Watchdog][$(date)] Program crashed with code $EXIT_CODE, restarting..."
        RESTART_COUNT=$((RESTART_COUNT+1))
        sleep 2
    fi
done

if [ $RESTART_COUNT -ge $MAX_RESTART ]; then
    echo "[Watchdog][$(date)] Max restart attempts reached, giving up"
fi
