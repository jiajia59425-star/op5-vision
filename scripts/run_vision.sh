#!/bin/sh
# 在仓库根目录启动融合服务。模型、串口和标定都看 config/fusion.json。
cd "$(dirname "$0")/.."
exec python3 -m vision.service --config config/fusion.json
