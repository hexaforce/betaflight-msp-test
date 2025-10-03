# Dockerfile
FROM debian:bullseye

# 必要なパッケージのインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    g++ \
    cmake \
    git \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# 作業ディレクトリ作成
WORKDIR /workspace

# デフォルトコマンド
CMD ["/bin/bash"]
