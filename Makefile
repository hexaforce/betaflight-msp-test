# Makefile - Docker用 Betaflight MSP Reader
# このMakefileはホスト側から使用し、Docker内でコンパイル・実行します
#
# 使い方:
#   make setup    : 初回セットアップ（ディレクトリ作成、Docker構築）
#   make build    : Dockerイメージ構築
#   make up       : コンテナ起動
#   make compile  : Docker内でコンパイル
#   make run      : Docker内でプログラム実行
#   make shell    : コンテナにログイン
#   make down     : コンテナ停止
#   make logs     : ログ表示
#   make clean    : ビルドファイル削除
#   make rebuild  : クリーンビルド

# Docker Compose コマンド
DOCKER_COMPOSE = docker-compose
CONTAINER_NAME = betaflight-dev
BUILD_DIR = build
SRC_DIR = src

# カラー出力
GREEN = \033[0;32m
YELLOW = \033[0;33m
NC = \033[0m # No Color

.PHONY: help setup build up down compile run shell logs clean rebuild restart check-device all

# デフォルトターゲット
all: up compile
	@echo "$(GREEN)========================================"
	@echo "セットアップ完了！"
	@echo "実行: make run"
	@echo "========================================$(NC)"

# ヘルプ
help:
	@echo "$(GREEN)Betaflight MSP Docker 開発環境$(NC)"
	@echo ""
	@echo "使用可能なコマンド:"
	@echo "  $(YELLOW)make setup$(NC)      - 初回セットアップ"
	@echo "  $(YELLOW)make build$(NC)      - Dockerイメージ構築"
	@echo "  $(YELLOW)make up$(NC)         - コンテナ起動"
	@echo "  $(YELLOW)make compile$(NC)    - コンパイル"
	@echo "  $(YELLOW)make run$(NC)        - プログラム実行"
	@echo "  $(YELLOW)make shell$(NC)      - コンテナにログイン"
	@echo "  $(YELLOW)make down$(NC)       - コンテナ停止"
	@echo "  $(YELLOW)make logs$(NC)       - ログ表示"
	@echo "  $(YELLOW)make clean$(NC)      - ビルドファイル削除"
	@echo "  $(YELLOW)make rebuild$(NC)    - クリーンビルド"
	@echo "  $(YELLOW)make restart$(NC)    - コンテナ再起動"
	@echo "  $(YELLOW)make check-device$(NC) - USBデバイス確認"

# 初回セットアップ
setup:
	@echo "$(GREEN)初回セットアップ中...$(NC)"
	@mkdir -p $(SRC_DIR) $(BUILD_DIR)
	@if [ ! -f "$(SRC_DIR)/msp_reader.cpp" ]; then \
		echo "$(YELLOW)警告: $(SRC_DIR)/msp_reader.cpp が見つかりません$(NC)"; \
		echo "msp_reader.cpp を $(SRC_DIR)/ に配置してください"; \
	fi
	@echo "$(GREEN)Dockerイメージ構築中...$(NC)"
	@$(DOCKER_COMPOSE) build
	@echo "$(GREEN)セットアップ完了！$(NC)"
	@echo "次のコマンド: make up"

# Dockerイメージ構築
build:
	@echo "$(GREEN)Dockerイメージ構築中...$(NC)"
	@$(DOCKER_COMPOSE) build
	@echo "$(GREEN)構築完了$(NC)"

# コンテナ起動
up:
	@echo "$(GREEN)コンテナ起動中...$(NC)"
	@$(DOCKER_COMPOSE) up -d
	@echo "$(GREEN)コンテナ起動完了$(NC)"
	@echo "次のコマンド: make compile"

# コンテナ停止
down:
	@echo "$(YELLOW)コンテナ停止中...$(NC)"
	@$(DOCKER_COMPOSE) down
	@echo "$(GREEN)停止完了$(NC)"

# Docker内でコンパイル
compile:
	@echo "$(GREEN)Docker内でコンパイル中...$(NC)"
	@$(DOCKER_COMPOSE) exec $(CONTAINER_NAME) bash -c "\
		mkdir -p build && \
		g++ -o build/msp_reader src/msp_reader.cpp -std=c++11 -Wall -Wextra -O2 -lpthread && \
		echo '$(GREEN)コンパイル成功！$(NC)' || \
		echo '$(YELLOW)コンパイルエラー$(NC)' \
	"

# Docker内でプログラム実行
run:
	@echo "$(GREEN)========================================"
	@echo "プログラム実行中..."
	@echo "終了: Ctrl+C"
	@echo "========================================$(NC)"
	@$(DOCKER_COMPOSE) exec $(CONTAINER_NAME) ./build/msp_reader

# コンテナにログイン
shell:
	@echo "$(GREEN)コンテナにログイン中...$(NC)"
	@echo "終了: exit または Ctrl+D"
	@$(DOCKER_COMPOSE) exec $(CONTAINER_NAME) bash

# ログ表示
logs:
	@$(DOCKER_COMPOSE) logs -f

# ビルドファイル削除
clean:
	@echo "$(YELLOW)ビルドファイル削除中...$(NC)"
	@rm -rf $(BUILD_DIR)/*
	@echo "$(GREEN)削除完了$(NC)"

# クリーンビルド
rebuild: clean compile
	@echo "$(GREEN)クリーンビルド完了$(NC)"

# コンテナ再起動
restart: down up
	@echo "$(GREEN)再起動完了$(NC)"

# USBデバイス確認
check-device:
	@echo "$(GREEN)ホスト側のUSBデバイス:$(NC)"
	@ls -l /dev/tty* | grep -E "ttyACM|ttyUSB" || echo "$(YELLOW)USBデバイスが見つかりません$(NC)"
	@echo ""
	@echo "$(GREEN)Docker内のUSBデバイス:$(NC)"
	@$(DOCKER_COMPOSE) exec $(CONTAINER_NAME) ls -l /dev/tty* | grep -E "ttyACM|ttyUSB" || echo "$(YELLOW)Docker内にUSBデバイスがマウントされていません$(NC)"

# デバッグビルド
debug:
	@echo "$(GREEN)デバッグビルド中...$(NC)"
	@$(DOCKER_COMPOSE) exec $(CONTAINER_NAME) bash -c "\
		mkdir -p build && \
		g++ -o build/msp_reader src/msp_reader.cpp -std=c++11 -Wall -Wextra -g -O0 -lpthread && \
		echo '$(GREEN)デバッグビルド成功！$(NC)' \
	"
