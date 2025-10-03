# Makefile - Betaflight MSP Reader
# 使い方:
#   make        : コンパイル
#   make clean  : ビルドファイル削除
#   make run    : コンパイルして実行
#   make rebuild: クリーンビルド

# コンパイラとフラグ
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
LDFLAGS = -lpthread

# ディレクトリとファイル
SRC_DIR = src
BUILD_DIR = build
TARGET = $(BUILD_DIR)/msp_reader
SRC = $(SRC_DIR)/msp_reader.cpp

# デフォルトターゲット
all: $(TARGET)
	@echo ""
	@echo "========================================"
	@echo "コンパイル完了！"
	@echo "実行: ./$(TARGET)"
	@echo "または: make run"
	@echo "========================================"

# コンパイル
$(TARGET): $(SRC)
	@echo "コンパイル中..."
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

# ビルドファイル削除
clean:
	@echo "ビルドファイルを削除中..."
	@rm -rf $(BUILD_DIR)/*
	@echo "削除完了"

# コンパイルして実行
run: $(TARGET)
	@echo ""
	@echo "========================================"
	@echo "プログラム実行中..."
	@echo "========================================"
	@./$(TARGET)

# クリーンビルド
rebuild: clean all

# デバッグビルド（最適化なし、デバッグ情報付き）
debug: CXXFLAGS = -std=c++11 -Wall -Wextra -g -O0
debug: clean $(TARGET)
	@echo "デバッグビルド完了"

# ヘルプ
help:
	@echo "使用可能なコマンド:"
	@echo "  make          - コンパイル"
	@echo "  make clean    - ビルドファイル削除"
	@echo "  make run      - コンパイルして実行"
	@echo "  make rebuild  - クリーンビルド"
	@echo "  make debug    - デバッグビルド"
	@echo "  make help     - このヘルプを表示"

# Phonyターゲット（ファイル名と衝突しないように）
.PHONY: all clean run rebuild debug help