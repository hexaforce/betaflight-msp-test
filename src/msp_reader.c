#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// MSPコマンド定義
// #define MSP_IDENT 100
// #define MSP_STATUS 101
#define MSP_RAW_IMU 102
// #define MSP_SERVO 103
// #define MSP_MOTOR 104
// #define MSP_RC 105
#define MSP_RAW_GPS 106
#define MSP_COMP_GPS 107
#define MSP_ATTITUDE 108
#define MSP_ALTITUDE 109
#define MSP_ANALOG 110
// #define MSP_RC_TUNING 111
// #define MSP_PID 112
// #define MSP_COMPASS_CONFIG 133

typedef struct {
  int serial_fd;
} MSP;

// ミリ秒単位でスリープ
void sleep_ms(int milliseconds) {
  struct timespec ts;
  ts.tv_sec = milliseconds / 1000;
  ts.tv_nsec = (milliseconds % 1000) * 1000000;
  nanosleep(&ts, NULL);
}

// MSP初期化
int msp_init(MSP *msp, const char *port, speed_t baudrate) {
  msp->serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
  if (msp->serial_fd < 0) {
    fprintf(stderr, "シリアルポートを開けません: %s\n", port);
    return 0;
  }

  // シリアルポート設定
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(msp->serial_fd, &tty) != 0) {
    fprintf(stderr, "tcgetattr エラー\n");
    return 0;
  }

  cfsetospeed(&tty, baudrate);
  cfsetispeed(&tty, baudrate);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8ビット
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5; // 0.5秒タイムアウト

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(msp->serial_fd, TCSANOW, &tty) != 0) {
    fprintf(stderr, "tcsetattr エラー\n");
    return 0;
  }

  // 接続安定化のための待機
  sleep(2);
  printf("MSP接続成功: %s\n", port);
  return 1;
}

// MSP終了処理
void msp_close(MSP *msp) {
  if (msp->serial_fd >= 0) {
    close(msp->serial_fd);
  }
}

// MSPコマンド送信
int msp_send_command(MSP *msp, uint8_t cmd, const uint8_t *data,
                     uint8_t data_size) {
  uint8_t checksum = data_size ^ cmd;

  // パケット構築: $M< + size + cmd + data + checksum
  uint8_t packet[256];
  int idx = 0;
  packet[idx++] = '$';
  packet[idx++] = 'M';
  packet[idx++] = '<';
  packet[idx++] = data_size;
  packet[idx++] = cmd;

  for (int i = 0; i < data_size; i++) {
    packet[idx++] = data[i];
    checksum ^= data[i];
  }
  packet[idx++] = checksum;

  // 送信
  ssize_t written = write(msp->serial_fd, packet, idx);
  return written == idx;
}

// MSP応答受信
int msp_receive_response(MSP *msp, uint8_t *response, int max_size,
                         int timeout_ms) {
  uint8_t buffer;
  struct timespec start, now;
  clock_gettime(CLOCK_MONOTONIC, &start);

  // ヘッダー待機: $M>
  while (1) {
    if (read(msp->serial_fd, &buffer, 1) == 1) {
      if (buffer == '$') {
        if (read(msp->serial_fd, &buffer, 1) == 1 && buffer == 'M') {
          if (read(msp->serial_fd, &buffer, 1) == 1 && buffer == '>') {
            break; // ヘッダー発見
          }
        }
      }
    }

    clock_gettime(CLOCK_MONOTONIC, &now);
    long elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                   (now.tv_nsec - start.tv_nsec) / 1000000;
    if (elapsed > timeout_ms) {
      fprintf(stderr, "タイムアウト: ヘッダーが見つかりません\n");
      return 0;
    }
  }

  // サイズ読み取り
  uint8_t size;
  if (read(msp->serial_fd, &size, 1) != 1) {
    return 0;
  }

  // コマンド読み取り
  uint8_t cmd;
  if (read(msp->serial_fd, &cmd, 1) != 1) {
    return 0;
  }

  // データ読み取り
  uint8_t checksum = size ^ cmd;
  for (int i = 0; i < size && i < max_size; i++) {
    if (read(msp->serial_fd, &buffer, 1) == 1) {
      response[i] = buffer;
      checksum ^= buffer;
    } else {
      return 0;
    }
  }

  // チェックサム検証
  uint8_t received_checksum;
  if (read(msp->serial_fd, &received_checksum, 1) != 1 ||
      checksum != received_checksum) {
    fprintf(stderr, "チェックサムエラー\n");
    return 0;
  }

  return size;
}

// 姿勢データ取得（ロール、ピッチ、ヨー）
int msp_get_attitude(MSP *msp, int16_t *roll, int16_t *pitch, int16_t *yaw) {
  if (!msp_send_command(msp, MSP_ATTITUDE, NULL, 0)) {
    return 0;
  }

  uint8_t response[64];
  int size = msp_receive_response(msp, response, sizeof(response), 1000);
  if (size >= 6) {
    *roll = response[0] | (response[1] << 8);
    *pitch = response[2] | (response[3] << 8);
    *yaw = response[4] | (response[5] << 8);
    return 1;
  }
  return 0;
}

// IMUデータ取得
int msp_get_raw_imu(MSP *msp, int16_t acc[3], int16_t gyro[3], int16_t mag[3]) {
  if (!msp_send_command(msp, MSP_RAW_IMU, NULL, 0)) {
    return 0;
  }

  uint8_t response[64];
  int size = msp_receive_response(msp, response, sizeof(response), 1000);
  if (size >= 18) {
    for (int i = 0; i < 3; i++) {
      acc[i] = response[i * 2] | (response[i * 2 + 1] << 8);
      gyro[i] = response[6 + i * 2] | (response[6 + i * 2 + 1] << 8);
      mag[i] = response[12 + i * 2] | (response[12 + i * 2 + 1] << 8);
    }
    return 1;
  }
  return 0;
}

// バッテリー情報取得
int msp_get_analog(MSP *msp, uint8_t *vbat, uint16_t *mAh, uint16_t *rssi,
                   uint16_t *amperage) {
  if (!msp_send_command(msp, MSP_ANALOG, NULL, 0)) {
    return 0;
  }

  uint8_t response[64];
  int size = msp_receive_response(msp, response, sizeof(response), 1000);
  if (size >= 7) {
    *vbat = response[0];
    *mAh = response[1] | (response[2] << 8);
    *rssi = response[3] | (response[4] << 8);
    *amperage = response[5] | (response[6] << 8);
    return 1;
  }
  return 0;
}

// GPS生データ取得
int msp_get_raw_gps(MSP *msp, uint8_t *fix, uint8_t *num_sat, int32_t *lat,
                    int32_t *lon, int16_t *alt, int16_t *speed,
                    int16_t *ground_course) {
  if (!msp_send_command(msp, MSP_RAW_GPS, NULL, 0)) {
    return 0;
  }

  uint8_t response[64];
  int size = msp_receive_response(msp, response, sizeof(response), 1000);
  if (size >= 16) {
    *fix = response[0];
    *num_sat = response[1];
    *lat = response[2] | (response[3] << 8) | (response[4] << 16) |
           (response[5] << 24);
    *lon = response[6] | (response[7] << 8) | (response[8] << 16) |
           (response[9] << 24);
    *alt = response[10] | (response[11] << 8);
    *speed = response[12] | (response[13] << 8);
    *ground_course = response[14] | (response[15] << 8);
    return 1;
  }
  return 0;
}

// コンパスGPS（ホームまでの距離と方向）
int msp_get_comp_gps(MSP *msp, uint16_t *distance_to_home,
                     int16_t *direction_to_home) {
  if (!msp_send_command(msp, MSP_COMP_GPS, NULL, 0)) {
    return 0;
  }

  uint8_t response[64];
  int size = msp_receive_response(msp, response, sizeof(response), 1000);
  if (size >= 4) {
    *distance_to_home = response[0] | (response[1] << 8);
    *direction_to_home = response[2] | (response[3] << 8);
    return 1;
  }
  return 0;
}

// 高度情報取得
int msp_get_altitude(MSP *msp, int32_t *estimated_alt, int16_t *vario) {
  if (!msp_send_command(msp, MSP_ALTITUDE, NULL, 0)) {
    return 0;
  }

  uint8_t response[64];
  int size = msp_receive_response(msp, response, sizeof(response), 1000);
  if (size >= 6) {
    *estimated_alt = response[0] | (response[1] << 8) | (response[2] << 16) |
                     (response[3] << 24);
    *vario = response[4] | (response[5] << 8);
    return 1;
  }
  return 0;
}

// シリアルポート自動検出
const char *detect_serial_port() {
  // よくあるシリアルポートのリスト
  const char *candidates[] = {"/dev/ttyACM0", // Linux
                              "/dev/ttyACM1",
                              "/dev/ttyUSB0",
                              "/dev/ttyUSB1",
                              "/dev/cu.usbmodem14201", // Mac
                              "/dev/cu.usbmodem3565375A34301",
                              "/dev/cu.usbserial-0001",
                              NULL};

  for (int i = 0; candidates[i] != NULL; i++) {
    int fd = open(candidates[i], O_RDWR | O_NOCTTY);
    if (fd >= 0) {
      close(fd);
      printf("シリアルポート検出: %s\n", candidates[i]);
      return candidates[i];
    }
  }

  return NULL;
}

// 使用例
int main() {
  MSP msp;

  // シリアルポート自動検出
  const char *port = detect_serial_port();
  if (port == NULL) {
    fprintf(stderr, "エラー: シリアルポートが見つかりません\n");
    fprintf(stderr, "手動で指定する場合:\n");
    fprintf(stderr, "  Linux: /dev/ttyACM0 または /dev/ttyUSB0\n");
    fprintf(stderr, "  Mac: /dev/cu.usbmodem* または /dev/cu.usbserial*\n");
    return 1;
  }

  if (!msp_init(&msp, port, B115200)) {
    return 1;
  }

  printf("Betaflight MSP データ取得開始...\n");

  while (1) {
    // 姿勢データ取得
    int16_t roll, pitch, yaw;
    if (msp_get_attitude(&msp, &roll, &pitch, &yaw)) {
      printf("姿勢 - Roll: %.1f° Pitch: %.1f° Yaw: %d°\n", roll / 10.0,
             pitch / 10.0, yaw);
    }

    // IMUデータ取得
    int16_t acc[3], gyro[3], mag[3];
    if (msp_get_raw_imu(&msp, acc, gyro, mag)) {
      printf("加速度 - X: %d Y: %d Z: %d\n", acc[0], acc[1], acc[2]);
      printf("ジャイロ - X: %d Y: %d Z: %d\n", gyro[0], gyro[1], gyro[2]);
      printf("磁力計 - X: %d Y: %d Z: %d\n", mag[0], mag[1], mag[2]);
    }

    // GPS生データ取得
    uint8_t gps_fix, num_sat;
    int32_t lat, lon;
    int16_t alt, speed, ground_course;
    if (msp_get_raw_gps(&msp, &gps_fix, &num_sat, &lat, &lon, &alt, &speed,
                        &ground_course)) {
      printf("GPS - Fix: %d, 衛星数: %d\n", gps_fix, num_sat);
      printf("      位置: %.6f°, %.6f° (高度: %dm)\n", lat / 10000000.0,
             lon / 10000000.0, alt);
      printf("      速度: %dcm/s, 進路: %d°\n", speed, ground_course);
    }

    // コンパスGPS（ホーム方向）
    uint16_t distance_to_home;
    int16_t direction_to_home;
    if (msp_get_comp_gps(&msp, &distance_to_home, &direction_to_home)) {
      printf("ホーム - 距離: %dm, 方向: %d°\n", distance_to_home,
             direction_to_home);
    }

    // 高度情報
    int32_t estimated_alt;
    int16_t vario;
    if (msp_get_altitude(&msp, &estimated_alt, &vario)) {
      printf("高度計 - 推定高度: %dcm, 昇降速度: %dcm/s\n", estimated_alt,
             vario);
    }

    // バッテリー情報取得
    uint8_t vbat;
    uint16_t mAh, rssi, amperage;
    if (msp_get_analog(&msp, &vbat, &mAh, &rssi, &amperage)) {
      printf("バッテリー電圧: %.1fV 消費電力: %dmAh\n", vbat / 10.0, mAh);
    }

    printf("---\n");
    sleep_ms(100);
  }

  msp_close(&msp);
  return 0;
}