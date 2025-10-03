#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>

// MSPコマンド定義
#define MSP_IDENT       100
#define MSP_STATUS      101
#define MSP_RAW_IMU     102
#define MSP_MOTOR       104
#define MSP_RC          105
#define MSP_RAW_GPS     106
#define MSP_ATTITUDE    108
#define MSP_ALTITUDE    109
#define MSP_ANALOG      110

class MSP {
private:
    int serial_fd;
    
public:
    // コンストラクタ
    MSP(const char* port = "/dev/ttyACM0", int baudrate = B115200) {
        serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd < 0) {
            std::cerr << "シリアルポートを開けません: " << port << std::endl;
            return;
        }
        
        // シリアルポート設定
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(serial_fd, &tty) != 0) {
            std::cerr << "tcgetattr エラー" << std::endl;
            return;
        }
        
        cfsetospeed(&tty, baudrate);
        cfsetispeed(&tty, baudrate);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8ビット
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;  // 0.5秒タイムアウト
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            std::cerr << "tcsetattr エラー" << std::endl;
            return;
        }
        
        // 接続安定化のための待機
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "MSP接続成功: " << port << std::endl;
    }
    
    // デストラクタ
    ~MSP() {
        if (serial_fd >= 0) {
            close(serial_fd);
        }
    }
    
    // MSPコマンド送信
    bool sendCommand(uint8_t cmd, const std::vector<uint8_t>& data = {}) {
        uint8_t size = data.size();
        uint8_t checksum = size ^ cmd;
        
        // パケット構築: $M< + size + cmd + data + checksum
        std::vector<uint8_t> packet = {'$', 'M', '<', size, cmd};
        
        for (uint8_t byte : data) {
            packet.push_back(byte);
            checksum ^= byte;
        }
        packet.push_back(checksum);
        
        // 送信
        ssize_t written = write(serial_fd, packet.data(), packet.size());
        return written == (ssize_t)packet.size();
    }
    
    // MSP応答受信
    std::vector<uint8_t> receiveResponse(int timeout_ms = 1000) {
        std::vector<uint8_t> response;
        uint8_t buffer;
        auto start = std::chrono::steady_clock::now();
        
        // ヘッダー待機: $M>
        while (true) {
            if (read(serial_fd, &buffer, 1) == 1) {
                if (buffer == '$') {
                    if (read(serial_fd, &buffer, 1) == 1 && buffer == 'M') {
                        if (read(serial_fd, &buffer, 1) == 1 && buffer == '>') {
                            break;  // ヘッダー発見
                        }
                    }
                }
            }
            
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
                std::cerr << "タイムアウト: ヘッダーが見つかりません" << std::endl;
                return response;
            }
        }
        
        // サイズ読み取り
        uint8_t size;
        if (read(serial_fd, &size, 1) != 1) {
            return response;
        }
        
        // コマンド読み取り
        uint8_t cmd;
        if (read(serial_fd, &cmd, 1) != 1) {
            return response;
        }
        
        // データ読み取り
        uint8_t checksum = size ^ cmd;
        for (int i = 0; i < size; i++) {
            if (read(serial_fd, &buffer, 1) == 1) {
                response.push_back(buffer);
                checksum ^= buffer;
            } else {
                return std::vector<uint8_t>();
            }
        }
        
        // チェックサム検証
        uint8_t received_checksum;
        if (read(serial_fd, &received_checksum, 1) != 1 || checksum != received_checksum) {
            std::cerr << "チェックサムエラー" << std::endl;
            return std::vector<uint8_t>();
        }
        
        return response;
    }
    
    // 姿勢データ取得（ロール、ピッチ、ヨー）
    bool getAttitude(int16_t& roll, int16_t& pitch, int16_t& yaw) {
        if (!sendCommand(MSP_ATTITUDE)) {
            return false;
        }
        
        auto response = receiveResponse();
        if (response.size() >= 6) {
            roll = response[0] | (response[1] << 8);
            pitch = response[2] | (response[3] << 8);
            yaw = response[4] | (response[5] << 8);
            return true;
        }
        return false;
    }
    
    // IMUデータ取得
    bool getRawIMU(int16_t acc[3], int16_t gyro[3], int16_t mag[3]) {
        if (!sendCommand(MSP_RAW_IMU)) {
            return false;
        }
        
        auto response = receiveResponse();
        if (response.size() >= 18) {
            for (int i = 0; i < 3; i++) {
                acc[i] = response[i*2] | (response[i*2+1] << 8);
                gyro[i] = response[6+i*2] | (response[6+i*2+1] << 8);
                mag[i] = response[12+i*2] | (response[12+i*2+1] << 8);
            }
            return true;
        }
        return false;
    }
    
    // バッテリー情報取得
    bool getAnalog(uint8_t& vbat, uint16_t& mAh, uint16_t& rssi, uint16_t& amperage) {
        if (!sendCommand(MSP_ANALOG)) {
            return false;
        }
        
        auto response = receiveResponse();
        if (response.size() >= 7) {
            vbat = response[0];
            mAh = response[1] | (response[2] << 8);
            rssi = response[3] | (response[4] << 8);
            amperage = response[5] | (response[6] << 8);
            return true;
        }
        return false;
    }
};

// 使用例
int main() {
    MSP msp("/dev/ttyACM0");
    
    std::cout << "Betaflight MSP データ取得開始..." << std::endl;
    
    while (true) {
        // 姿勢データ取得
        int16_t roll, pitch, yaw;
        if (msp.getAttitude(roll, pitch, yaw)) {
            std::cout << "姿勢 - Roll: " << roll/10.0 << "° "
                      << "Pitch: " << pitch/10.0 << "° "
                      << "Yaw: " << yaw << "°" << std::endl;
        }
        
        // IMUデータ取得
        int16_t acc[3], gyro[3], mag[3];
        if (msp.getRawIMU(acc, gyro, mag)) {
            std::cout << "加速度 - X: " << acc[0] << " Y: " << acc[1] << " Z: " << acc[2] << std::endl;
            std::cout << "ジャイロ - X: " << gyro[0] << " Y: " << gyro[1] << " Z: " << gyro[2] << std::endl;
        }
        
        // バッテリー情報取得
        uint8_t vbat;
        uint16_t mAh, rssi, amperage;
        if (msp.getAnalog(vbat, mAh, rssi, amperage)) {
            std::cout << "バッテリー電圧: " << vbat/10.0 << "V "
                      << "消費電力: " << mAh << "mAh" << std::endl;
        }
        
        std::cout << "---" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}