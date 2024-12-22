#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

// (상수 정의는 기존 코드와 동일합니다)

typedef struct {
    int client_fd;
} ThreadArgs;

// 전역 변수
int temperature, humidity;
int adxl345_i2c_fd;
int is_alarm_triggered = 0;
short prev_x_SPI = 0, prev_y_SPI = 0, prev_z_SPI = 0;
short prev_x_I2C = 0, prev_y_I2C = 0, prev_z_I2C = 0;

// 함수 선언
void *handle_dht_sensor(void *arg);
void *handle_adxl345_sensor(void *arg);
void *handle_socket_communication(void *arg);

// DHT 센서 읽기 스레드
void *handle_dht_sensor(void *arg) {
    int client_fd = ((ThreadArgs *)arg)->client_fd;
    char buffer[BUFFER_SIZE];

    while (1) {
        read_dht_data(&temperature, &humidity);
        snprintf(buffer, BUFFER_SIZE, "현재 온도: %d°C, 현재 습도: %d%%\n", temperature, humidity);
        send(client_fd, buffer, strlen(buffer), 0);
        printf("온습도 데이터 전송됨: %s", buffer);

        if ((temperature <= 20 || temperature >= 30) || (humidity <= 30 || humidity >= 40)) {
            snprintf(buffer, BUFFER_SIZE, "온습도 경고: 온도=%d°C, 습도=%d%%\n", temperature, humidity);
            send(client_fd, buffer, strlen(buffer), 0);
            printf("온습도 경고 전송됨: %s", buffer);
        }

        delay(2000);
    }

    pthread_exit(NULL);
}

// ADXL345 센서 처리 스레드
void *handle_adxl345_sensor(void *arg) {
    int client_fd = ((ThreadArgs *)arg)->client_fd;
    unsigned char buffer_accel_SPI[6];
    char buffer[BUFFER_SIZE];

    while (1) {
        // SPI 데이터 읽기
        memset(buffer_accel_SPI, 0, sizeof(buffer_accel_SPI));
        readRegister_ADXL345_SPI(DATAX0, 6, buffer_accel_SPI);

        short x_SPI = (((short)buffer_accel_SPI[1] << 8) | (short)buffer_accel_SPI[2]) / 100;
        short y_SPI = (((short)buffer_accel_SPI[3] << 8) | (short)buffer_accel_SPI[4]) / 100;
        short z_SPI = (((short)buffer_accel_SPI[5] << 8) | (short)buffer_accel_SPI[6]) / 100;

        // I2C 데이터 읽기
        short x_I2C = readADXL345_I2C(adxl345_i2c_fd, ADXL345_I2C_DATAX0) / 100;
        short y_I2C = readADXL345_I2C(adxl345_i2c_fd, ADXL345_I2C_DATAY0) / 100;
        short z_I2C = readADXL345_I2C(adxl345_i2c_fd, ADXL345_I2C_DATAZ0) / 100;

        printf("ADXL345 데이터: SPI(X=%d, Y=%d, Z=%d), I2C(X=%d, Y=%d, Z=%d)\n",
               x_SPI, y_SPI, z_SPI, x_I2C, y_I2C, z_I2C);

        if (!is_alarm_triggered &&
            (abs(x_SPI - prev_x_SPI) >= THRESHOLD || abs(y_SPI - prev_y_SPI) >= THRESHOLD || abs(z_SPI - prev_z_SPI) >= THRESHOLD ||
             abs(x_I2C - prev_x_I2C) >= THRESHOLD || abs(y_I2C - prev_y_I2C) >= THRESHOLD || abs(z_I2C - prev_z_I2C) >= THRESHOLD)) {

            snprintf(buffer, BUFFER_SIZE, "도난 감지됨!\n");
            blink_LED(5, 200);
            rotate_Servo(ANGLE_CLOSE);
            send(client_fd, buffer, strlen(buffer), 0);
            printf("도난 경고 전송됨: %s", buffer);
            is_alarm_triggered = 1;
        }

        prev_x_SPI = x_SPI;
        prev_y_SPI = y_SPI;
        prev_z_SPI = z_SPI;
        prev_x_I2C = x_I2C;
        prev_y_I2C = y_I2C;
        prev_z_I2C = z_I2C;

        delay(500);
    }

    pthread_exit(NULL);
}

// 소켓 통신 스레드
void *handle_socket_communication(void *arg) {
    int client_fd = ((ThreadArgs *)arg)->client_fd;
    char buffer[BUFFER_SIZE];

    while (1) {
        int bytes_received = recv(client_fd, buffer, BUFFER_SIZE, 0);
        if (bytes_received <= 0) {
            printf("클라이언트 연결 종료\n");
            close(client_fd);
            exit(0);
        }
        buffer[bytes_received] = '\0';
        printf("클라이언트 메시지: %s", buffer);
    }

    pthread_exit(NULL);
}

int main() {
    int server_fd, client_fd;
    struct sockaddr_in address;
    int addr_len = sizeof(address);

    // 초기화
    if (wiringPiSetupGpio() == -1) {
        perror("WiringPi 설정 실패");
        return 1;
    }
    setup_sensors_and_gpio(); // 센서 및 GPIO 초기화 함수 (기존 코드 재사용)

    // 소켓 설정
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("소켓 생성 실패");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("소켓 바인딩 실패");
        close(server_fd);
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("소켓 리스닝 실패");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    printf("클라이언트 연결 대기 중...\n");
    if ((client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addr_len)) < 0) {
        perror("클라이언트 연결 실패");
        close(server_fd);
        exit(EXIT_FAILURE);
    }
    printf("클라이언트 연결됨.\n");

    // 스레드 생성
    pthread_t dht_thread, adxl_thread, socket_thread;
    ThreadArgs args = { .client_fd = client_fd };

    pthread_create(&dht_thread, NULL, handle_dht_sensor, (void *)&args);
    pthread_create(&adxl_thread, NULL, handle_adxl345_sensor, (void *)&args);
    pthread_create(&socket_thread, NULL, handle_socket_communication, (void *)&args);

    pthread_join(dht_thread, NULL);
    pthread_join(adxl_thread, NULL);
    pthread_join(socket_thread, NULL);

    close(client_fd);
    close(server_fd);

    return 0;
}
