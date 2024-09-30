#include <mbed.h>
#include <iostream>
#include <cmath>
#include <firstpenguin.hpp>
#include <algorithm>
#include <chrono>
#include <thread>

int16_t pwm[3] = {0, 0, 0}; // それぞれの出力
double theta = 0; // PS4コントローラーの左スティックの角度
double L_x; // PS4コントローラーの左スティックのx座標
double L_y; // PS4コントローラーの左スティックのy座標
double R_x; // PS4コントローラーの右スティックのx座標
double R_y; // PS4コントローラーの右スティックのy座標
uint8_t DATA[8] = {};
float speed = 0;
float speed_change = -0.1;
float rotation_change = -0.03;
//ジャンプのための出力
float jump = 0;

constexpr uint32_t penguinID = 35;

int raw_angle1 = 0;
int angle1 = 0;
int raw_angle2 = 0;
int angle2 = 0;
int raw_angle3 = 0;
int angle3 = 0;

int first_angle1 = 0;
int first_angle2 = 0;
int first_angle3 = 0;

double angle_deviation1 = 0;
double angle_deviation2 = 0;
double angle_deviation3 = 0;

double angle_deviation = 0;
double angle = 0;

// constexpr uint32_t can_id = 35;
BufferedSerial pc(USBTX, USBRX, 250000);
CAN can(PA_11, PA_12, (int)1e6); // m2006
CAN can2(PB_12, PB_13, (int)1e6); // firstpenguin
FirstPenguin penguin{penguinID, can2};
CANMessage msg;
Timer timer;

void readUntilPipe(char *output_buf, int output_buf_size)
{
    char buf[20];
    int output_buf_index = 0;
    while (1)
    {
        if (pc.readable())
        {
            ssize_t num = pc.read(buf, sizeof(buf) - 1); // -1 to leave space for null terminator
            buf[num] = '\0';
            for (int i = 0; i < num; i++)
            {
                if (buf[i] == '|')
                {
                    output_buf[output_buf_index] = '\0';
                    return;
                }
                else if (buf[i] != '\n' && output_buf_index < output_buf_size - 1)
                {
                    output_buf[output_buf_index++] = buf[i];
                }
            }
        }
        if (output_buf_index >= output_buf_size - 1) // Prevent buffer overflow
        {
            output_buf[output_buf_index] = '\0';
            return;
        }
    }
}

void canSend()
{
    static int previous_angle1 = 0;
    static int previous_angle2 = 0;
    static int previous_angle3 = 0;
    static bool first_angles_set = false;
    while (1)
    {
        CANMessage msg1;
        while(can.read(msg1))
        {
            if (!first_angles_set)
            {
                // 初期値を設定する
                if (msg1.id == 0x201)
                {
                    first_angle1 = (msg1.data[0] << 8) | msg1.data[1];
                }
                if (msg1.id == 0x202)
                {
                    first_angle2 = (msg1.data[0] << 8) | msg1.data[1];
                }
                if (msg1.id == 0x203)
                {
                    first_angle3 = (msg1.data[0] << 8) | msg1.data[1];
                }
                first_angles_set = true;
                continue; // 角度をセットしたら次のループへ
            }
            // printf("CAN ID: 0x%x\n", msg.id);
            if (R_x != 0)
            {
                if (msg1.id == 0x201)
                {
                    raw_angle1 = ((msg1.data[0] << 8) | msg1.data[1]) - first_angle1;
                    int diff = raw_angle1 - previous_angle1;
                    if (diff > 4095) // ラップアラウンドの巻き戻し
                    {
                        angle1 -= (8192 - raw_angle1);
                    }
                    else if (diff < -4095) // ラップアラウンドの巻き進み
                    {
                        angle1 += (8192 + raw_angle1);
                    }
                    else
                    {
                        angle1 += diff;
                    }
                    previous_angle1 = raw_angle1;
                }
                if (msg1.id == 0x202)
                {
                    raw_angle2 = ((msg1.data[0] << 8) | msg1.data[1]) - first_angle2;
                    int diff = raw_angle2 - previous_angle2;
                    if (diff > 4095) // ラップアラウンドの巻き戻し
                    {
                        angle2 -= (8192 - raw_angle2);
                    }
                    else if (diff < -4095) // ラップアラウンドの巻き進み
                    {
                        angle2 += (8192 + raw_angle2);
                    }
                    else
                    {
                        angle2 += diff;
                    }
                    previous_angle2 = raw_angle2;
                }
                if (msg1.id == 0x203)
                {
                    raw_angle3 = ((msg1.data[0] << 8) | msg1.data[1]) - first_angle3;
                    int diff = raw_angle3 - previous_angle3;
                    if (diff > 4095) // ラップアラウンドの巻き戻し
                    {
                        angle3 -= (8192 - raw_angle3);
                    }
                    else if (diff < -4095) // ラップアラウンドの巻き進み
                    {
                        angle3 += (8192 + raw_angle3);
                    }
                    else
                    {
                        angle3 += diff;
                    }
                    previous_angle3 = raw_angle3;
                }
            }
        }
        
        printf("pulse1: %d, pulse2: %d, pulse3: %d", angle1, angle2, angle3);
        double all_angle = angle1 + angle2 + angle3;
        if (all_angle < 0)
        {
            angle = static_cast<double>(std::max({angle1, angle2, angle3}));
        }
        else
        {
            angle = static_cast<double>(std::min({angle1, angle2, angle3}));
        }
        printf("angle: %f\n", angle);
        
        double rotation_number = angle / (8192.0 * 36);
        double angle_deviation = rotation_number * 39.6606746678 * (M_PI / 180.0);

        // double rotation_number1 = angle1 / (8192.0 * 36);
        // double rotation_number2 = angle2 / (8192.0 * 36);
        // double rotation_number3 = angle3 / (8192.0 * 36);
        // printf("rotation_number1: %f, rotation_number2: %f, rotation_number3: %f\n", rotation_number1, rotation_number2, rotation_number3);

        // double angle_deviation1 = rotation_number1 * 39.6606746678 * (M_PI / 180.0);
        // double angle_deviation2 = rotation_number2 * 39.6606746678 * (M_PI / 180.0);
        // double angle_deviation3 = rotation_number3 * 39.6606746678 * (M_PI / 180.0);

        // PWM値を計算
        pwm[0] = speed_change * speed * cos(theta - 2 * M_PI / 3 + angle_deviation) + (rotation_change * R_x);
        pwm[1] = speed_change * speed * cos(theta - 4 * M_PI / 3 + angle_deviation) + (rotation_change * R_x);
        pwm[2] = speed_change * speed * cos(theta + angle_deviation) + (rotation_change * R_x);

        // ロボマスのid:1
        int16_t outputRightInt16 = static_cast<int16_t>(pwm[0]);
        DATA[0] = outputRightInt16 >> 8;   // MSB
        DATA[1] = outputRightInt16 & 0xFF; // LSB
        // ロボマスのid:2
        int16_t outputLeftInt16 = static_cast<int16_t>(pwm[1]);
        DATA[2] = outputLeftInt16 >> 8;   // MSB
        DATA[3] = outputLeftInt16 & 0xFF;
        // ロボマスのid:3
        int16_t outputLeft16 = static_cast<int16_t>(pwm[2]);
        DATA[4] = outputLeft16 >> 8;   // MSB
        DATA[5] = outputLeft16 & 0xFF; // LSB

        int16_t outputJumpInt16 = static_cast<int16_t>(jump);
        DATA[6] = outputJumpInt16 >> 8;   // MSB
        DATA[7] = outputJumpInt16 & 0xFF; // LSB

        CANMessage msg0(0x200, DATA, 8);
        can.write(msg0);
        penguin.send();
        // if(penguin.send()){
            
        // }
        // else{
        //     printf("no\n");
        // }
        ThisThread::sleep_for(50ms);
    }
}


int main() {
    Thread thread;
    thread.start(canSend);//canSend関数を実行

    if (!can.frequency(1e6))
    {
        printf("CANの初期化に失敗しました\n");
        return 1;
    }
    char output_buf[20]; // 出力用のバッファを作成
    while (true) {
        readUntilPipe(output_buf, sizeof(output_buf)); // '|'が受け取られるまでデータを読み込み
        // おむに停止する
        if (strncmp(output_buf, "cross", 5) == 0) 
        {
            pwm[0] = 0;
            pwm[1] = 0;
            pwm[2] = 0;
        }
        // 回収停止
        else if (strncmp(output_buf, "release", 7) == 0)
        {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 0;
            penguin.pwm[2] = 0;
            penguin.pwm[3] = 0;
        }
        else if (strncmp(output_buf, "R1OFF", 5) == 0)
        {
            jump = 0;
        }
        else if (strncmp(output_buf, "L1OFF", 5) == 0)
        {
            jump = 0;
        }
        // おむに制御
        else if (strncmp(output_buf, "L3_x:", 5) == 0) // "L3_x:"という文字列で始まるか確認
        {
            char *dataPointer = output_buf + 5;
            L_x = atoi(dataPointer);
        }
        else if (strncmp(output_buf, "L3_y:", 5) == 0) // "L3_y:"という文字列で始まるか確認
        {
            char *dataPointer = output_buf + 5;
            L_y = atoi(dataPointer);
        }
        else if (strncmp(output_buf, "R3_x:", 5) == 0) // "R3_x:"という文字列で始まるか確認
        {
            char *dataPointer = output_buf + 5;
            R_x = atoi(dataPointer);
        }
        // 回収up
        else if (strncmp(output_buf, "circle", 6) == 0)
        {
            penguin.pwm[0] = 2000;
            penguin.pwm[1] = 0;
            penguin.pwm[2] = 0;
            penguin.pwm[3] = 0;
        }
        else if (strncmp(output_buf, "triangle", 8) == 0)
        {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 2000;
            penguin.pwm[2] = 0;
            penguin.pwm[3] = 0;
        }
        else if (strncmp(output_buf, "square", 6) == 0)
        {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 0;
            penguin.pwm[2] = 2000;
            penguin.pwm[3] = 0;
        }
        // 回収down
        else if (strncmp(output_buf, "right", 5) == 0)
        {
            penguin.pwm[0] = -2000;
            penguin.pwm[1] = 0;
            penguin.pwm[2] = 0;
            penguin.pwm[3] = 0;
        }
        else if (strncmp(output_buf, "up", 8) == 0)
        {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = -2000;
            penguin.pwm[2] = 0;
            penguin.pwm[3] = 0;
        }
        else if (strncmp(output_buf, "left", 4) == 0)
        {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 0;
            penguin.pwm[2] = -2000;
            penguin.pwm[3] = 0;
        }
        // ジャンプ機構
        else if (strncmp(output_buf, "L1ON", 4) == 0)
        {
            jump = 10000;
        }
        else if (strncmp(output_buf, "R1ON", 4) == 0)
        {
            jump = -10000;
        }

        theta = atan2(L_y, L_x); // ラジアンで
        speed = hypot(L_x, L_y); // スティックの倒し具合を計算
    }
}
