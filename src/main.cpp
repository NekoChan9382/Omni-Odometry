#include <mbed.h>
#include <PID_new.hpp>
#include <array>
#include <cmath>

float duration_to_sec(const std::chrono::duration<float> &duration);

int main()
{
    CAN can(PA_11, PA_12, 1000000);
    BufferedSerial pc(USBTX, USBRX, 115200);
    CANMessage msg;
    PidGain gain_xy = {35.0, 2.0, 0.0};
    PidGain gain_theta = {15000.0, 2000.0, 0.0};
    std::array<Pid, 3> pid = {Pid({gain_xy, -12000, 12000}), Pid({gain_xy, -12000, 12000}), Pid({gain_theta, -12000, 12000})};
    DigitalIn sw(BUTTON1, PullUp);

    constexpr int ppr = 512;
    constexpr int enc_amount = 4;
    constexpr int wheel_radius = 25;
    constexpr int robot_size = 220;
    constexpr int motor_amount = 4;
    int enc_diff[enc_amount] = {0};
    int enc_diff_pre[enc_amount] = {0};
    float pos_mm[3] = {0};
    bool start = false;
    printf("start\n");
    int16_t enc_data[enc_amount] = {0};
    float goal_pos[3] = {0, 900, 1.57};
    float goal_current[3] = {0};
    float goal_time = 2.0;
    int16_t motor_output[motor_amount];

    while (true)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        if (sw.read() == 0 && start == false)
        {
            start = true;
            printf("go\n");
            for (int i = 0; i < 3; i++)
            {
                pid[i].reset();
            }
        }

        if (can.read(msg))
        {
            if (msg.id == 10)
            {

                for (int i = 0; i < enc_amount; i++)
                {
                    enc_data[i] = msg.data[2 * i + 1] << 8 | msg.data[2 * i];
                    // int test = enc_data[i] * 360 / (2.0 * ppr);
                    // printf("deg: %d\n", test);
                    // enc_diff[i] += enc_data[i] - enc_diff_pre[i];
                    // enc_diff_pre[i] = enc_data[i];
                }
            }
        }

        if (now - pre > 10ms)
        {
            float elapsed = duration_to_sec(now - pre);
            if (start)
            {
                float step = goal_time / elapsed;
                for (int i = 0; i < 3; i++)
                {
                    if (goal_current[i] < goal_pos[i])
                        goal_current[i] += goal_pos[i] / step;
                }
                // printf("goal_current: %f, %f, %f\n", goal_current[0], goal_current[1], goal_current[2]);
            }

            float wheel_move[enc_amount] = {0};
            for (int i = 0; i < enc_amount; ++i)
            {
                enc_diff[i] = enc_data[i] - enc_diff_pre[i];
                float enc_to_rad = (2.0 * M_PI) / (ppr * 2.0);
                wheel_move[i] = enc_diff[i] * enc_to_rad * wheel_radius;
                // printf("angle: %f\n", test);
                // printf("wheel_move[%d]: %f\n", i, wheel_move[i]);
                enc_diff_pre[i] = enc_data[i];
            }
            float pos_local[2] = {0};
            pos_local[0] += (wheel_move[1] + wheel_move[3] * -1) / 2;
            pos_local[1] += (wheel_move[0] * -1 + wheel_move[2]) / 2;
            pos_mm[2] += (wheel_move[0] + wheel_move[1] + wheel_move[2] + wheel_move[3]) / -4 / robot_size;
            float r = hypot(pos_local[0], pos_local[1]);
            float theta = atan2(pos_local[1], pos_local[0]);
            pos_mm[0] += r * cos(theta + pos_mm[2]);
            pos_mm[1] += r * sin(theta + pos_mm[2]);
            printf("pos_mm: %d, %d, %d\n", (int)pos_mm[0], (int)pos_mm[1], (int)(pos_mm[2] * 180 / M_PI));

            float output_power[3] = {0};
            for (int i = 0; i < 3; i++)
            {
                output_power[i] = pid[i].calc(goal_current[i], pos_mm[i], elapsed);
                // printf("output_power %d: %f\n", i, output_power[i]);
            }
            float r_output = hypot(output_power[0], output_power[1]);
            float theta_output = atan2(output_power[1], output_power[0]);
            // printf("r_output: %f, theta_output: %f\n", r_output, theta_output);
            // printf("output_power: %d, %d, %d\n", (int)output_power[0], (int)output_power[1], (int)output_power[2]);
            float motor_ang_base = M_PI / 2;
            float motor_offset = M_PI / 4;

            for (int i = 0; i < motor_amount; i++)
            {
                float goal_output = sin(theta_output - (motor_ang_base * i + motor_offset + pos_mm[2])) * r_output + output_power[2];
                motor_output[i] = (int)goal_output;
                // printf("goal_output %d: %f\n", i, goal_output);
            }
            printf("motor_output: %d, %d, %d, %d\n", motor_output[0], motor_output[1], motor_output[2], motor_output[3]);
            CANMessage msg_motor(2, (const uint8_t *)motor_output, sizeof(motor_output));
            can.write(msg_motor);

            pre = now;
        }
    }
}

float duration_to_sec(const std::chrono::duration<float> &duration)
{
    return duration.count();
}