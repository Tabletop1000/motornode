/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bdc_motor.h"

static const char *TAG = "motornode";

#define RX_BUF_SIZE                   1024
#define RX_LINE_MAX                   64

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC1_MCPWM_GPIO_A              16
#define BDC1_MCPWM_GPIO_B              17
#define BDC2_MCPWM_GPIO_A              33
#define BDC2_MCPWM_GPIO_B              32

#define BDC1_EN_A                      26
#define BDC1_EN_B                      27
#define BDC2_EN_A                      22
#define BDC2_EN_B                      23

#define BDC1_ENCODER_GPIO_A            18
#define BDC1_ENCODER_GPIO_B            19
#define BDC2_ENCODER_GPIO_A            4
#define BDC2_ENCODER_GPIO_B            5
#define BDC1_ENCODER_PPR               3895.9f
#define BDC2_ENCODER_PPR               3895.9f
#define BDC_ENCODER_PCNT_HIGH_LIMIT    5000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -5000

#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
#define BDC_PID_LOOP_PERIOD_S         (BDC_PID_LOOP_PERIOD_MS / 1000.0f)

// Duty (in ticks) added on top of the PID output to get past the motor's static friction.
// Only applied when the PID is pushing in the same direction as the command, so the drive
// never flips between +offset and -offset around zero (that flip caused a limit cycle).
#define DEADBAND_OFFSET_TICKS         60

// Commands with magnitude below this (rad/s) are treated as "stop": PID is reset and the
// bridge is put in the idle state instead of servoing around zero.
#define CMD_ZERO_EPS                  1e-3f

// Don't drive the bridge against the direction of rotation while the wheel is still turning
// faster than this (rad/s). Avoids plug-braking current spikes; brake passively instead.
#define REVERSE_LOCKOUT_SPEED         1.0f

// How long the driver enable pins are held low on an "EN" reset command.
#define DRIVER_RESET_LOW_MS           20

#define TXD_PIN 1
#define RXD_PIN 3

#define CMD_TIMEOUT_US (500 * 1000)

static const float pi = 3.1415926536f;

// Wheel speed commands in rad/s, written by the UART task and read by the PID timer callbacks.
static volatile float speed_cmd[2] = {0.0f, 0.0f};
static portMUX_TYPE speed_cmd_lock = portMUX_INITIALIZER_UNLOCKED;

// Incremental (velocity form) PID, same formula as espressif/pid_ctrl but with a reset.
typedef struct {
    float kp;
    float ki;
    float kd;
    float out_max;   // output is clamped to [-out_max, out_max]
    float err1;      // e(k-1)
    float err2;      // e(k-2)
    float out;       // u(k-1)
} speed_pid_t;

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    speed_pid_t pid;
    float encoder_ppr;
    float output_scale;     // duty ticks per unit of PID output
    int cmd_index;          // index into speed_cmd[]
    int last_pulse_count;
    volatile int report_pulses_diff;
} motor_control_context_t;

static void speed_pid_reset(speed_pid_t *pid)
{
    pid->err1 = 0.0f;
    pid->err2 = 0.0f;
    pid->out = 0.0f;
}

static float speed_pid_step(speed_pid_t *pid, float error)
{
    float out = pid->out +
                (error - pid->err1) * pid->kp +
                (error - 2.0f * pid->err1 + pid->err2) * pid->kd +
                error * pid->ki;
    // Clamping the accumulated output is the anti-windup for the incremental form.
    if (out > pid->out_max) {
        out = pid->out_max;
    } else if (out < -pid->out_max) {
        out = -pid->out_max;
    }
    pid->err2 = pid->err1;
    pid->err1 = error;
    pid->out = out;
    return out;
}

static float get_speed_command(int index)
{
    taskENTER_CRITICAL(&speed_cmd_lock);
    float cmd = speed_cmd[index];
    taskEXIT_CRITICAL(&speed_cmd_lock);
    return cmd;
}

static void set_speed_commands(float cmd1, float cmd2)
{
    taskENTER_CRITICAL(&speed_cmd_lock);
    speed_cmd[0] = cmd1;
    speed_cmd[1] = cmd2;
    taskEXIT_CRITICAL(&speed_cmd_lock);
}

float ticks_to_angular_vel(int32_t ticks, float encoder_ppr, float dt)
{
    float angular_vel = 0.0f;
    if (encoder_ppr > 0) {
        angular_vel = (ticks / dt) * ((2 * pi) / (encoder_ppr));
    }
    return angular_vel;
}

float ticks_to_angle(int32_t ticks, float encoder_ppr)
{
    float angle = 0.0f;
    if (encoder_ppr > 0) {
        angle = (ticks) * ((2 * pi) / (encoder_ppr));
    }
    return angle;
}

float get_current_angle(motor_control_context_t *motor_ctx)
{
    int cur_pulse_count = 0;
    pcnt_unit_get_count(motor_ctx->pcnt_encoder, &cur_pulse_count);
    return ticks_to_angle(cur_pulse_count, motor_ctx->encoder_ppr);
}

// Zero duty with both half-bridges held low. On a BTS7960/IBT-2 style driver that means both
// low-side switches are on, i.e. the motor is shorted (resistive braking, no reverse drive).
static void motor_idle(bdc_motor_handle_t motor)
{
    bdc_motor_set_speed(motor, 0);
    bdc_motor_coast(motor);
}

static void pid_loop_cb(void *args)
{
    motor_control_context_t *ctx = (motor_control_context_t *)args;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(ctx->pcnt_encoder, &cur_pulse_count);
    int real_pulses = cur_pulse_count - ctx->last_pulse_count;
    ctx->last_pulse_count = cur_pulse_count;
    ctx->report_pulses_diff = real_pulses;
    float measured_speed = ticks_to_angular_vel(real_pulses, ctx->encoder_ppr, BDC_PID_LOOP_PERIOD_S);

    float cmd = get_speed_command(ctx->cmd_index);

    // Stop command: don't servo around zero (encoder quantisation + deadband offset made that
    // oscillate). Reset the PID so the next command starts from a clean state.
    if (fabsf(cmd) < CMD_ZERO_EPS) {
        speed_pid_reset(&ctx->pid);
        motor_idle(ctx->motor);
        return;
    }

    float out = speed_pid_step(&ctx->pid, cmd - measured_speed);

    // Positive PID output produces positive measured speed (via bdc_motor_reverse, per wiring).
    // Refuse to drive against the current rotation until the wheel has slowed down.
    if ((out > 0 && measured_speed < -REVERSE_LOCKOUT_SPEED) ||
        (out < 0 && measured_speed > REVERSE_LOCKOUT_SPEED)) {
        ctx->pid.out = 0.0f;
        motor_idle(ctx->motor);
        return;
    }

    uint32_t duty = (uint32_t)(fabsf(out) * ctx->output_scale);
    if (out * cmd > 0) {
        duty += DEADBAND_OFFSET_TICKS;
    }
    if (duty >= BDC_MCPWM_DUTY_TICK_MAX) {
        duty = BDC_MCPWM_DUTY_TICK_MAX - 1;
    }

    if (out > 0) {
        bdc_motor_reverse(ctx->motor);
    } else if (out < 0) {
        bdc_motor_forward(ctx->motor);
    } else {
        motor_idle(ctx->motor);
        return;
    }
    bdc_motor_set_speed(ctx->motor, duty);
}

void enable_motor(void)
{
    gpio_set_level(BDC1_EN_A, 1U);
    gpio_set_level(BDC1_EN_B, 1U);
    gpio_set_level(BDC2_EN_A, 1U);
    gpio_set_level(BDC2_EN_B, 1U);
}

void disable_motor(void)
{
    gpio_set_level(BDC1_EN_A, 0U);
    gpio_set_level(BDC1_EN_B, 0U);
    gpio_set_level(BDC2_EN_A, 0U);
    gpio_set_level(BDC2_EN_B, 0U);
}

void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Handle one complete message. Returns true if it was a valid speed command.
static bool process_line(const char *line)
{
    // "EN": pulse the driver enable pins low to clear a latched driver fault.
    if (strcmp(line, "EN") == 0) {
        disable_motor();
        vTaskDelay(pdMS_TO_TICKS(DRIVER_RESET_LOW_MS));
        enable_motor();
        return false;
    }

    float cmd1, cmd2;
    if (sscanf(line, "%f:%f", &cmd1, &cmd2) == 2 && isfinite(cmd1) && isfinite(cmd2)) {
        set_speed_commands(cmd1, cmd2);
        return true;
    }
    return false;
}

static void rx_task(void *arg)
{
    uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE);
    char line[RX_LINE_MAX];
    size_t line_len = 0;
    bool line_overflow = false;
    int64_t last_cmd_us = 0;

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, pdMS_TO_TICKS(10));

        for (int i = 0; i < rxBytes; i++) {
            char c = (char)data[i];
            if (c == '\n' || c == '\r') {
                if (line_len > 0 && !line_overflow) {
                    line[line_len] = '\0';
                    if (process_line(line)) {
                        last_cmd_us = esp_timer_get_time();
                    }
                }
                line_len = 0;
                line_overflow = false;
            } else if (line_len < RX_LINE_MAX - 1) {
                line[line_len++] = c;
            } else {
                line_overflow = true;   // discard the rest of an over-long line
            }
        }

        // A 10ms gap with no data also ends a message, in case the sender doesn't send newlines.
        if (rxBytes == 0 && line_len > 0) {
            if (!line_overflow) {
                line[line_len] = '\0';
                if (process_line(line)) {
                    last_cmd_us = esp_timer_get_time();
                }
            }
            line_len = 0;
            line_overflow = false;
        }

        if (esp_timer_get_time() - last_cmd_us > CMD_TIMEOUT_US) {
            set_speed_commands(0.0f, 0.0f);
        }
    }
    free(data);
}

void app_main(void)
{
    gpio_set_direction(BDC1_EN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDC1_EN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDC2_EN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDC2_EN_B, GPIO_MODE_OUTPUT);

    // PID output limits are set so that out_max * scale + offset == max duty: anything beyond
    // that can't change the duty and would only wind up.
    static motor_control_context_t motor1_ctrl_ctx = {
        .encoder_ppr = BDC1_ENCODER_PPR,
        .output_scale = 2.0f,
        .cmd_index = 0,
        .pid = {
            .kp = 0.8f,
            .ki = 0.4f,
            .kd = 0.2f,
            .out_max = (BDC_MCPWM_DUTY_TICK_MAX - 1 - DEADBAND_OFFSET_TICKS) / 2.0f,
        },
    };

    static motor_control_context_t motor2_ctrl_ctx = {
        .encoder_ppr = BDC2_ENCODER_PPR,
        .output_scale = 2.0f,
        .cmd_index = 1,
        .pid = {
            .kp = 0.8f,
            .ki = 0.4f,
            .kd = 0.2f,
            .out_max = (BDC_MCPWM_DUTY_TICK_MAX - 1 - DEADBAND_OFFSET_TICKS) / 2.0f,
        },
    };

    ESP_LOGI(TAG, "Create DC motors");
    bdc_motor_config_t motor1_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC1_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC1_MCPWM_GPIO_B,
    };
    bdc_motor_config_t motor2_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC2_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC2_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm1_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm2_config = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor1 = NULL;
    bdc_motor_handle_t motor2 = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor1_config, &mcpwm1_config, &motor1));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor2_config, &mcpwm2_config, &motor2));
    motor1_ctrl_ctx.motor = motor1;
    motor2_ctrl_ctx.motor = motor2;

    ESP_LOGI(TAG, "Init pcnt drivers to decode rotary signals");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt1_unit = NULL;
    pcnt_unit_handle_t pcnt2_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt1_unit));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt2_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt1_unit, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt2_unit, &filter_config));
    pcnt_chan_config_t chan1_a_config = {
        .edge_gpio_num = BDC1_ENCODER_GPIO_A,
        .level_gpio_num = BDC1_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt1_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt1_unit, &chan1_a_config, &pcnt1_chan_a));
    pcnt_chan_config_t chan1_b_config = {
        .edge_gpio_num = BDC1_ENCODER_GPIO_B,
        .level_gpio_num = BDC1_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt1_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt1_unit, &chan1_b_config, &pcnt1_chan_b));

    pcnt_chan_config_t chan2_a_config = {
        .edge_gpio_num = BDC2_ENCODER_GPIO_A,
        .level_gpio_num = BDC2_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt2_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt2_unit, &chan2_a_config, &pcnt2_chan_a));
    pcnt_chan_config_t chan2_b_config = {
        .edge_gpio_num = BDC2_ENCODER_GPIO_B,
        .level_gpio_num = BDC2_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt2_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt2_unit, &chan2_b_config, &pcnt2_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt1_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt1_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt1_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt1_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt1_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt1_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt1_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt1_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt1_unit));
    motor1_ctrl_ctx.pcnt_encoder = pcnt1_unit;

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt2_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt2_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt2_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt2_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt2_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt2_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt2_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt2_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt2_unit));
    motor2_ctrl_ctx.pcnt_encoder = pcnt2_unit;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args1 = {
        .callback = pid_loop_cb,
        .arg = &motor1_ctrl_ctx,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer1 = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args1, &pid_loop_timer1));

    const esp_timer_create_args_t periodic_timer_args2 = {
        .callback = pid_loop_cb,
        .arg = &motor2_ctrl_ctx,
        .name = "pid_loop2"
    };
    esp_timer_handle_t pid_loop_timer2 = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args2, &pid_loop_timer2));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));
    ESP_ERROR_CHECK(bdc_motor_enable(motor2));

    enable_motor();

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer1, BDC_PID_LOOP_PERIOD_MS * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer2, BDC_PID_LOOP_PERIOD_MS * 1000));
    init_uart();
    xTaskCreate(rx_task, "uart_rx_task", 2048 * 2, NULL, configMAX_PRIORITIES - 1, NULL);

    // Output format is parsed by the ROS hardware interface: "vel1:vel2:angle1:angle2"
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
        printf("%f:%f:%f:%f\n",
               ticks_to_angular_vel(motor1_ctrl_ctx.report_pulses_diff, BDC1_ENCODER_PPR, BDC_PID_LOOP_PERIOD_S),
               ticks_to_angular_vel(motor2_ctrl_ctx.report_pulses_diff, BDC2_ENCODER_PPR, BDC_PID_LOOP_PERIOD_S),
               get_current_angle(&motor1_ctrl_ctx),
               get_current_angle(&motor2_ctrl_ctx));
    }
}
