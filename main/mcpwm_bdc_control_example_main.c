/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "string.h"
#include "math.h"

static const char *TAG = "motornode";

static const int RX_BUF_SIZE = 1024;

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           1

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
#define BDC1_ENCODER_PPR               5281.1
#define BDC2_ENCODER_PPR               1425.1
#define BDC_ENCODER_PCNT_HIGH_LIMIT    5000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -5000

#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
//#define Speed1_Command          5  // expected motor speed, in the pulses counted by the rotary encoder

#define TXD_PIN 1
#define RXD_PIN 3
static const float pi = 3.1415926536;
static float Speed1_Command = 0.0;
static float Speed2_Command = 0.0;

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    float encoder_ppr;
    int report_pulses_diff;
    int report_pulses;
} motor_control_context_t;


float ticks_to_angular_vel(int32_t ticks, float encoder_ppr, float dt)
{
    float angular_vel = 0.0;
    if(encoder_ppr > 0) {
        angular_vel = (ticks/dt)*((2*pi)/(encoder_ppr));
    }
    return angular_vel;
}

float ticks_to_angle(int32_t ticks, float encoder_ppr)
{
    float angle = 0.0;
    if(encoder_ppr > 0) {
        angle = (ticks)*((2*pi)/(encoder_ppr));
    }
    return angle;
}

float get_current_angle(motor_control_context_t *motor_ctx)
{
    int cur_pulse_count = 0;
    pcnt_unit_get_count(motor_ctx->pcnt_encoder, &cur_pulse_count);
    return ticks_to_angle(cur_pulse_count, motor_ctx->encoder_ppr);
}


static void pid_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;
    float ppr = ctx->encoder_ppr;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses_diff = real_pulses;
    float measured_speed = ticks_to_angular_vel(real_pulses, ppr, 0.01);
    

    // calculate the speed error
    float error = Speed1_Command - measured_speed;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    int offset = 0;
    if(new_speed < -0.01) {
        offset = 60;
        bdc_motor_forward(motor);
    }
    else if (new_speed > 0.01){
        offset = 60;
        bdc_motor_reverse(motor);
    } else {
        offset = 0;
    }

    uint32_t new_speed_uint = (uint32_t)(fabs(new_speed*5 + offset));
    if (new_speed_uint >= BDC_MCPWM_DUTY_TICK_MAX)
        new_speed_uint = BDC_MCPWM_DUTY_TICK_MAX-1;
    bdc_motor_set_speed(motor, new_speed_uint);
}

static void pid2_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses_diff = real_pulses;
    float ppr = ctx->encoder_ppr;
    float measured_speed = ticks_to_angular_vel(real_pulses, ppr, 0.01);

    // calculate the speed error
    float error = Speed2_Command - measured_speed;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    int offset = 0;
    if(new_speed < -0.01) {
        offset = 60;
        bdc_motor_forward(motor);
    }
    else if (new_speed > 0.01){
        offset = 60;
        bdc_motor_reverse(motor);
    } else {
        offset = 0;
    }

    uint32_t new_speed_uint = (uint32_t)(fabs(new_speed*2 + offset));
    if (new_speed_uint >= BDC_MCPWM_DUTY_TICK_MAX)
        new_speed_uint = BDC_MCPWM_DUTY_TICK_MAX-1;
    bdc_motor_set_speed(motor, new_speed_uint);
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

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    char* data = (char*) malloc(RX_BUF_SIZE + 1);
    while(1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        if (rxBytes > 5) {
            data[rxBytes] = 0;
            sscanf(data,"%f:%f",&Speed1_Command, &Speed2_Command);
        }
    }
    free(data);
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

void app_main(void)
{
    // Instal UART driver using an event queue here
    gpio_set_direction(BDC1_EN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDC1_EN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDC2_EN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(BDC2_EN_B, GPIO_MODE_OUTPUT);

    static motor_control_context_t motor1_ctrl_ctx = {
        .pcnt_encoder = NULL,
        .encoder_ppr = BDC1_ENCODER_PPR,
    };

    static motor_control_context_t motor2_ctrl_ctx = {
        .pcnt_encoder = NULL,
        .encoder_ppr = BDC2_ENCODER_PPR,
    };

    ESP_LOGI(TAG, "Create DC motors");
    bdc_motor_config_t motor1_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC1_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC2_MCPWM_GPIO_B,
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

    ESP_LOGI(TAG, "Create PID control blocks");
    pid_ctrl_parameter_t pid1_runtime_param = {
        .kp = 0.8,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = (-1) * (BDC_MCPWM_DUTY_TICK_MAX + 1),
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid1_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor1_ctrl_ctx.pid_ctrl = pid_ctrl;

    pid_ctrl_parameter_t pid2_runtime_param = {
        .kp = 0.5,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = (-1) * (BDC_MCPWM_DUTY_TICK_MAX + 1),
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid2_ctrl = NULL;
    pid_ctrl_config_t pid2_config = {
        .init_param = pid2_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid2_config, &pid2_ctrl));
    motor2_ctrl_ctx.pid_ctrl = pid2_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args1 = {
        .callback = pid_loop_cb,
        .arg = &motor1_ctrl_ctx,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer1 = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args1, &pid_loop_timer1));

    const esp_timer_create_args_t periodic_timer_args2 = {
        .callback = pid2_loop_cb,
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
    xTaskCreate(rx_task, "uart_rx_task", 2048*2, NULL, configMAX_PRIORITIES - 1, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
        printf("%f:%f:%f:%f\n",
            ticks_to_angular_vel(motor1_ctrl_ctx.report_pulses_diff, BDC1_ENCODER_PPR, 0.01),
            ticks_to_angular_vel(motor2_ctrl_ctx.report_pulses_diff, BDC2_ENCODER_PPR, 0.01),
            get_current_angle(&motor1_ctrl_ctx),
            get_current_angle(&motor2_ctrl_ctx));

        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information
#if SERIAL_STUDIO_DEBUG
        //printf("/*%d*/\r\n", motor1_ctrl_ctx.report_pulses_diff);
#endif
    }
}
