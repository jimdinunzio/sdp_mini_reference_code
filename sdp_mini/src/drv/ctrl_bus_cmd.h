/*
 * SlamTec Base Ref Design
 * Copyright 2009 - 2017 RoboPeak
 * Copyright 2013 - 2017 Shanghai SlamTec Co., Ltd.
 * http://www.slamtec.com
 * All rights reserved.
 */
/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once


#if defined(_WIN32) || defined(__ICCARM__) || defined(__CC_ARM)
#pragma pack(1)
#endif

#define SLAMWARECORE_PROTOCOL_VERSION              (0x1)

// EXTEND MODE Command for slamware core ctrl bus
#define CMD_CODE_SLAMWARECORE_CTRL_BUS             (0xF8)

#define SLAMWARECORECB_CMD_CONNECT_BASE            (0x10)
#define SLAMWARECORECB_CMD_GET_BASE_CONF           (0x20)
#define SLAMWARECORECB_CMD_GET_BINARY_CONF         (0x21)
#define SLAMWARECORECB_CMD_GET_BASE_STATUS         (0x30)
#define SLAMWARECORECB_CMD_GET_BASE_MOTOR_DATA     (0x31)
#define SLAMWARECORECB_CMD_GET_BASE_SENSOR_DATA    (0x32)
#define SLAMWARECORECB_CMD_GET_BASE_BUMPER_DATA    (0x33)
#define SLAMWARECORECB_CMD_SET_BASE_MOTOR          (0x40)
#define SLAMWARECORECB_CMD_SET_V_AND_GET_DEADRECKON (0x41)
#define SLAMWARECORECB_CMD_POLL_BASE_CMD           (0x50)
#    define SLAMWARECORECB_BASE_CMD_GET_INFO       (0x51)
#    define SLAMWARECORECB_BASE_CMD_RESET_WIFI     (0x52)
#    define SLAMWARECORECB_BASE_CMD_START_SWEEP    (0x80)
#    define SLAMWARECORECB_BASE_CMD_STOP_SWEEP     (0x81)
#    define SLAMWARECORECB_BASE_CMD_SPOT_SWEEP     (0x82)
#define SLAMWARECORECB_CMD_POLL_BASE_ANS_CMD       (0x5f)
#define SLAMWARECORECB_CMD_SEND_EVENT              (0x60)
#    define SLAMWARECORECB_EVENT_LIDAR_CONN_FAIL   (0x61)
#    define SLAMWARECORECB_EVENT_LIDAR_RAMPUP_FAIL (0x62)
#    define SLAMWARECORECB_EVENT_SYSTEM_UP_OK      (0x63)
#    define SLAMWARECORECB_EVENT_FIRMWARE_UPDATE   (0x64)
#    define SLAMWARECORECB_EVENT_CORE_DISCONNECT   (0x65)
#    define SLAMWARECORECB_EVENT_FIRMWARE_UPDATE_OK (0x66)
#    define SLAMWARECORECB_EVENT_START_SWEEP       (0x80)
#    define SLAMWARECORECB_EVENT_END_SWEEP         (0x81)

// EXTEND MODE Command for base & UI
#define CMD_CODE_BASE_CMD             (0xF9)

// Protocol Packages
typedef struct _base_cb_cmd_t
{
    _u8 cmd;
    _u8 __reserved__;
} __attribute__((packed)) base_cb_cmd_t;


#define BASE_DEVMGMT_ENTER_DFU      1
#define BASE_DEVMGMT_REBOOT         2

// DFU Command
#define CMD_CODE_DFU_CLASS       0x20

typedef struct _slamcore_cb_cmd_t       //协议报文定义
{
    _u8 cmd;
    _u8 payload[1];
} __attribute__((packed)) slamcore_cb_cmd_t;

typedef struct _base_connect_request
{
    _u8  protocol_version;
} __attribute__((packed)) base_connect_request_t;

typedef struct _base_connect_response
{
    _u8  model[12];
    _u16 firmware_version;
    _u16 hardware_version;
    _u32 serial_number[3];
} __attribute__((packed)) base_connect_response_t;

enum _base_type {
    CIRCLE = 0,
    SQUARE = 1,
};

enum _base_motor_type {
    TWO_WHEEL = 0,
};

enum _base_motor_status {
    BASE_MOTOR_STATUS_STALL   = (0x1<<1),
    BASE_MOTOR_TRACTION_LOST  = (0x1<<2),
};

typedef struct _base_sensor_pos
{
    _s32  x_to_center_mm_q8;
    _s32  y_to_center_mm_q8;
    _s32  z_to_center_mm_q8;
    _u32  anti_clockwise_angle_to_center_degree_q8;
} __attribute__((packed)) base_pos_t;

typedef struct _base_conf_response
{
    _u8        base_type;
    _u32       base_radius_q8;
    _u8        base_motor_type;
    _u8        base_sensor_num;
    base_pos_t base_sensors[8];
    _u8        base_bumper_num;
    base_pos_t base_bumpers[8];
} __attribute__((packed)) base_conf_response_t;

typedef struct _base_status_response
{
    _u8 battery_percentage;
    _u8 power_status;
} __attribute__((packed)) base_status_response_t;

enum _base_power_status {
    PowerStatusNone = 0,
    PowerStatusCharging = 1,
    PowerStatusDcConnected = 2,
    PowerStatusOnChargingDock = 4
};

typedef struct _base_motor_status_response
{
    _s32 left_motor_cumulate_dist_mm;
    _s32 right_motor_cumulate_dist_mm;
    _u8  status_bitmap;
} __attribute__((packed)) base_motor_status_response_t;

typedef struct _base_deadreckon_response
{
    _s32 base_dx_mm_q16;
    _s32 base_dy_mm_q16;
    _s32 base_dtheta_degree_q16;
    _u8  status_bitmap;
} __attribute__((packed)) base_deadreckon_response_t;

typedef struct _base_sensor_data_response
{
    _u32  sensor_data_mm_q16[16];
} __attribute__((packed)) base_sensor_data_response_t;

typedef struct _base_bumper_data_response
{
    _u8  bumper_data;           //每一位代表一个bump传感器的状态，最多支持8个
} __attribute__((packed)) base_bumper_data_response_t;

typedef struct _base_set_motor_request
{
    _s32 motor_speed_mm[4];
} __attribute__((packed)) base_set_motor_request_t;

typedef struct _base_set_velocity_request
{
    _s32 velocity_x_q16;
    _s32 velocity_y_q16;
    _s32 angular_velocity_q16;
} __attribute__((packed)) base_set_velocity_request_t;

typedef struct _base_cmd
{
    _u8 base_cmd;
    _u8 base_payload[1];
} __attribute__((packed)) base_cmd_t;

typedef struct _slamcore_event_t
{
    _u8 slamcore_event;
} __attribute__((packed)) slamcore_event_t;

typedef struct _base_devmgmt_request
{
    _u8  req_cmd;
    _u32 __reserved__;
} __attribute__((packed)) base_devmgmt_request_t;

#if defined(_WIN32) || defined(__ICCARM__) || defined(__CC_ARM)
#pragma pack()
#endif

