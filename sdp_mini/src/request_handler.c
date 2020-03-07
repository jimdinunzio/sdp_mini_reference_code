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

#include "common/common.h"
#include "net/interchip.h"
#include "drv/ctrl_bus_cmd.h"

#include "drv/battery.h"
#include "drv/motor.h"
#include "drv/distir.h"
#include "drv/homeir.h"
#include "drv/bump.h"
#include "drv/sonar.h"
#include "drv/drv_ctrlbus.h"
#include "drv/beep.h"
#include "drv/health_monitor.h"
#include "bump_monitor.h"
#include <math.h>

#ifndef M_PIf
#define M_PIf  3.14159265358979f
#endif

extern _u32 shutdownHeartbeatFrequency;
extern unsigned char slamware_config[];
extern size_t slamware_config_size;

static void update_shutdown_ticks(void)
{
    shutdownHeartbeatFrequency = getms();
}

static inline _u8 envSensorData2CtrlBusData(_u8 v) {
    // the internal order is main - left - right
    // while the control bus required order is left - main - right
    // so this function is responsible to swap left and main bit
    return (v & 0xfcu)     // erase lower 2 bits
        |  ((v & 1) << 1)  // rise main bit
        |  ((v & 2) >> 1); // drop left bit
}

#ifdef FEATURE_SET_V

static const float robot_radius_mm = 118.f;

static bool is_first_motor_data = true;
static int last_motor_l_speed = 0;
static int last_motor_r_speed = 0;

static void calculateMotorStatus(base_deadreckon_response_t* ans_pkt)
{
    memset(ans_pkt, 0, sizeof(*ans_pkt));
    
    float d_dist_l_mm_f = walkingmotor_delta_ldist_mm_f();
    float d_dist_r_mm_f = walkingmotor_delta_rdist_mm_f();
    
    if(is_first_motor_data)
    {
    is_first_motor_data = false;
    ans_pkt->base_dx_mm_q16 = 0;
    ans_pkt->base_dy_mm_q16 = 0;
    ans_pkt->base_dtheta_degree_q16 = 0;        
    }
    else
    {
        if(last_motor_l_speed < 0)
        {
            d_dist_l_mm_f = -d_dist_l_mm_f;
        }
        if(last_motor_r_speed < 0)
        {
            d_dist_r_mm_f = -d_dist_r_mm_f;
        }
        float d_yaw = (d_dist_r_mm_f - d_dist_l_mm_f) / 2.0f / robot_radius_mm;
        float displacement = (d_dist_l_mm_f + d_dist_r_mm_f) / 2.0f;
        
        float dx = cos(d_yaw)*displacement;
        float dy = sin(d_yaw)*displacement;
        
        ans_pkt->base_dx_mm_q16 = (_s32)(dx * (1<<16));
        ans_pkt->base_dy_mm_q16 = (_s32)(dy * (1<<16));
        ans_pkt->base_dtheta_degree_q16 = (_s32)(d_yaw / M_PIf * 180 * (1<<16));       
    }   
}
#endif

/*
 * ctrl bus用户报文处理函数
 */
static void on_request_slamcore_cb(infra_channel_desc_t * channel)
{
    slamcore_cb_cmd_t *request = (slamcore_cb_cmd_t *) _request_data;

    switch (request->cmd) {
    case SLAMWARECORECB_CMD_CONNECT_BASE:
        {
            base_connect_request_t *req_pkt = (base_connect_request_t *) request->payload;
            if (req_pkt->protocol_version != SLAMWARECORE_PROTOCOL_VERSION) {
                goto on_error;
            } else {
                base_connect_response_t ans_pkt;
                memcpy(&ans_pkt.model, &baseInfo.model, sizeof(ans_pkt.model));
                ans_pkt.firmware_version = baseInfo.firmwareVersion;
                ans_pkt.hardware_version = baseInfo.hardwareVersion;
                //memcpy(&ans_pkt.serial_number, &baseInfo.serialNumber, sizeof(ans_pkt.serial_number));
                ans_pkt.serial_number[0] = baseInfo.serialNumber[0];
                ans_pkt.serial_number[1] = baseInfo.serialNumber[1];
                ans_pkt.serial_number[2] = baseInfo.serialNumber[2];

                net_send_ans(channel, &ans_pkt, sizeof(base_connect_response_t));
                beep_beeper(6000, 300, 2);
            }
        }
        break;
    case SLAMWARECORECB_CMD_GET_BASE_CONF:
        {
            base_conf_response_t ans_pkt;                                               //  ^ z
            ans_pkt.base_type = CIRCLE;                                                 //  |          . y
            ans_pkt.base_radius_q8 = (_u32) (FP_Q8(180));                               //  |        .
            ans_pkt.base_motor_type = TWO_WHEEL;                                        //  |      .
            ans_pkt.base_sensor_num = 4;                                                //  |    .
            memset(ans_pkt.base_sensors, 0, sizeof(base_pos_t) * 8);                    //  |   .
            ans_pkt.base_bumper_num = 2;                                                //  | .
            memset(ans_pkt.base_bumpers, 0, sizeof(base_pos_t) * 8);                    //  |- - - - - - - > x   右手笛卡尔坐标系
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
            // base_sensors
            ans_pkt.base_sensors[0].y_to_center_mm_q8 = (_s32)(FP_Q8(0.0));
            ans_pkt.base_sensors[0].x_to_center_mm_q8 = (_s32)(FP_Q8(170.0));
            ans_pkt.base_sensors[0].z_to_center_mm_q8 = (_s32)(FP_Q8(60.0));
            ans_pkt.base_sensors[0].anti_clockwise_angle_to_center_degree_q8 = (_s32)(FP_Q8(0.0));
          
            ans_pkt.base_sensors[1].y_to_center_mm_q8 = (_s32)(FP_Q8(-120.0));
            ans_pkt.base_sensors[1].x_to_center_mm_q8 = (_s32)(FP_Q8(120.0));
            ans_pkt.base_sensors[1].z_to_center_mm_q8 = (_s32)(FP_Q8(60.0));
            ans_pkt.base_sensors[1].anti_clockwise_angle_to_center_degree_q8 = (_s32)(FP_Q8(315.0));
          
            ans_pkt.base_sensors[2].y_to_center_mm_q8 = (_s32)(FP_Q8(120.0));
            ans_pkt.base_sensors[2].x_to_center_mm_q8 = (_s32)(FP_Q8(120.0));
            ans_pkt.base_sensors[2].z_to_center_mm_q8 = (_s32)(FP_Q8(60.0));
            ans_pkt.base_sensors[2].anti_clockwise_angle_to_center_degree_q8 = (_s32)(FP_Q8(45.0));   
          
            ans_pkt.base_sensors[3].y_to_center_mm_q8 = (_s32)(FP_Q8(147.0));
            ans_pkt.base_sensors[3].x_to_center_mm_q8 = (_s32)(FP_Q8(85.0));
            ans_pkt.base_sensors[3].z_to_center_mm_q8 = (_s32)(FP_Q8(60.0));
            ans_pkt.base_sensors[3].anti_clockwise_angle_to_center_degree_q8 = (_s32)(FP_Q8(75.0));
#endif
            // base_bumpers
            ans_pkt.base_bumpers[0].x_to_center_mm_q8 = (_s32) (FP_Q8(120.0));
            ans_pkt.base_bumpers[0].y_to_center_mm_q8 = (_s32) (FP_Q8(120.0));
            ans_pkt.base_bumpers[0].z_to_center_mm_q8 = (_s32) (FP_Q8(50.0));
            ans_pkt.base_bumpers[0].anti_clockwise_angle_to_center_degree_q8 = (_s32) (FP_Q8(45.0));
            ans_pkt.base_bumpers[1].x_to_center_mm_q8 = (_s32) (FP_Q8(120.0));
            ans_pkt.base_bumpers[1].y_to_center_mm_q8 = (_s32) (FP_Q8(-120.0));
            ans_pkt.base_bumpers[1].z_to_center_mm_q8 = (_s32) (FP_Q8(50.0));
            ans_pkt.base_bumpers[1].anti_clockwise_angle_to_center_degree_q8 = (_s32) (FP_Q8(315.0));

            net_send_ans(channel, &ans_pkt, sizeof(base_conf_response_t));
        }
        break;
        
    case SLAMWARECORECB_CMD_GET_BINARY_CONF:
        {
            net_send_ans(channel, slamware_config, slamware_config_size);
        }
        break;
        
    case SLAMWARECORECB_CMD_GET_BASE_STATUS:
        {
            base_status_response_t ans_pkt;
            ans_pkt.power_status = PowerStatusNone;
            ans_pkt.power_status |= (charge_detect_getstatus() == ISCHARGE_CHRG) ? PowerStatusCharging : PowerStatusNone;
            ans_pkt.power_status |= get_home_charge_status() ? PowerStatusOnChargingDock : PowerStatusNone;
            ans_pkt.power_status |= get_dc_charge_status() ? PowerStatusDcConnected : PowerStatusNone;
            ans_pkt.battery_percentage = get_electricitypercentage();
            net_send_ans(channel, &ans_pkt, sizeof(base_status_response_t));
        }
        break;
    case SLAMWARECORECB_CMD_GET_BASE_MOTOR_DATA:
        {
            base_motor_status_response_t ans_pkt;
            memset(&ans_pkt, 0, sizeof(ans_pkt));
            ans_pkt.left_motor_cumulate_dist_mm = (_s32) (walkingmotor_cumulate_ldist_mm());
            ans_pkt.right_motor_cumulate_dist_mm = (_s32) (walkingmotor_cumulate_rdist_mm());
            ans_pkt.status_bitmap |= is_ontheground()?0:BASE_MOTOR_TRACTION_LOST;
            net_send_ans(channel, &ans_pkt, sizeof(base_motor_status_response_t));
        }
        break;
    case SLAMWARECORECB_CMD_GET_BASE_SENSOR_DATA:
        {
            base_sensor_data_response_t ans_pkt;
            memset(&ans_pkt, 0, sizeof(ans_pkt));
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
            /* Sonar sensor. */
            for (size_t ch = 0; ch < CONFIG_SONAR_CHANNEL_NUM; ch++) {
                ans_pkt.sensor_data_mm_q16[ch] = FP_Q16(sonar_get(ch));
            }
#if USE_FRONT_IR_SENSORS
            ans_pkt.sensor_data_mm_q16[8] = (_u32)((float)FP_Q16(homeir_getmaindata()));
            ans_pkt.sensor_data_mm_q16[9] = (_u32)((float)FP_Q16(homeir_getleftdata()));
            ans_pkt.sensor_data_mm_q16[10] = (_u32)((float)FP_Q16(homeir_getrightdata()));
#endif
#endif
            net_send_ans(channel, &ans_pkt, sizeof(base_sensor_data_response_t));
        }
        break;
    case SLAMWARECORECB_CMD_GET_BASE_BUMPER_DATA:
        {
            base_bumper_data_response_t ans_pkt;
            ans_pkt.bumper_data = get_bump_bitmap();
            ans_pkt.bumper_data |= (get_distir_value()->bottomSensorBitmap << 2);
            net_send_ans(channel, &ans_pkt, sizeof(base_bumper_data_response_t));
        }
        break;
    case SLAMWARECORECB_CMD_GET_AUTO_HOME_DATA:
        {
            base_auto_home_data_request_t* req = (base_auto_home_data_request_t*)request->payload;
            if (req->data_type == SLAMWARECORECB_AUTO_HOME_GET_BEACON) {
                base_auto_home_beacon_data_response_t ans_pkt;
                
                ans_pkt.beacon_count = 3;
                ans_pkt.probe_count = 3;
                ans_pkt.probe_data[0] = envSensorData2CtrlBusData(homeir_getleftdata());
                ans_pkt.probe_data[1] = envSensorData2CtrlBusData(homeir_getmaindata());
                ans_pkt.probe_data[2] = envSensorData2CtrlBusData(homeir_getrightdata());

                net_send_ans(channel, &ans_pkt, 2 + ans_pkt.probe_count);
            } else {
                net_send_errorcode(channel, PKT_ERRORCODE_NOT_SUPPORT);
            }
        }
        break;
    case SLAMWARECORECB_CMD_SET_BASE_MOTOR:
        {
            base_set_motor_request_t *ans_pkt = (base_set_motor_request_t *) request->payload;
            if (!bumpermonitor_filter_motorcmd(ans_pkt->motor_speed_mm[0], ans_pkt->motor_speed_mm[1])) {
                set_walkingmotor_speed(ans_pkt->motor_speed_mm[0], ans_pkt->motor_speed_mm[1]);
#ifdef FEATURE_SET_V
                if(ans_pkt->motor_speed_mm[0] != 0)
                {
                    last_motor_l_speed = ans_pkt->motor_speed_mm[0];
                }
                if(ans_pkt->motor_speed_mm[1] != 0)
                {
                    last_motor_r_speed = ans_pkt->motor_speed_mm[1];
                }
#endif
            }
            net_send_ans(channel, NULL, 0);
        }
        break;
#ifdef FEATURE_SET_V
    case SLAMWARECORECB_CMD_SET_V_AND_GET_DEADRECKON:
        {
            base_deadreckon_response_t ans_pkt;
            calculateMotorStatus(&ans_pkt);
          
            base_set_velocity_request_t *req = (base_set_velocity_request_t*)request->payload;
            float speed_l_mm = (float)req->velocity_x_q16 * 1000.0 / (1 << 16);
            float speed_r_mm = speed_l_mm;
            float line_speed_mm = (float)req->angular_velocity_q16 / (1 << 16) * robot_radius_mm;
            speed_l_mm -= line_speed_mm;
            speed_r_mm += line_speed_mm;
          
            set_walkingmotor_speed((int32_t)bumpermonitor_clamp_motorcmd(speed_l_mm)
                                , (int32_t)bumpermonitor_clamp_motorcmd(speed_r_mm));
            if(speed_l_mm != 0)
            {
                last_motor_l_speed = (int)speed_l_mm;
            }
            if(speed_r_mm != 0)
            {
                last_motor_r_speed = (int)speed_r_mm;
            }
            ans_pkt.status_bitmap |= (is_ontheground()?0:BASE_MOTOR_TRACTION_LOST);
            net_send_ans(channel, &ans_pkt, sizeof(base_deadreckon_response_t));
        }
        break;
#endif
    case SLAMWARECORECB_CMD_POLL_BASE_CMD:
        {
            if (is_drv_ctrlbus_ncmd()) {
                base_cmd_t cmd;
                cmd.base_cmd = get_drv_ctrlbus_ncmd();
                net_send_ans(channel, &cmd, sizeof(cmd));
            } else {
                net_send_ans(channel, NULL, 0);
            }
        }
        break;
    case SLAMWARECORECB_CMD_POLL_BASE_ANS_CMD:
        {
            net_send_ans(channel, NULL, 0);
            clear_drv_ctrlbus_ncmd();
        }
        break;
    case SLAMWARECORECB_CMD_SEND_EVENT:
        {
            slamcore_event_t *event = (slamcore_event_t *) request->payload;
            switch (event->slamcore_event) {
            case SLAMWARECORECB_EVENT_SYSTEM_UP_OK:
                {
                }
                break;
            case SLAMWARECORECB_EVENT_FIRMWARE_UPDATE:
                {
                }
                break;
            case SLAMWARECORECB_EVENT_CORE_DISCONNECT:
                {
                }
              	break;
            case SLAMWARECORECB_EVENT_START_SWEEP:
			 	{
                    // Do sweep action, like starting brush and blower.
				}
                break;
            case SLAMWARECORECB_EVENT_END_SWEEP: 
				{
                    // Stop sweep action.
              	}
				break;
            }
            net_send_ans(channel, NULL, 0);
        }
        break;
    case SLAMWARECORECB_CMD_HEALTH_MGMT:
        {
            health_mgmt_cmd_t *health_request = (health_mgmt_cmd_t *)request->payload;
            
            switch(health_request->cmd) {
            case SLAMWARECORECB_CMD_HEALTH_GET_HEALTH:
                {
                    get_health_response_t ans_pkt;

                    health_monitor_get_status(&ans_pkt.health_flag, &ans_pkt.error_count);
                    net_send_ans(channel, &ans_pkt, sizeof(get_health_response_t));
                }
                break;
            case SLAMWARECORECB_CMD_HEALTH_GET_ERROR:
                {
                    health_error_request_t *req_pkt = (health_error_request_t *)health_request->payload;
                    health_error_response_t ans_pkt;

                    if (health_monitor_get_desc(req_pkt->error_id) != NULL) {
                        ans_pkt.error_code = health_monitor_get_error(req_pkt->error_id);
                        strcpy((char *)ans_pkt.error_msg, health_monitor_get_desc(req_pkt->error_id));
                        net_send_ans(channel, &ans_pkt, sizeof(health_error_response_t));
                    } else {
                        net_send_errorcode(channel, PKT_ERRORCODE_OPERATION_FAIL);
                    }
                }
                break;
            case SLAMWARECORECB_CMD_HEALTH_CLR_ERROR:
                {
                    health_clear_error_request_t *req_pkt = (health_clear_error_request_t *)health_request->payload;

                    if (health_monitor_clear(req_pkt->error_code)) {
                        net_send_ans(channel, NULL, 0);
                    } else {
                        net_send_errorcode(channel, PKT_ERRORCODE_OPERATION_FAIL);
                    }
                }
                break;
            default:
                {
                    net_send_errorcode(channel, PKT_ERRORCODE_OPERATION_FAIL);
                }
            }
        }
        break;
    default:
        goto on_error;
    }
    return;
  on_error:
    net_send_errorcode(channel, PKT_ERRORCODE_NOT_SUPPORT);
}
/*
 * 响应interchip协议处理函数
 * 主要是处理ctrl bus用户报文
 */
void on_host_request(infra_channel_desc_t * channel)
{
    switch (net_get_request_cmd(channel)) {
    case CMD_CODE_SLAMWARECORE_CTRL_BUS:
        update_shutdown_ticks();
        on_request_slamcore_cb(channel);
        break;
    default:
        net_send_errorcode(channel, PKT_ERRORCODE_NOT_SUPPORT);
    }
}