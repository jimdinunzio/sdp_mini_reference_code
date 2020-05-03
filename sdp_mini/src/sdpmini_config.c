/*
* This file is automatically generated by RoboStudio Config Tool Plugin
* The encoded config is
{
   "base" : {
      "hardware_version" : 3,
      "manufacture_id" : 255,
      "manufacture_name" : "Slamtec",
      "model_id" : 56593,
      "model_name" : "Slamware SDP Mini",
      "robot_size" : 0.370,
      "sensors" : [
         {
            "id" : 0,
            "installation_pose" : {
               "x" : 0.1270,
               "y" : 0.1050,
               "yaw" : 0.698130,
               "z" : -0.050
            },
            "type" : "bumper"
         },
         {
            "id" : 1,
            "installation_pose" : {
               "x" : 0.1270,
               "y" : -0.1050,
               "yaw" : 5.585050,
               "z" : -0.050
            },
            "type" : "bumper"
         },
         {
            "id" : 2,
            "installation_pose" : {
               "x" : 0.1650,
               "z" : -0.050
            },
            "type" : "bumper"
         },
         {
            "id" : 4,
            "installation_pose" : {
               "x" : 0.08500000000000001,
               "y" : 0.1050,
               "yaw" : 0.722222220,
               "z" : 0.050
            },
            "max_distance" : 4.50,
            "min_distance" : 0.01999999955296516,
            "type" : "sonar"
         },
         {
            "id" : 5,
            "installation_pose" : {
               "x" : 0.08500000000000001,
               "y" : -0.1050,
               "yaw" : 5.560,
               "z" : 0.050
            },
            "max_distance" : 4.50,
            "min_distance" : 0.01999999955296516,
            "type" : "sonar"
         },
         {
            "id" : 6,
            "installation_pose" : {
               "x" : 0.1250,
               "z" : 0.050
            },
            "max_distance" : 4.50,
            "min_distance" : 0.01999999955296516,
            "type" : "sonar"
         }
      ],
      "slamware_core_pose" : {
         "head_up" : true,
         "yaw" : 3.141592741012573
      },
      "wheel_span" : 0.27650
   },
   "docking" : {
      "backward_docking" : true
   },
   "features" : {
      "has_ir_docking_tower" : false
   },
   "lidar" : {
      "installation_pose" : {
         "y" : 0.07000000000000001
      },
      "reverse_installation" : false
   },
   "motion_plan" : {
      "side_margin" : 0.07500000298023224
   }
}

* Please DO NOT modify this by hand
*/

#include <stdint.h>
#include <stdlib.h>

const unsigned char slamware_config[] = {
0x50,0x05,0x00,0x50,0x09,0x05,0x1a,0x03,0x00,0x00,0x00,0x01,0x1a,0xff,0x00,0x00,
0x00,0x02,0x30,0x07,0x53,0x6c,0x61,0x6d,0x74,0x65,0x63,0x03,0x1a,0x11,0xdd,0x00,
0x00,0x04,0x30,0x11,0x53,0x6c,0x61,0x6d,0x77,0x61,0x72,0x65,0x20,0x53,0x44,0x50,
0x20,0x4d,0x69,0x6e,0x69,0x06,0x20,0xa4,0x70,0xbd,0x3e,0x0a,0x40,0x06,0x50,0x03,
0x19,0x1a,0x00,0x00,0x00,0x00,0x0d,0x50,0x04,0x0e,0x20,0x4a,0x0c,0x02,0x3e,0x0f,
0x20,0x3d,0x0a,0xd7,0x3d,0x11,0x20,0xa6,0xb8,0x32,0x3f,0x10,0x20,0xcd,0xcc,0x4c,
0xbd,0x0b,0x31,0x0c,0x50,0x03,0x19,0x1a,0x01,0x00,0x00,0x00,0x0d,0x50,0x04,0x0e,
0x20,0x4a,0x0c,0x02,0x3e,0x0f,0x20,0x3d,0x0a,0xd7,0xbd,0x11,0x20,0xbb,0xb8,0xb2,
0x40,0x10,0x20,0xcd,0xcc,0x4c,0xbd,0x0b,0x31,0x0c,0x50,0x03,0x19,0x1a,0x02,0x00,
0x00,0x00,0x0d,0x50,0x02,0x0e,0x20,0xc3,0xf5,0x28,0x3e,0x10,0x20,0xcd,0xcc,0x4c,
0xbd,0x0b,0x31,0x0c,0x50,0x05,0x19,0x1a,0x04,0x00,0x00,0x00,0x0d,0x50,0x04,0x0e,
0x20,0x7b,0x14,0xae,0x3d,0x0f,0x20,0x3d,0x0a,0xd7,0x3d,0x11,0x20,0x8e,0xe3,0x38,
0x3f,0x10,0x20,0xcd,0xcc,0x4c,0x3d,0x3c,0x20,0x00,0x00,0x90,0x40,0x3d,0x20,0x0a,
0xd7,0xa3,0x3c,0x0b,0x31,0x15,0x50,0x05,0x19,0x1a,0x05,0x00,0x00,0x00,0x0d,0x50,
0x04,0x0e,0x20,0x7b,0x14,0xae,0x3d,0x0f,0x20,0x3d,0x0a,0xd7,0xbd,0x11,0x20,0x85,
0xeb,0xb1,0x40,0x10,0x20,0xcd,0xcc,0x4c,0x3d,0x3c,0x20,0x00,0x00,0x90,0x40,0x3d,
0x20,0x0a,0xd7,0xa3,0x3c,0x0b,0x31,0x15,0x50,0x05,0x19,0x1a,0x06,0x00,0x00,0x00,
0x0d,0x50,0x02,0x0e,0x20,0x00,0x00,0x00,0x3e,0x10,0x20,0xcd,0xcc,0x4c,0x3d,0x3c,
0x20,0x00,0x00,0x90,0x40,0x3d,0x20,0x0a,0xd7,0xa3,0x3c,0x0b,0x31,0x15,0x27,0x50,
0x02,0x26,0x60,0x11,0x20,0xdb,0x0f,0x49,0x40,0x28,0x20,0x68,0x91,0x8d,0x3e,0x1f,
0x50,0x01,0x21,0x60,0x16,0x50,0x01,0x17,0x61,0x18,0x50,0x02,0x0d,0x50,0x01,0x0f,
0x20,0x29,0x5c,0x8f,0x3d,0x29,0x61,0x1a,0x50,0x01,0x1e,0x20,0x9a,0x99,0x99,0x3d
};
const size_t slamware_config_size = 352;
