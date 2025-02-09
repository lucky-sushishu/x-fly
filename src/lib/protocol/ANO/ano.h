#ifndef _ANO_H_
#define _ANO_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ANO_OK    0
#define ANO_ERROR 1

typedef int ano_res_t;

#define ANO_HEAD           (0xAA)
#define ANO_BROADCAST_ADDR (0xFF)

#define ANO_ELSE_DATA_PACKET_LENGTH 6

typedef signed char int8_t;
typedef signed short int16_t;
typedef signed int int32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;


#define ANO_ATTITUDE_EULER_LENGTH 7
#define ANO_ATTITUDE_Q_LENGTH 9

#define ANO_PARAM_READ_1_LENGTH 2
#define ANO_PARAM_READ_2_LENGTH 4

enum ano_id {
    ATTITUDE_EULER = 0x03,
    ATTITUDE_Q = 0x04,

    PARAM_READ = 0xE1,
};

typedef struct ano_check_s {
    uint8_t sum_check;
    uint8_t add_check;
} ano_check_t;

typedef struct attitude_euler_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t yaw_1;
    int8_t fusion_status;
} attitude_euler_t;

typedef struct attitude_q_s {
    int16_t v0;
    int16_t v1;
    int16_t v2;
    int16_t v3;
    int8_t fusion_status;
} attitude_q_t;

typedef struct param_read_1_s {
    uint16_t param_id;
} param_read_1_t;

typedef struct param_read_2_s {
    uint16_t param_id_start;
    uint16_t read_num;
} param_read_2_t;

ano_res_t ano_check(uint8_t* buffer);
uint8_t* ano_pack_data(uint8_t d_addr, enum ano_id id, uint8_t data_length, uint8_t* data);

#endif
