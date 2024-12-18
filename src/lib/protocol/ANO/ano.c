#include "ano.h"

/* get sum & add check data */
void _ano_check(const uint8_t* data, ano_check_t* ano_check)
{
    for(int i = 0; i < (data[3] + 4); i++)
    {
        ano_check->sum_check += data[i];
        ano_check->add_check += ano_check->sum_check;
    }
}

/* check if the buffer data is an ANO packet, return result */
ano_res_t ano_check(uint8_t* buffer)
{
    ano_check_t ano_check;

    _ano_check(buffer, &ano_check);
    if((buffer[buffer[3] + 4] == ano_check.sum_check) && (buffer[buffer[3] + 5] == ano_check.add_check))
    {
        return ANO_OK;
    }

    return ANO_ERROR;
}


/* pack data as ANO's format, return an ANO packet */
uint8_t* ano_pack_data(uint8_t d_addr, enum ano_id id, uint8_t data_length, uint8_t* data)
{
    uint8_t* ano_packet = malloc(ANO_ELSE_DATA_PACKET_LENGTH + data_length);
    if(ano_packet == NULL)
    {
        return NULL;
    }

    ano_packet[0] = ANO_HEAD;
    ano_packet[1] = d_addr;
    ano_packet[2] = id;
    ano_packet[3] = data_length;
    memcpy(&ano_packet[4], data, data_length);

    ano_check_t ano_check;
    _ano_check(ano_packet, &ano_check);

    ano_packet[4 + data_length] = ano_check.sum_check;
    ano_packet[5 + data_length] = ano_check.add_check;
    
    return ano_packet;
}
