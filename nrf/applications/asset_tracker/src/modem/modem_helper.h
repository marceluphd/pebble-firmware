#ifndef _IOTEX_MODEM_H_
#define _IOTEX_MODEM_H_


#define TIMESTAMP_STR_LEN 50

typedef struct {
    char data[TIMESTAMP_STR_LEN];
} __attribute__((packed)) iotex_st_timestamp;

char *iotex_modem_get_clock(iotex_st_timestamp *stamp);

#endif //_IOTEX_MODEM_H_