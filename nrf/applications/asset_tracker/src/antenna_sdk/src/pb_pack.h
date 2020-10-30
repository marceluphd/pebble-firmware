#ifndef _IOTEX_PB_PACK_H_
#define _IOTEX_PB_PACK_H_

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef enum {
    PB_WT_VARINT = 0,
    PB_WT_64 = 1,
    PB_WT_LD = 2,
    PB_WT_32 = 5,
    PB_WT_EMB = 0x82
} pb_em_wtype;

typedef enum {
    PB_NO_ERR,
    PB_TYPE_ERR,
    PB_BUFF_ERR,
    PB_PEMB_ERR
} pb_em_error;

/* NULL means it's an optional field */
typedef struct {
    /* write type one of pb_em_wtype or PB_EMBMSG_MASK | PB_WT_LD */
    uint8_t type;

    /* field number */
    uint8_t field;

    /* data to write, for embedded messages, it pointer to embedded message items list */
    void *data;

    /* for length-delimited type use, indicate data length in byte, for embedded messages it means embedded message item length */
    uint8_t length;
} pb_st_item;

#define PB_MAX_TYPE (PB_WT_32)

#define PB_VARINT_MASK 0x7f
#define PB_VARINT_MORE 0x80

#define PB_EMBMSG_MASK 0x80
#define PB_GET_WTYPE(x) ((x) & 0x7)
#define PB_GET_FIELD(x) (((x) & 0x78) >> 3)
#define PB_SET_KEY(field, type) (((field & 0xf) << 3) | ((type) & 0x7))
#define IS_EMB_MSG(x) (PB_GET_WTYPE(x) == PB_WT_LD && ((x) & (PB_EMBMSG_MASK)))


typedef int (*PB_SERIALIZE_FUNC)(const pb_st_item *item, uint8_t *buffer);


int pb_pack(const pb_st_item *messages, size_t len, uint8_t *buffer, size_t size);

#ifdef	__cplusplus
}
#endif


#endif /* _IOTEX_PB_PACK_H_ */
