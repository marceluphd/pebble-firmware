#include <string.h>
#include <assert.h>
#include "pb_pack.h"
#include "endian_conv.h"


static int pb_pack_varint(const pb_st_item *item, uint8_t *buffer) {

    uint8_t *key = buffer;
    uint8_t *value = buffer + 1;
    uint64_t varint = *((uint64_t *)item->data);

    *key = PB_SET_KEY(item->field, PB_WT_VARINT);

    do {
        *value = PB_VARINT_MORE | (varint & PB_VARINT_MASK);
        varint >>= 7;
        value++;
    } while (varint);

    *(--value) &= ~PB_VARINT_MORE;

    return value - buffer + 1;
}

static int pb_pack_64bit(const pb_st_item *item, uint8_t *buffer) {

    uint8_t *key = buffer;
    uint8_t *value = buffer + 1;
    uint64_t fixed64 = ENDIAN_H2LE64(*((uint64_t *)item->data));

    *key = PB_SET_KEY(item->field, PB_WT_64);
    memcpy(value, &fixed64, sizeof(fixed64));

    return sizeof(fixed64) + 1;
}

static int pb_pack_32bit(const pb_st_item *item, uint8_t *buffer) {

    uint8_t *key = buffer;
    uint8_t *value = buffer + 1;
    uint32_t fixed32 = ENDIAN_H2LE32(*((uint32_t *)item->data));

    *key = PB_SET_KEY(item->field, PB_WT_32);
    memcpy(value, &fixed32, sizeof(fixed32));

    return sizeof(fixed32) + 1;
}

static int pb_pack_ld(const pb_st_item *item, uint8_t *buffer) {

    uint8_t *key = buffer;
    uint8_t *value = buffer + 2;
    uint8_t *length = buffer + 1;

    *length = item->length;
    *key = PB_SET_KEY(item->field, PB_WT_LD);
    memcpy(value, item->data, item->length);

    return item->length + 2;
}

int pb_pack(const pb_st_item *messages, size_t len, uint8_t *buffer, size_t size) {

    assert(messages != NULL);
    assert(buffer != NULL);

    int i = 0;
    int emb_msg_len;
    int packed_len = 0;
    uint8_t write_type = 0;
    PB_SERIALIZE_FUNC serialize = NULL;
    static const PB_SERIALIZE_FUNC serialize_funcs[PB_MAX_TYPE + 1] = {
        pb_pack_varint,
        pb_pack_64bit,
        pb_pack_ld,
        NULL,
        NULL,
        pb_pack_32bit,
    };

    for (i = 0; i < len; i++, messages++) {
        write_type = PB_GET_WTYPE(messages->type);

        if (write_type > PB_MAX_TYPE) {
            return -PB_TYPE_ERR;
        }

        /* Skip deprecated type and optional field */
        if (!(serialize = serialize_funcs[write_type]) || !messages->data) {
            continue;
        }

        /* Pack embedded message */
        if (IS_EMB_MSG(messages->type)) {
            emb_msg_len = pb_pack((const pb_st_item *)messages->data, messages->length, buffer + packed_len + 2, size - packed_len - 2);

            if (emb_msg_len < 0) {
                return -PB_PEMB_ERR;
            }

            buffer[packed_len] = PB_SET_KEY(messages->field, PB_WT_LD);
            buffer[packed_len + 1] = emb_msg_len;
            packed_len += emb_msg_len + 2;
        }
        else {
            packed_len += serialize(messages, buffer + packed_len);
        }

        /* Buffer to short */
        if (packed_len >= size) {
            return -PB_BUFF_ERR;
        }
    }

    return packed_len;
}
