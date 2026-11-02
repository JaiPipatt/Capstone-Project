#include "protocol.h"
#include <string.h>

enum { S_SOF1, S_SOF2, S_TYPE, S_LEN, S_PAYLOAD, S_CRC_L, S_CRC_H };

static uint16_t crc16_update(uint16_t crc, const uint8_t *d, uint16_t n)
{
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void Proto_Init(ProtoParser *p, ProtoFrameCb cb)
{
    p->cb = cb;
    Proto_Reset(p);
}

void Proto_Reset(ProtoParser *p)
{
    p->st  = S_SOF1;
    p->idx = 0U;
    p->len = 0U;
}

void Proto_Feed(ProtoParser *p, const uint8_t *data, uint16_t n)
{
    for (uint16_t k = 0; k < n; k++) {
        const uint8_t b = data[k];

        switch (p->st) {
        case S_SOF1:
            if (b == PROTO_SOF1) p->st = S_SOF2;
            break;

        case S_SOF2:
            if (b == PROTO_SOF2)      p->st = S_TYPE;
            else if (b == PROTO_SOF1) p->st = S_SOF2;   /* AA AA 55 ... */
            else                      p->st = S_SOF1;
            break;

        case S_TYPE:
            p->type = b;
            p->st = S_LEN;
            break;

        case S_LEN:
            if (b > PROTO_MAX_PAYLOAD) {
                p->st = S_SOF1;                         /* nonsense length */
            } else {
                p->len = b;
                p->idx = 0U;
                p->st = (b == 0U) ? S_CRC_L : S_PAYLOAD;
            }
            break;

        case S_PAYLOAD:
            p->buf[p->idx++] = b;
            if (p->idx >= p->len) p->st = S_CRC_L;
            break;

        case S_CRC_L:
            p->crc_rx = b;
            p->st = S_CRC_H;
            break;

        case S_CRC_H: {
            p->crc_rx |= (uint16_t)((uint16_t)b << 8);

            const uint8_t hdr[2] = { p->type, p->len };
            uint16_t crc = crc16_update(0xFFFFU, hdr, 2U);
            crc = crc16_update(crc, p->buf, p->len);

            if (crc == p->crc_rx && p->cb != 0) {
                p->cb(p->type, p->buf, p->len);
            }
            p->st = S_SOF1;
            break;
        }

        default:
            p->st = S_SOF1;
            break;
        }
    }
}

uint16_t Proto_Build(uint8_t type, const void *payload, uint8_t len, uint8_t *out)
{
    out[0] = PROTO_SOF1;
    out[1] = PROTO_SOF2;
    out[2] = type;
    out[3] = len;
    memcpy(&out[4], payload, len);

    const uint16_t crc = crc16_update(0xFFFFU, &out[2], (uint16_t)(2U + len));
    out[4U + len] = (uint8_t)(crc & 0xFFU);
    out[5U + len] = (uint8_t)(crc >> 8);

    return (uint16_t)(len + PROTO_OVERHEAD);
}
