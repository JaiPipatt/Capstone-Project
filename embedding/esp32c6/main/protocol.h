#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* ----------------------------------------------------------------------
 * Frame layout (all multi-byte values little-endian):
 *
 *   [0xAA][0x55][TYPE][LEN][PAYLOAD (LEN bytes)][CRC16_lo][CRC16_hi]
 *
 *   CRC16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect)
 *           computed over TYPE, LEN and PAYLOAD.
 * -------------------------------------------------------------------- */

#define PROTO_SOF1          0xAAU
#define PROTO_SOF2          0x55U
#define PROTO_MAX_PAYLOAD   48U
#define PROTO_OVERHEAD      6U      /* SOF1 SOF2 TYPE LEN CRC_L CRC_H */

typedef enum {
    MSG_CMD   = 0x01,   /* host -> STM32 : mode + target for 4 motors    */
    MSG_ZERO  = 0x04,   /* host -> STM32 : zero position of motors (mask) */
    MSG_STATE = 0x81    /* STM32 -> host : position / velocity feedback   */
} MsgType;

typedef enum {
    MODE_STOP     = 0,  /* coast, PID reset                               */
    MODE_VELOCITY = 1,  /* target = rad/s                                 */
    MODE_POSITION = 2,  /* target = rad  (cascade: position -> velocity)  */
    MODE_DUTY     = 3   /* target = duty -1..+1 (open loop, for testing)  */
} CtrlMode;

/* Index order everywhere: [0]=M1_R [1]=M2_L [2]=M3_IR [3]=M4_IL */
typedef struct __attribute__((packed)) {
    uint8_t mode[4];
    float   target[4];
} CmdPayload;                       /* 20 bytes */

typedef struct __attribute__((packed)) {
    uint8_t mask;                   /* bit i set -> zero motor i */
} ZeroPayload;                      /* 1 byte */

typedef struct __attribute__((packed)) {
    uint32_t stamp_ms;
    float    pos[4];                /* rad   */
    float    vel[4];                /* rad/s */
    uint8_t  flags;                 /* bit0 = command watchdog tripped */
} StatePayload;                     /* 37 bytes */

_Static_assert(sizeof(CmdPayload)   == 20U, "CmdPayload size");
_Static_assert(sizeof(StatePayload) == 37U, "StatePayload size");
_Static_assert(sizeof(StatePayload) <= PROTO_MAX_PAYLOAD, "payload too big");

typedef void (*ProtoFrameCb)(uint8_t type, const uint8_t *payload, uint8_t len);

/* One parser per input channel (e.g. one for the ESP32 UART, one for USB).
 * Do not share one parser between two interrupt priorities. */
typedef struct {
    uint8_t      st;
    uint8_t      type;
    uint8_t      len;
    uint8_t      idx;
    uint16_t     crc_rx;
    uint8_t      buf[PROTO_MAX_PAYLOAD];
    ProtoFrameCb cb;
} ProtoParser;

void     Proto_Init(ProtoParser *p, ProtoFrameCb cb);
void     Proto_Reset(ProtoParser *p);              /* drop half-received frame */
void     Proto_Feed(ProtoParser *p, const uint8_t *data, uint16_t n);

/* Build a complete frame into out[]; returns total bytes (len + 6).
 * out must hold at least PROTO_MAX_PAYLOAD + PROTO_OVERHEAD bytes. */
uint16_t Proto_Build(uint8_t type, const void *payload, uint8_t len, uint8_t *out);

#endif /* PROTOCOL_H */
