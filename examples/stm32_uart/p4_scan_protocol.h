#ifndef P4_SCAN_PROTOCOL_H
#define P4_SCAN_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

#define P4_SCAN_SEQUENCE_MAX 999u

/* UART v6: compact == 1 for C (Compact), 0 for F (Full).
 * Only yaw_mdeg/plane_mm/along_mm are present in Compact.
 * Interpret pose fields only if pose_valid == 1.
 * Call from main loop / UART DMA consumer, not from an interrupt handler.
 */
typedef struct {
    uint32_t sequence; /* Capture sequence modulo 1000. */
    uint8_t compact, pose_valid;
    uint16_t payload_length;
    uint8_t payload[255]; /* ASCII '0'..'9'; use payload_length, no terminator guaranteed. */
    int32_t corners_x10[8];
    int32_t yaw_mdeg, pitch_mdeg, roll_mdeg;
    int32_t range_mm, depth_mm, plane_mm, tx_mm, along_mm;
} p4_scan_packet_t;

/* Returns 1 = valid telemetry, 0 = another message, -1 = invalid packet.
 * Accepts a single line with optional CRLF; output is unchanged on rejection.
 * Maximum accepted line length including CRLF is 1024 bytes.
 */
int p4_scan_parse_line(const char *line, size_t length, p4_scan_packet_t *output);
uint16_t p4_scan_crc16(const char *bytes, size_t length);
/* First payload character '0'/'1'/'2' selects 30/50/70 mm.
 * Returns 0 for empty/null payload or an unknown prefix.
 * Payload is a digit string; preserve leading zero. Distances are already
 * calculated by P4, so do not multiply received distances by this side.
 */
uint16_t p4_scan_code_side_mm(const uint8_t *payload, size_t payload_length);
#endif
