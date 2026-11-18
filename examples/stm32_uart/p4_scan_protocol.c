#include "p4_scan_protocol.h"
#include <limits.h>
#include <string.h>

uint16_t p4_scan_code_side_mm(const uint8_t *payload, size_t payload_length)
{
    if (!payload || !payload_length) return 0;
    switch (payload[0]) {
    case '0': return 30;
    case '1': return 50;
    case '2': return 70;
    default: return 0;
    }
}

uint16_t p4_scan_crc16(const char *bytes, size_t length)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)(uint8_t)bytes[i] << 8;
        for (int j = 0; j < 8; ++j)
            crc = (uint16_t)((crc & 0x8000) ? (crc << 1)^0x1021 : crc << 1);
    }
    return crc;
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c-'0';
    if (c >= 'A' && c <= 'F') return c-'A'+10;
    if (c >= 'a' && c <= 'f') return c-'a'+10;
    return -1;
}

static int number(const char *text, int negative_allowed, uint32_t max, int64_t *value)
{
    int negative = *text == '-';
    if (negative && !negative_allowed) return 0;
    if (negative) ++text;
    if (!*text) return 0;
    uint32_t n = 0;
    while (*text) {
        if (*text < '0' || *text > '9') return 0;
        uint32_t d = (uint32_t)(*text++-'0');
        if (n > max/10 || (n == max/10 && d > max%10)) return 0;
        n = n*10+d;
    }
    *value = negative ? -(int64_t)n : (int64_t)n;
    return 1;
}

int p4_scan_parse_line(const char *line, size_t length, p4_scan_packet_t *output)
{
    if (!line || !output || length > 1024) return -1;
    if (length < 6 || memcmp(line, "@SCAN,", 6)) return 0;
    while (length && (line[length-1] == '\r' || line[length-1] == '\n')) --length;
    if (length < 7 || line[length-5] != '*') return -1;
    size_t body_length = length-5;
    uint16_t expected = 0;
    for (size_t i = length-4; i < length; ++i) {
        int d = hex_digit(line[i]);
        if (d < 0) return -1;
        expected = (uint16_t)((expected << 4)|d);
    }
    if (expected != p4_scan_crc16(line, body_length)) return -1;
    char buffer[1025];
    memcpy(buffer, line, body_length);
    buffer[body_length] = 0;
    if (memchr(buffer, 0, body_length)) return -1;
    char *fields[22];
    unsigned count = 1;
    fields[0] = buffer;
    for (size_t i = 0; i < body_length; ++i) {
        if (buffer[i] == ',') {
            if (count == 22) return -1;
            buffer[i] = 0;
            fields[count++] = buffer+i+1;
        }
    }
    if (count < 6 || strcmp(fields[1], "6")) return -1;
    int compact = !strcmp(fields[2], "C");
    if ((!compact && strcmp(fields[2], "F")) || count != (compact ? 9u : 22u)) return -1;
    p4_scan_packet_t packet = {0};
    packet.compact = (uint8_t)compact;
    int64_t n;
    if (!number(fields[3], 0, P4_SCAN_SEQUENCE_MAX, &n)) return -1;
    packet.sequence = (uint32_t)n;
    if (!number(fields[4], 0, 1, &n)) return -1;
    packet.pose_valid = (uint8_t)n;
    size_t payload_length = strlen(fields[5]);
    if (!payload_length || payload_length > sizeof(packet.payload)) return -1;
    for (size_t i = 0; i < payload_length; ++i) {
        if (fields[5][i] < '0' || fields[5][i] > '9') return -1;
    }
    packet.payload_length = (uint16_t)payload_length;
    memcpy(packet.payload, fields[5], payload_length);
    unsigned start = 6;
    if (!compact) {
        for (unsigned i = 0; i < 8; ++i) {
            if (!number(fields[start++], 0, i%2 ? 7200 : 12800, &n)) return -1;
            packet.corners_x10[i] = (int32_t)n;
        }
    }
    int32_t *dest[8] = {&packet.yaw_mdeg, &packet.pitch_mdeg, &packet.roll_mdeg,
        &packet.range_mm, &packet.depth_mm, &packet.plane_mm, &packet.tx_mm, &packet.along_mm};
    for (unsigned i = 0; i < 8; ++i) {
        if (compact && i != 0 && i != 5 && i != 7) continue;
        const char *text = fields[start++];
        if (!number(text, 1, text[0] == '-' ? 2147483648u : INT32_MAX, &n)) return -1;
        *dest[i] = (int32_t)n;
    }
    *output = packet;
    return 1;
}
