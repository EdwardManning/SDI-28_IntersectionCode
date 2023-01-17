#pragma once

#define SWERRINT(x) fprintf(stderr, "SWERR at %s:%d - (%d)\n", __FILE__, __LINE__, x)
#define SWERRSTR(x) fprintf(stderr, "SWERR at %s:%d - (%s)\n", __FILE__, __LINE__, x)

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

const bool X = 0;
const bool Y = 1;