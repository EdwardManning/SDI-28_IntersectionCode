#pragma once

//data types to reduce memory usage and make reading easier
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

typedef signed char int8;

typedef float (*modifier)(float period);

typedef float (*probability)(float);