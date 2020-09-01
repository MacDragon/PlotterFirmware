//
//  gparse.hpp
//  GCodeParser
//
//  Created by Visa Harvey on 1.9.2020.
//

#ifndef gparse_hpp
#define gparse_hpp

#include <stdio.h>

/*
 
 Relatively simple parser for mdraw output gcode.
 
 But advantages e.g. deterministic memory usage, doesn't modify string to be parsed so const safe, only allocates a known set of processing buffers with stack of ~ 15 bytes of buffers + a few counters and return struct, so very lean for embedded use and not limited on buffer length.
 
 */

enum command_type { bad, none, init, limit, savepen, setpen, savestepper, setlaser, origin, goxy };

// structs for the various command requirements, all as in32_t's to allow array indexing them interchangeably.

struct penstore {
    int32_t up;
    int32_t down;
};

struct pen {
    int32_t pos;
};

struct stepper {
    int32_t A;
    int32_t B;
    int32_t height;
    int32_t width;
    int32_t speed;
};

struct laser {
    int32_t power;
};

struct pos {
    int32_t x; // mm * 100
    int32_t y; // mm * 100
    int32_t abs;
};

// not sure if best way to do this, but would ensure that any queues would always have identical sized items if stored in full..
struct command {
    command_type cmd;
    union {
        struct penstore penstore;
        struct pen pen;
        struct stepper stepper;
        struct laser laser;
        struct pos pos;
    };
};

command GCodeParser( const char *instr );

#endif /* gparse_hpp */
