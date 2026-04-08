#ifndef PRINT_H
#define PRINT_H

#ifdef __cplusplus
extern "C" {
#endif

void dbg(char* format, ...);
void info(char* format, ...);
void err(char* format, ...);

#ifdef __cplusplus
}
#endif

#endif