// src/time/time.h
#ifndef TIME_H
#define TIME_H

void set_timezone(void);
void initialize_sntp(void);
void obtain_time(void);
void print_current_time(void);

#endif // TIME_H
