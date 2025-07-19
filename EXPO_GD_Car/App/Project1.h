#ifndef _PROJECT1_H
#define _PROJECT1_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "systick.h"
#include "hsp_timer.h"
#include "Ex3.h"
#include "Ex6.h"
#include "Ex7.h"

extern volatile int cross_detected;        // cross detection flag

void Project_LFR(void);
uint16_t hsp_image_judge(image2_t image);
uint16_t hsp_image_judge2(image2_t image);
uint8_t hsp_cross_detect(image2_t image);
void hsp_image2_binary_simple(image2_t input, image2_t output);

#endif