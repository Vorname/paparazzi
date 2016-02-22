#pragma once

#define FINKEN_SONAR_DIFF_FREE_DIST (FINKEN_SONAR_DIFF_GOAL_DIST*FINKEN_SONAR_DIFF_FREE_FACTOR)
#define FINKEN_SONAR_DIFF_GUARD_DIST (FINKEN_SONAR_DIFF_GOAL_DIST*FINKEN_SONAR_DIFF_GUARD_FACTOR)

#include <stdint.h>
struct sonar_diff_s {
	int16_t x, y;
};

extern struct sonar_values_s sonar_filtered_values;
extern struct sonar_diff_s sonar_filtered_diff_values;

void finken_sonar_filter_init(void);
void finken_sonar_filter_periodic(void);
