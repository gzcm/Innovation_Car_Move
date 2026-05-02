#ifndef __TRACKER_H
#define __TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

void Tracker_Init(float base_rps, float k_track);
void Tracker_SetSpeed(float base_rps);
void Tracker_Update(float position, float *target_left, float *target_right);

#ifdef __cplusplus
}
#endif

#endif
