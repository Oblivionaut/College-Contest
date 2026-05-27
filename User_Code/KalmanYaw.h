#ifndef __KALMANYAW_H
#define __KALMANYAW_H

#include "headfile.h"

typedef struct
{
    float angle;

    float bias;

    float rate;

    float P[2][2];

    float Q_angle;

    float Q_bias;

    float R_measure;

}KalmanYaw_t;

extern KalmanYaw_t KalmanYaw;

void KalmanYaw_Init(void);

float KalmanYaw_Update(float newAngle,
                       float newRate,
                       float dt);

#endif
					   
					   