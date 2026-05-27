#include "headfile.h"

KalmanYaw_t KalmanYaw;

void KalmanYaw_Init(void)
{
    KalmanYaw.Q_angle = 0.001f;

    KalmanYaw.Q_bias = 0.003f;

    KalmanYaw.R_measure = 0.5f;
}

float KalmanYaw_Update(float newAngle,
                       float newRate,
                       float dt)
{
    KalmanYaw.rate =
        newRate - KalmanYaw.bias;

    KalmanYaw.angle +=
        dt * KalmanYaw.rate;

    KalmanYaw.P[0][0] +=
        dt * (dt * KalmanYaw.P[1][1]
        - KalmanYaw.P[0][1]
        - KalmanYaw.P[1][0]
        + KalmanYaw.Q_angle);

    KalmanYaw.P[0][1] -=
        dt * KalmanYaw.P[1][1];

    KalmanYaw.P[1][0] -=
        dt * KalmanYaw.P[1][1];

    KalmanYaw.P[1][1] +=
        KalmanYaw.Q_bias * dt;

    float S =
        KalmanYaw.P[0][0]
        + KalmanYaw.R_measure;

    float K0 =
        KalmanYaw.P[0][0] / S;

    float K1 =
        KalmanYaw.P[1][0] / S;

    float y =
        newAngle - KalmanYaw.angle;

    KalmanYaw.angle += K0 * y;

    KalmanYaw.bias += K1 * y;

    float P00 =
        KalmanYaw.P[0][0];

    float P01 =
        KalmanYaw.P[0][1];

    KalmanYaw.P[0][0] -=
        K0 * P00;

    KalmanYaw.P[0][1] -=
        K0 * P01;

    KalmanYaw.P[1][0] -=
        K1 * P00;

    KalmanYaw.P[1][1] -=
        K1 * P01;

    if(KalmanYaw.angle < 0)
    {
        KalmanYaw.angle += 360;
    }

    if(KalmanYaw.angle >= 360)
    {
        KalmanYaw.angle -= 360;
    }

    return KalmanYaw.angle;
}
