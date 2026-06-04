#include "KalmanYaw.h"

KalmanYaw_t KalmanYaw;

static float KalmanYaw_Normalize(float Angle)
{
    while(Angle >= 360.0f)
    {
        Angle -= 360.0f;
    }

    while(Angle < 0.0f)
    {
        Angle += 360.0f;
    }

    return Angle;
}

static float KalmanYaw_Error(float Target, float Current)
{
    float Error;

    Error = KalmanYaw_Normalize(Target) - KalmanYaw_Normalize(Current);

    while(Error > 180.0f)
    {
        Error -= 360.0f;
    }

    while(Error < -180.0f)
    {
        Error += 360.0f;
    }

    return Error;
}

void KalmanYaw_Init(void)
{
    KalmanYaw.angle = 0.0f;
    KalmanYaw.bias = 0.0f;
    KalmanYaw.rate = 0.0f;

    KalmanYaw.P[0][0] = 0.0f;
    KalmanYaw.P[0][1] = 0.0f;
    KalmanYaw.P[1][0] = 0.0f;
    KalmanYaw.P[1][1] = 0.0f;

    KalmanYaw.Q_angle = 0.300f;
    KalmanYaw.Q_bias = 0.003f;
    KalmanYaw.R_measure = 0.250f;
}

void KalmanYaw_SetAngle(float angle)
{
    KalmanYaw_Init();
    KalmanYaw.angle = KalmanYaw_Normalize(angle);
}

float KalmanYaw_Predict(float newRate, float dt)
{
    KalmanYaw.rate = newRate - KalmanYaw.bias;
    KalmanYaw.angle =
        KalmanYaw_Normalize(KalmanYaw.angle + dt * KalmanYaw.rate);

    KalmanYaw.P[0][0] +=
        dt * (dt * KalmanYaw.P[1][1] -
              KalmanYaw.P[0][1] -
              KalmanYaw.P[1][0] +
              KalmanYaw.Q_angle);

    KalmanYaw.P[0][1] -= dt * KalmanYaw.P[1][1];
    KalmanYaw.P[1][0] -= dt * KalmanYaw.P[1][1];
    KalmanYaw.P[1][1] += KalmanYaw.Q_bias * dt;

    return KalmanYaw.angle;
}

float KalmanYaw_Correct(float newAngle)
{
    float S;
    float K0;
    float K1;
    float Y;
    float P00;
    float P01;

    S = KalmanYaw.P[0][0] + KalmanYaw.R_measure;

    if(S <= 0.000001f)
    {
        return KalmanYaw.angle;
    }

    K0 = KalmanYaw.P[0][0] / S;
    K1 = KalmanYaw.P[1][0] / S;
    Y = KalmanYaw_Error(newAngle, KalmanYaw.angle);

    KalmanYaw.angle = KalmanYaw_Normalize(KalmanYaw.angle + K0 * Y);
    KalmanYaw.bias += K1 * Y;

    P00 = KalmanYaw.P[0][0];
    P01 = KalmanYaw.P[0][1];

    KalmanYaw.P[0][0] -= K0 * P00;
    KalmanYaw.P[0][1] -= K0 * P01;
    KalmanYaw.P[1][0] -= K1 * P00;
    KalmanYaw.P[1][1] -= K1 * P01;

    return KalmanYaw.angle;
}

float KalmanYaw_Update(float newAngle, float newRate, float dt)
{
    (void)KalmanYaw_Predict(newRate, dt);
    return KalmanYaw_Correct(newAngle);
}
