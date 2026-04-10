#ifndef ENCODER_H
#define ENCODER_H

void Encoder_Init(void);
float Encoder_Get_L(void);
float Encoder_Get_R(void);
void Encoder_Get_Speed(void);
extern float speed_L;
extern float speed_R;
#endif /* ENCODER_H */
