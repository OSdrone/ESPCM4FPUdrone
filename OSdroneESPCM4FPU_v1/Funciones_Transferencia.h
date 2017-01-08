
#ifndef QUADROTOR_V1_3_1_FUNCIONES_TRANSFERENCIA_H_
#define QUADROTOR_V1_3_1_FUNCIONES_TRANSFERENCIA_H_

#include "arm_math.h"

#define Jq1 0.8613
#define Jq2 0.8613
#define Jq3 0.8613

#define Jm  0.096369

#define Km 0.009637	   //Fuerza(N) / Accion (microseg)
#define K_sistema 1.8931
#define K_q K_sistema/Km;

#define Wn 10;
#define Wn_2 10;
#define chi 1;

#define Kpr 0.0

/*
Kp1 = Wn^2*Jq1/K;
Kp2 = Wn^2*Jq2/K;
Kp3 = Wn^2*Jq3/K2;

Kv1 = (2*chi*Wn - 1) / K;
Kv2 = (2*chi*Wn - 1) / K;
Kv3 = (2*chi*Wn - 1) / K2;
 */


/*
#define Kp1 15.0
#define Kv1 2.5

#define Kp2 15.0
#define Kv2 2.5

#define Kp3 15.0
#define Kv3 2.5
*/

#define Kp1 15
#define Kv1 2.5

#define Kp2 0.0
#define Kv2 0.0

#define Kp3 0.0
#define Kv3 0.0

#define Ki 0.00002
#define Ki_EST 0.0002

#define Kp_ROLLPITCH 0.00005
#define Ki_ROLLPITCH 0.00000001

#define Kp_YAW 0.00005
#define Ki_YAW 0.00000001


extern const float32_t F_matriz[];
extern const arm_matrix_instance_f32 F;

extern const float32_t G_matriz[];
extern const arm_matrix_instance_f32 G;

extern const float32_t Gp_matriz[];
extern const arm_matrix_instance_f32 Gp;

extern const float32_t K_4_matriz[];
extern const arm_matrix_instance_f32 K_4;

extern const float32_t K_3_matriz[];
extern const arm_matrix_instance_f32 K_3;

extern const float32_t K_pre_4_matriz[];
extern const arm_matrix_instance_f32 K_pre_4;

extern const float32_t K_pre_3_matriz[];
extern const arm_matrix_instance_f32 K_pre_3;

extern const float32_t Lo_per_matriz[];
extern const arm_matrix_instance_f32 Lo_per;

extern const float32_t Lp_matriz[];
extern const arm_matrix_instance_f32 Lp;

extern const float32_t La_matriz[];
extern const  arm_matrix_instance_f32 La;


#endif /* QUADROTOR_V1_3_1_FUNCIONES_TRANSFERENCIA_H_ */
