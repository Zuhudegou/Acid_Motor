#ifndef __BLDC_FOC_H__
#define __BLDC_FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"
#include "tim.h"
#include "adc.h"

#define PWM_PERIOD              4250U

#define ADC_RESOLUTION          4095U           
#define ADC_REF_VOLTAGE         3.3f            
#define ADC_VREF_MV             3300U           

#define POLE_PAIRS              7U              
#define RAD_PER_REV             (2.0f * TWO_PI) 

#define OPAMP_GAIN              1.0f            
#define CURRENT_SENSE_RESISTOR  0.05f           
#define CURRENT_SENSE_GAIN      (OPAMP_GAIN / CURRENT_SENSE_RESISTOR)  

#define ADC_TO_CURRENT_SCALE    (ADC_REF_VOLTAGE / ADC_RESOLUTION / CURRENT_SENSE_RESISTOR)  

#define ADC_OFFSET_U_DEFAULT    2048U           
#define ADC_OFFSET_V_DEFAULT    2048U
#define ADC_OFFSET_W_DEFAULT    2048U           

#define ID_KP                   0.5f            
#define ID_KI                   0.1f            
#define ID_KD                   0.0f            

#define IQ_KP                   0.5f            
#define IQ_KI                   0.1f            
#define IQ_KD                   0.0f            

#define VEL_KP                  0.1f            
#define VEL_KI                  0.05f           
#define VEL_KD                  0.0f            

#define POS_KP                  2.0f            
#define POS_KI                  0.0f            
#define POS_KD                  0.1f            

#define ID_REF_MAX              5.0f            
#define IQ_REF_MAX              10.0f           
#define VEL_REF_MAX             100.0f          
#define I_ERROR_MAX             50.0f           
#define UD_MAX                  400.0f          
#define UQ_MAX                  400.0f          

#define VOLTAGE_LIMIT           400.0f          

#define DUTY_MAX                (PWM_PERIOD - 100)      
#define DUTY_MIN                100                      

#define ELECTRO_ANGLE_SCALE     (65535U)        
#define SQRT3                   1.732050808f    
#define SQRT3_INV               0.577350269f    

typedef struct {

    uint16_t  adc_u_raw;        
    uint16_t  adc_v_raw;        

    int16_t   adc_u_offset;     
    int16_t   adc_v_offset;     

    float     i_u;              
    float     i_v;              
    float     i_w;              

} CurrentSensor_t;

typedef struct {
    float     alpha;            
    float     beta;             
} AlphaBeta_t;

typedef struct {
    float     d;                
    float     q;                
} DQ_t;

typedef struct {
    float     kp;               
    float     ki;               
    float     kd;               

    float     error;            
    float     error_prev;       
    float     integral;         
    float     output;           
    float     output_max;       
} PIDController_t;

typedef enum {
    FOC_MODE_TORQUE = 0,        
    FOC_MODE_VELOCITY,          
    FOC_MODE_POSITION,          
    FOC_MODE_OPEN_LOOP,         
    FOC_MODE_HOMING             
} FOC_ControlMode_t;

typedef struct {

    FOC_ControlMode_t mode;     
    float     id_ref;           
    float     iq_ref;           
    float     vel_ref;          
    float     pos_ref;          

    float     theta_e;          
    uint16_t  theta_e_angle;    
    float     theta_m;          
    float     velocity_m;       

    DQ_t      i_dq_meas;        
    DQ_t      i_dq_ref;         
    DQ_t      i_dq_error;       

    DQ_t      u_dq;             
    AlphaBeta_t u_ab;           

    uint16_t  duty_u;           
    uint16_t  duty_v;           
    uint16_t  duty_w;           

    PIDController_t pid_id;     
    PIDController_t pid_iq;     
    PIDController_t pid_vel;    
    PIDController_t pid_pos;    

    bool      enable;           
    bool      fault;            
    uint32_t  loop_counter;     

} FOCControl_t;

typedef struct {
    float     ua;               
    float     ub;               
    float     uc;               

} SVPWM_Duty_t;

extern FOCControl_t g_foc_ctrl;
extern CurrentSensor_t g_current_sensor;

void FOC_Init(void);

void FOC_CascadeLoopCallback(void);

void FOC_CurrentLoopCallback(ADC_HandleTypeDef *hadc);

void FOC_SetReferences(float id_ref, float target, float theta_e);

void FOC_SetControlMode(FOC_ControlMode_t mode);

void FOC_UpdateFeedback(float mech_angle, float mech_vel);

void FOC_SetEnable(bool enable);

const FOCControl_t* FOC_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
