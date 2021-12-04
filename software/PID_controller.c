#include "PID_controller.h"

#include <math.h> 
#include <stdlib.h>
#include <stdio.h>
#include "nrf_log.h"
#include "functions.h"

double PID_controller(PID_parameters_t* pid, double reference, double measurement, double period)
{

    //Find error 
    double error    = reference - measurement; 
    return PID_controller_with_error_as_input(pid, error, reference, measurement, period);

}

double PID_controller_with_error_as_input(PID_parameters_t* pid, double error, double reference, double measurement, double period)
{

    //Add error to integral
    pid->error_integral  += error;
    
    //Prevent integral windup
    if(pid->error_integral > pid->integral_boundary) pid->error_integral = pid->integral_boundary;
    else if(pid->error_integral < -pid->integral_boundary) pid->error_integral = -pid->integral_boundary;



    //PID
    double  output = 0;
    switch(pid->derivative_select){
        case MEASUREMENT:
            output =    pid->K_P * error  
                    +  (pid->K_I * pid->error_integral *  period)  
                    +  (pid->K_D * (error - pid->error_previous)) / period;
            break;  
        case ERROR:
        default:
            output =    pid->K_P * error  
                    +  (pid->K_I * pid->error_integral *  period)  
                    +  (pid->K_D * (error - pid->error_previous)) / period;
            break;
    }
    

    //Limit output
    double output_limited = output;
    if(output  >  pid->max_output ) output_limited  = pid->max_output ;
    else if(output  < -pid->max_output ) output_limited  = -pid->max_output ;
    else output_limited = output;

    //Update parameters for next iteration
    pid->measurement_previous = measurement;
    pid->error_previous  =  error;
    pid->error_previous = output_limited ; 
    pid->reference_previous = reference;

    //Stop if output is close to zero
    if(abs(output_limited) < pid->min_output) output_limited = 0;

    return output_limited;
}