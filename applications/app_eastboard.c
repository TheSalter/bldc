#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "commands.h" // Debug Print
#include "math.h"
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "comm_can.h" //enable Can communication
#include "imu.h" //include IMU functions
#include "utils.h" //Filter
#include "datatypes.h"
#include "app.h"

// Settings
#define OUTPUT_ITERATION_TIME_MS    8 //to run every 8 ms for 125Hz
#define MAX_CAN_AGE                 0.1
#define FlexiF_threshold 0.20 //Thresholdvalue Front Sensor
#define FlexiR_threshold 0.15 //Thresholdvalue Rear Sensor
#define IMU_threshold 11	//maximum acceleration measured by IMU ? -> Master Study p.46 ? (added by GOF)
#define DebugPrints true     //Toggle Debug Prints on or of

// Eastboard thread
static THD_FUNCTION(eastboard_thread, arg);
static THD_WORKING_AREA(eastboard_thread_wa, 2048); // 2kb stack for this thread

// Private variables

//Private Functions 
void app_eastboard_init(void) {
	//Put init operations here, like initialising pins etc.

	// Start the Eastboard thread
	chThdCreateStatic(eastboard_thread_wa, sizeof(eastboard_thread_wa),
		NORMALPRIO, eastboard_thread, NULL);
}

float getmagnitude(float x,float y,float z){
  static float mag=0;
  mag = sqrt(SQ(x)+SQ(y)+SQ(z));
  return mag;
}

// Problems with Types, result always 0 ....
// Workaround with LowPass
float movingAvg(float *ptrArrNumbers, long *ptrSum, int pos, int len, float nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}
 
static THD_FUNCTION(eastboard_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_Eastboard");
    bool was_cc = false;

	for(;;) {
		chThdSleepMilliseconds(OUTPUT_ITERATION_TIME_MS);
        // Reset the timeout
		timeout_reset();

        static float rpm_filtered = 0.0, ff1_filtered = 0.0, ff2_filtered =0.0;
		static float magn_Accl=0.0, magn_Accl_filtered=0;
		static float Accel[3];
        
		// Read the external ADC pin and convert the value to a voltage. 
        // Vref = 3.3V
		float ff1 = (float)ADC_Value[ADC_IND_EXT];
		ff1 /= 4096.0; //normalize: 2^12bit ADC = 4096
        UTILS_LP_FAST(ff1_filtered,ff1,0.4);
		float ff2 = (float)ADC_Value[ADC_IND_EXT2];
		ff2 /= 4096.0;
        UTILS_LP_FAST(ff2_filtered,ff2,0.4);

		//Read current IMU Values and calculate Magnitude
		imu_get_accel(Accel);
		magn_Accl = getmagnitude(Accel[0],Accel[1],Accel[2]);
        //Use a fast LowPass Filter 
		UTILS_LP_FAST(magn_Accl_filtered, magn_Accl, 0.25);

        if (!was_cc) {
			//If the Cruise Control is not activated get new RPM Value
            UTILS_LP_FAST(rpm_filtered, mc_interface_get_rpm(), 0.5);
        }
        #if DebugPrints
        if (magn_Accl_filtered>IMU_threshold){
            commands_printf("Mangitude is: %.2f",magn_Accl_filtered);
        }
        #endif
        
        if (ff1_filtered>FlexiF_threshold && ff2_filtered>FlexiR_threshold && magn_Accl_filtered < IMU_threshold) {
			// If FrontFlexiForce and RearFlexiForce are over the threshold, 
			//run the motor with speed control at the current RPM (cruise control)
            #if DebugPrints
		    if (!was_cc){
				commands_printf("Started with ff1: %f \t ff2: %f \t RPM: %f",ff1_filtered,ff2_filtered,rpm_filtered);
			} 
            #endif
            was_cc = true; 
			mc_interface_set_pid_speed(rpm_filtered);                   
			

            float current = mc_interface_get_tot_current_directional_filtered();

            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    comm_can_set_current(msg->id, current);
                }
            }
		}
		else{
            // If the CC is not active, release the motor
            // sets current to 0 (coasting)
            mc_interface_release_motor();
            // Send the same current to the other controllers
            float current = mc_interface_get_tot_current_directional_filtered();

            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    comm_can_set_current(msg->id, current);
                }
            }
            #if DebugPrints
            if (was_cc) {
            commands_printf("Ended with ffs1: %f \t ffs2: %f \n",ff1_filtered,ff2_filtered);
            }
            #endif
            was_cc = false;
		}

	}
}
