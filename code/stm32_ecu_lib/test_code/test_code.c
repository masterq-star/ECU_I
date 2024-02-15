
/*
 *
 * This test code file simply sets the analogue sensors to a pre-set level.
 *
 *
 */



#include "global.h"

#if DIAGNOSTIC_MODE == 1


#include "test_code.h"
#include "scheduler.h"
#include "cfg_data.h"
#include "auto_idle.h"
#include "cyclic_tasks.h"
#include "ecu_services.h"
#include "trigger_wheel_handler.h"


// prototypes
void testCyclic(void);
void testEvent(void);


/*
 * testCodeInitialise() is called at the end of ecuInitialisation() in ecu_main
 *
 */
void testCodeInitialise(){
	keyData.v.TPSVoltage = 500.0F;
	keyData.v.TPS = 20.0F;
	keyData.v.MAP = 80.0F;
	keyData.v.lambdaVoltage = 750.0F;
	keyData.v.airTemperature = 25.0F;
	keyData.v.coolantTemperature = 10.0F;
	keyData.v.voltage2 = 0.0F;
}


/*
 * testCodeLoop() is called from ecuLoop() in ecu_main
 *
 */
void testCodeLoop(){

}


#endif

