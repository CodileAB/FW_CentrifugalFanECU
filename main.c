#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include <FREERTOS/task.h>

#include "Tasks/tasks.h"
#include "Tasks/MotorManager.h"
#include "Tasks/MessageManager.h"
#include "CAN_Config.h"


void Handle_FanControl_Received(FanControl_t msg, uint8_t from_node_id, uint8_t to_node_id)
{
	MotorDirection_t motor_direction;
	uint8_t motor_duty_cycle;

	if (NODE_ID != to_node_id) {
		return;
	}

	motor_duty_cycle = msg.DutyCycle;

	switch (msg.Direction) {
	case 2U:
		motor_direction = DIR_FORWARD;
		break;
	case 1U:
		motor_direction = DIR_BACKWARD;
		break;
	case 0U:
	default:
		motor_direction = DIR_NONE;
		break;
	}

	if (DIR_NONE == motor_direction) {
		MotorManager_Stop();
	} else {
		MotorManager_SetParameters(motor_duty_cycle, motor_direction, msg.Revolutions);
	}
}

void Task_Main(void *pvParameters)
{
	FanStatus_t fan_status;
	MotorStatus_t motor_status;
	MotorParameters_t requested_motor_params;
	MotorDiagnosis_t motor_diag;
	uint16_t rpm, temperature;

	while (1U) {
		MotorManager_GetStatus(&motor_status);
		MotorManager_GetDiagnosis(&motor_diag);
		MotorManager_GetTemperature(&temperature);
		MotorManager_GetRPM(&rpm);
		MotorManager_GetRequestedParameters(&requested_motor_params);

		fan_status.RevolutionsPerMinute = rpm;
		fan_status.DutyCycle = requested_motor_params.duty_cycle;
		fan_status.Direction = requested_motor_params.direction;
		fan_status.Temperature = (uint8_t) temperature;
		fan_status.Status = motor_status;

		fan_status.OvertemperatureShutdown = motor_diag.OvertemperatureShutdown;
		fan_status.CurrentLimitation = motor_diag.CurrentLimitation;
		fan_status.ShortCircuitCode = motor_diag.ShortCircuitCode;
		fan_status.OpenLoad = motor_diag.OpenLoad;
		fan_status.Undervoltage = motor_diag.Undervoltage;

		MessageManager_Send_FanStatus(&fan_status, 0x00);

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

int main(void)
{
	DAVE_STATUS_t status;

	status = DAVE_Init();           /* Initialization of DAVE APPs  */

	if (status != DAVE_STATUS_SUCCESS)
	{
		/* Placeholder for error handler code. The while loop below can be replaced with an user error handler. */
		XMC_DEBUG("DAVE APPs initialization failed\n");

		while(1U)
		{

		}
	}

	/* Initiate tasks */
	MotorManager_Init();
	MessageManager_Init();
	xTaskCreate(Task_Main, "Task_Main", 40U, NULL, (tskIDLE_PRIORITY + 2), NULL);

	vTaskStartScheduler();

	/* Placeholder for user application code. The while loop below can be replaced with user application code. */
	while (1U)
	{

	}
}
