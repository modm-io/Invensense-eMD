/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    Copyright (c) 2018, Niklas Hauser
    See included License.txt for License information.
 $
 */
// ----------------------------------------------------------------------------

#include <modm/board.hpp>
#include <invensense/modm.hpp>
#include <invensense/modm/driver/mpu_device.hpp>

using namespace Board;

extern "C" void inv_get_ms(uint32_t *count);

namespace imu
{
using Int = Board::D12;
using Scl = Board::D15;
using Sda = Board::D14;
using Master = I2cMaster1;

using Device = modm::MpuDevice<I2cMaster1, Int>;
}

void board_init()
{
	Board::initialize();
	LedD13::setOutput();

	imu::Master::connect<imu::Sda::Sda, imu::Scl::Scl>();
	imu::Master::initialize<Board::SystemClock, 400_kHz>();

	invensense::emd::set_uart_handler([](uint8_t c)
	{
		Board::stlink::Uart::write(c);
		return true;
	});

	imu::Device::configure();
}

MODM_ISR(EXTI9_5)
{
	if (imu::Int::getExternalInterruptFlag())
	{
		imu::Device::interrupt();
		imu::Int::acknowledgeExternalInterruptFlag();
	}
}

/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

struct hal_s {
	unsigned char lp_accel_mode;
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned char motion_int_mode;
	unsigned long no_dmp_hz;
	unsigned long next_pedo_ms;
	unsigned long next_temp_ms;
	unsigned long next_compass_ms;
	unsigned int report;
	unsigned short dmp_features;
};
extern struct hal_s hal;

#define COMPASS_ENABLED 1


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void)
{
	long msg, data[9];
	int8_t accuracy;
	unsigned long timestamp;
	float float_data[3] = {0};

	// imu::Device::Quaternion quad = imu::Device::getQuaternion();
	// MPL_LOGI("%lu ms, %u, %u: %2.2f %2.2f %2.2f %2.2f\n",
	//          quad.time().getTime(), quad.accuracy(), quad.is_new(),
	//          quad.w, quad.x, quad.y, quad.z);
	imu::Device::Heading head = imu::Device::getHeading();
	MPL_LOGI("%lu ms, %u, %u: %7.3f\n",
	         head.time().getTime(), head.accuracy(), head.is_new(),
	         head.heading);

	if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
	   /* Sends a quaternion packet to the PC. Since this is used by the Python
		* test app to visually represent a 3D quaternion, it's sent each time
		* the MPL has new data.
		*/
		eMPL_send_quat(data);

		/* Specific data packets can be sent or suppressed using USB commands. */
		if (hal.report & PRINT_QUAT)
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}

	if (hal.report & PRINT_ACCEL) {
		if (inv_get_sensor_type_accel(data, &accuracy,
			(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_ACCEL, data);
	}
	if (hal.report & PRINT_GYRO) {
		if (inv_get_sensor_type_gyro(data, &accuracy,
			(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_GYRO, data);
	}
#ifdef COMPASS_ENABLED
	if (hal.report & PRINT_COMPASS) {
		if (inv_get_sensor_type_compass(data, &accuracy,
			(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_COMPASS, data);
	}
#endif
	if (hal.report & PRINT_EULER) {
		if (inv_get_sensor_type_euler(data, &accuracy,
			(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_EULER, data);
	}
	if (hal.report & PRINT_ROT_MAT) {
		if (inv_get_sensor_type_rot_mat(data, &accuracy,
			(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_ROT, data);
	}
	if (hal.report & PRINT_HEADING) {
		if (inv_get_sensor_type_heading(data, &accuracy,
			(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_HEADING, data);
	}
	if (hal.report & PRINT_LINEAR_ACCEL) {
		if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
			MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
					float_data[0], float_data[1], float_data[2]);
		 }
	}
	if (hal.report & PRINT_GRAVITY_VECTOR) {
			if (inv_get_sensor_type_gravity(float_data, &accuracy,
				(inv_time_t*)&timestamp))
				MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
						float_data[0], float_data[1], float_data[2]);
	}
	if (hal.report & PRINT_PEDO) {
		inv_get_ms(&timestamp);
		if (timestamp > hal.next_pedo_ms) {
			hal.next_pedo_ms = timestamp + PEDO_READ_MS;
			unsigned long step_count, walk_time;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);
			MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
			walk_time);
		}
	}

	/* Whenever the MPL detects a change in motion state, the application can
	 * be notified. For this example, we use an LED to represent the current
	 * motion state.
	 */
	msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
			INV_MSG_NO_MOTION_EVENT);
	if (msg) {
		if (msg & INV_MSG_MOTION_EVENT) {
			MPL_LOGI("Motion!\n");
		} else if (msg & INV_MSG_NO_MOTION_EVENT) {
			MPL_LOGI("No motion!\n");
		}
	}
}

#ifdef COMPASS_ENABLED
void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("Accuracy= %d\r\n", accuracy);

}
#endif


static void tap_cb(imu::Device::TapDirection direction, unsigned char count)
{
	switch (direction) {
	case imu::Device::TapDirection::XUp:
		MPL_LOGI("Tap X+ ");
		break;
	case imu::Device::TapDirection::XDown:
		MPL_LOGI("Tap X- ");
		break;
	case imu::Device::TapDirection::YUp:
		MPL_LOGI("Tap Y+ ");
		break;
	case imu::Device::TapDirection::YDown:
		MPL_LOGI("Tap Y- ");
		break;
	case imu::Device::TapDirection::ZUp:
		MPL_LOGI("Tap Z+ ");
		break;
	case imu::Device::TapDirection::ZDown:
		MPL_LOGI("Tap Z- ");
		break;
	default:
		return;
	}
	MPL_LOGI("x%d\n", count);
	return;
}

static void android_orient_cb(imu::Device::Orientation orientation)
{
	switch (orientation) {
	case imu::Device::Orientation::Portrait:
		MPL_LOGI("Portrait\n");
		break;
	case imu::Device::Orientation::Landscape:
		MPL_LOGI("Landscape\n");
		break;
	case imu::Device::Orientation::ReversePortrait:
		MPL_LOGI("Reverse Portrait\n");
		break;
	case imu::Device::Orientation::ReverseLandscape:
		MPL_LOGI("Reverse Landscape\n");
		break;
	default:
		return;
	}
}


static inline void run_self_test(void)
{
	int result = imu::Device::calibrate();
	if (!(result & 0x1)) {
		MPL_LOGE("Gyro failed.\n");
	}
	if (!(result & 0x2)) {
		MPL_LOGE("Accel failed.\n");
	}
	if (!(result & 0x4)) {
		MPL_LOGE("Compass failed.\n");
	}
}

static void handle_input(void)
{

	uint8_t c;
	if (not Board::stlink::Uart::read(c))
		return;

	switch (c) {
	/* These commands turn off individual sensors. */
	case '8':
		imu::Device::enableAccelerometer(!(hal.sensors & ACCEL_ON));
		break;
	case '9':
		imu::Device::enableGyroscope(!(hal.sensors & GYRO_ON));
		break;
#ifdef COMPASS_ENABLED
	case '0':
		imu::Device::enableCompass(!(hal.sensors & COMPASS_ON));
		break;
#endif
	/* The commands send individual sensor data or fused data to the PC. */
	case 'a':
		hal.report ^= PRINT_ACCEL;
		break;
	case 'g':
		hal.report ^= PRINT_GYRO;
		break;
#ifdef COMPASS_ENABLED
	case 'c':
		hal.report ^= PRINT_COMPASS;
		break;
#endif
	case 'e':
		hal.report ^= PRINT_EULER;
		break;
	case 'r':
		hal.report ^= PRINT_ROT_MAT;
		break;
	case 'q':
		hal.report ^= PRINT_QUAT;
		break;
	case 'h':
		hal.report ^= PRINT_HEADING;
		break;
	case 'i':
		hal.report ^= PRINT_LINEAR_ACCEL;
		break;
	case 'o':
		hal.report ^= PRINT_GRAVITY_VECTOR;
		break;
#ifdef COMPASS_ENABLED
	case 'w':
		send_status_compass();
		break;
#endif
	/* This command prints out the value of each gyro register for debugging.
	 * If logging is disabled, this function has no effect.
	 */
	case 'd':
		mpu_reg_dump();
		break;
	/* Test out low-power accel mode. */
	case 'p':
		if (hal.dmp_on)
			/* LP accel is not compatible with the DMP. */
			break;
		mpu_lp_accel_mode(20);
		/* When LP accel mode is enabled, the driver automatically configures
		 * the hardware for latched interrupts. However, the MCU sometimes
		 * misses the rising/falling edge, and the hal.new_gyro flag is never
		 * set. To avoid getting locked in this state, we're overriding the
		 * driver's configuration and sticking to unlatched interrupt mode.
		 *
		 * TODO: The MCU supports level-triggered interrupts.
		 */
		mpu_set_int_latched(0);
		hal.sensors &= ~(GYRO_ON|COMPASS_ON);
		hal.sensors |= ACCEL_ON;
		hal.lp_accel_mode = 1;
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
	/* The hardware self test can be run without any interaction with the
	 * MPL since it's completely localized in the gyro driver. Logging is
	 * assumed to be enabled; otherwise, a couple LEDs could probably be used
	 * here to display the test results.
	 */
	case 't':
		run_self_test();
		break;
	/* Depending on your application, sensor data may be needed at a faster or
	 * slower rate. These commands can speed up or slow down the rate at which
	 * the sensor data is pushed to the MPL.
	 *
	 * In this example, the compass rate is never changed.
	 */
	case '1':
		imu::Device::setSampleRate(10);
		break;
	case '2':
		imu::Device::setSampleRate(20);
		break;
	case '3':
		imu::Device::setSampleRate(40);
		break;
	case '4':
		imu::Device::setSampleRate(50);
		break;
	case '5':
		imu::Device::setSampleRate(100);
		break;
	// case '6':
	// 	imu::Device::setSampleRate(200);
	// 	break;
	case ',':
		/* Set hardware to interrupt on gesture event only. This feature is
		 * useful for keeping the MCU asleep until the DMP detects as a tap or
		 * orientation event.
		 */
		dmp_set_interrupt_mode(DMP_INT_GESTURE);
		break;
	case '.':
		/* Set hardware to interrupt periodically. */
		dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
		break;
	case '6':
		/* Toggle pedometer display. */
		hal.report ^= PRINT_PEDO;
		break;
	case '7':
		/* Reset pedometer. */
		dmp_set_pedometer_step_count(0);
		dmp_set_pedometer_walk_time(0);
		break;
	case 'f':
		if (hal.lp_accel_mode)
			/* LP accel is not compatible with the DMP. */
			return;
		/* Toggle DMP. */
		imu::Device::enableDmp(!(hal.dmp_on));
		break;
	case 'm':
		/* Test the motion interrupt hardware feature. */
	#ifndef MPU6050 // not enabled for 6050 product
	hal.motion_int_mode = 1;
	#endif
		break;

	case 'v':
		/* Toggle LP quaternion.
		 * The DMP features can be enabled/disabled at runtime. Use this same
		 * approach for other features.
		 */
		hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
		dmp_enable_feature(hal.dmp_features);
		if (!(hal.dmp_features & DMP_FEATURE_6X_LP_QUAT)) {
			inv_quaternion_sensor_was_turned_off();
			MPL_LOGI("LP quaternion disabled.\n");
		} else
			MPL_LOGI("LP quaternion enabled.\n");
		break;
	default:
		break;
	}
}

/*******************************************************************************/
int main(void)
{
	board_init();
	imu::Device::configure();
	imu::Device::setTapHandler(tap_cb);
	imu::Device::setOrientationHandler(android_orient_cb);

    while(1)
    {
		handle_input();

		imu::Device::update();

		if (imu::Device::execute())
		{
			read_from_mpl();
			LedD13::toggle();
		}
	}
}
