/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    Copyright (c) 2018, Niklas Hauser
    See included License.txt for License information.
 $
 */
// ----------------------------------------------------------------------------

#include "mpu_base.hpp"
#include <modm/architecture/interface/assert.hpp>

extern "C" void inv_get_ms(uint32_t *count);

bool modm_new_data{false};

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
struct hal_s hal = {0};

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
	signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
	.orientation = { 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
	.orientation = { 0, 1, 0,
					 1, 0, 0,
					 0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
	.orientation = {-1, 0, 0,
					 0, 1, 0,
					 0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
	.orientation = {-1, 0, 0,
					 0,-1, 0,
					 0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
	unsigned char mask = 0, lp_accel_was_on = 0;
	if (hal.sensors & ACCEL_ON)
		mask |= INV_XYZ_ACCEL;
	if (hal.sensors & GYRO_ON) {
		mask |= INV_XYZ_GYRO;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
#ifdef COMPASS_ENABLED
	if (hal.sensors & COMPASS_ON) {
		mask |= INV_XYZ_COMPASS;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
#endif
	/* If you need a power transition, this function should be called with a
	 * mask of the sensors still enabled. The driver turns off any sensors
	 * excluded from this mask.
	 */
	mpu_set_sensors(mask);
	mpu_configure_fifo(mask);
	if (lp_accel_was_on) {
		unsigned short rate;
		hal.lp_accel_mode = 0;
		/* Switching out of LP accel, notify MPL of new accel sampling rate. */
		mpu_get_sample_rate(&rate);
		inv_set_accel_sample_rate(1000000L / rate);
	}
}

modm::MpuBase::Quaternion
modm::MpuBase::getQuaternion()
{
	int32_t data[4];
	int8_t accuracy;
	inv_time_t timestamp;
	int is_new = inv_get_sensor_type_quat(data, &accuracy, &timestamp);

	auto quad = Quaternion(inv_q30_to_float(data[0]), inv_q30_to_float(data[1]),
						   inv_q30_to_float(data[2]), inv_q30_to_float(data[3]));
	quad.new_accuracy = accuracy;
	quad.timestamp = modm::Timestamp(modm::Duration(timestamp));
	if (is_new) quad.new_accuracy |= 0x80;
	return quad;
}

modm::MpuBase::Heading
modm::MpuBase::getHeading()
{
	int32_t data;
	int8_t accuracy;
	inv_time_t timestamp;
	int is_new = inv_get_sensor_type_heading(&data, &accuracy, &timestamp);

	auto head = Heading();
	head.heading = data / 65536.f;
	head.new_accuracy = accuracy;
	head.timestamp = modm::Timestamp(modm::Duration(timestamp));
	if (is_new) head.new_accuracy |= 0x80;
	return head;
}

void
modm::MpuBase::setTapHandler(tap_handler_t handler)
{
	dmp_register_tap_cb((void (*)(unsigned char, unsigned char)) handler);
}

void
modm::MpuBase::setOrientationHandler(orientation_handler_t handler)
{
	dmp_register_android_orient_cb((void (*)(unsigned char)) handler);
}

void
modm::MpuBase::setSampleRate(uint8_t hz)
{
	const uint32_t rate = 1'000'000ul / hz;
	if (hal.dmp_on) {
		dmp_set_fifo_rate(hz);
		inv_set_quat_sample_rate(rate);
	} else {
		mpu_set_sample_rate(hz);
	}
	inv_set_gyro_sample_rate(rate);
	inv_set_accel_sample_rate(rate);
}

void
modm::MpuBase::enableAccelerometer(bool enable)
{
	if (enable) {
		hal.sensors |= ACCEL_ON;
	} else {
		hal.sensors &= ~ACCEL_ON;
	}
	setup_gyro();
	if (!(hal.sensors & ACCEL_ON))
		inv_accel_was_turned_off();
}

void
modm::MpuBase::enableGyroscope(bool enable)
{
	if (enable) {
		hal.sensors |= GYRO_ON;
	} else {
		hal.sensors &= ~GYRO_ON;
	}
	setup_gyro();
	if (!(hal.sensors & GYRO_ON))
		inv_gyro_was_turned_off();
}

void
modm::MpuBase::enableCompass(bool enable)
{
	if (enable) {
		hal.sensors |= COMPASS_ON;
	} else {
		hal.sensors &= ~COMPASS_ON;
	}
	setup_gyro();
	if (!(hal.sensors & COMPASS_ON))
		inv_compass_was_turned_off();
}

void
modm::MpuBase::enableDmp(bool enable)
{
	if (!enable) {
		unsigned short dmp_rate;
		unsigned char mask = 0;
		hal.dmp_on = 0;
		mpu_set_dmp_state(0);
		/* Restore FIFO settings. */
		if (hal.sensors & ACCEL_ON)
			mask |= INV_XYZ_ACCEL;
		if (hal.sensors & GYRO_ON)
			mask |= INV_XYZ_GYRO;
		if (hal.sensors & COMPASS_ON)
			mask |= INV_XYZ_COMPASS;
		mpu_configure_fifo(mask);
		/* When the DMP is used, the hardware sampling rate is fixed at
		 * 200Hz, and the DMP is configured to downsample the FIFO output
		 * using the function dmp_set_fifo_rate. However, when the DMP is
		 * turned off, the sampling rate remains at 200Hz. This could be
		 * handled in inv_mpu.c, but it would need to know that
		 * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
		 * put the extra logic in the application layer.
		 */
		dmp_get_fifo_rate(&dmp_rate);
		mpu_set_sample_rate(dmp_rate);
		inv_quaternion_sensor_was_turned_off();
		MPL_LOGI("DMP disabled.\n");
	} else {
		unsigned short sample_rate;
		hal.dmp_on = 1;
		/* Preserve current FIFO rate. */
		mpu_get_sample_rate(&sample_rate);
		dmp_set_fifo_rate(sample_rate);
		inv_set_quat_sample_rate(1000000L / sample_rate);
		mpu_set_dmp_state(1);
		MPL_LOGI("DMP enabled.\n");
	}
}

void
modm::MpuBase::configure()
{
	inv_error_t result;
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	struct int_param_s int_param;

#ifdef COMPASS_ENABLED
	unsigned short compass_fsr;
#endif

	result = mpu_init(&int_param);
	modm_assert(result == 0, "mpu.gyro", "Gyro initialization", result);

	/* If you're not using an MPU9150 AND you're not using DMP features, this
	 * function will place all slaves on the primary bus.
	 * mpu_set_bypass(1);
	 */
	result = inv_init_mpl();
	modm_assert(result == 0, "mpu.mpl", "MPL initialization", result);

	/* Compute 6-axis and 9-axis quaternions. */
	inv_enable_quaternion();
	inv_enable_9x_sensor_fusion();
	/* The MPL expects compass data at a constant rate (matching the rate
	 * passed to inv_set_compass_sample_rate). If this is an issue for your
	 * application, call this function, and the MPL will depend on the
	 * timestamps passed to inv_build_compass instead.
	 *
	 * inv_9x_fusion_use_timestamps(1);
	 */

	/* This function has been deprecated.
	 * inv_enable_no_gyro_fusion();
	 */

	/* Update gyro biases when not in motion.
	 * WARNING: These algorithms are mutually exclusive.
	 */
	inv_enable_fast_nomot();
	/* inv_enable_motion_no_motion(); */
	/* inv_set_no_motion_time(1000); */

	/* Update gyro biases when temperature changes. */
	inv_enable_gyro_tc();

	/* This algorithm updates the accel biases when in motion. A more accurate
	 * bias measurement can be made when running the self-test (see case 't' in
	 * handle_input), but this algorithm can be enabled if the self-test can't
	 * be executed in your application.
	 *
	 * inv_enable_in_use_auto_calibration();
	 */
#ifdef COMPASS_ENABLED
	/* Compass calibration algorithms. */
	inv_enable_vector_compass_cal();
	inv_enable_magnetic_disturbance();
#endif
	/* If you need to estimate your heading before the compass is calibrated,
	 * enable this algorithm. It becomes useless after a good figure-eight is
	 * detected, so we'll just leave it out to save memory.
	 * inv_enable_heading_from_gyro();
	 */

	/* Allows use of the MPL APIs in read_from_mpl. */
	inv_enable_eMPL_outputs();

	result = inv_start_mpl();
	modm_assert(result != INV_ERROR_NOT_AUTHORIZED, "mpu.unauthorized", "Wrong MPL key", result);
	modm_assert(result == 0, "mpu.start", "MPL failed to start", result);

	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
#ifdef COMPASS_ENABLED
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
	/* Push both gyro and accel data into the FIFO. */
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
	/* The compass sampling rate can be less than the gyro/accel sampling rate.
	 * Use this function for proper power management.
	 */
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
	/* Read back configuration in case it was set improperly. */
	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
	mpu_get_compass_fsr(&compass_fsr);
#endif
	/* Sync driver configuration with MPL. */
	/* Sample rate expected in microseconds. */
	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
	/* The compass rate is independent of the gyro and accel rates. As long as
	 * inv_set_compass_sample_rate is called with the correct value, the 9-axis
	 * fusion algorithm's compass correction gain will work properly.
	 */
	inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
	/* Set chip-to-body orientation matrix.
	 * Set hardware units to dps/g's/degrees scaling factor.
	 */
	inv_set_gyro_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)gyro_fsr<<15);
	inv_set_accel_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
	inv_set_compass_orientation_and_scale(
			inv_orientation_matrix_to_scalar(compass_pdata.orientation),
			(long)compass_fsr<<15);
#endif
	/* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
	hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
	hal.sensors = ACCEL_ON | GYRO_ON;
#endif
	hal.dmp_on = 0;
	hal.next_pedo_ms = 0;
	hal.next_compass_ms = 0;
	hal.next_temp_ms = 0;

	/* To initialize the DMP:
	 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
	 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
	 * 2. Push the gyro and accel orientation matrix to the DMP.
	 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
	 *    executed unless the corresponding feature is enabled.
	 * 4. Call dmp_enable_feature(mask) to enable different features.
	 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	 * 6. Call any feature-specific control functions.
	 *
	 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
	 * be called repeatedly to enable and disable the DMP at runtime.
	 *
	 * The following is a short summary of the features supported in the DMP
	 * image provided in inv_mpu_dmp_motion_driver.c:
	 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	 * 200Hz. Integrating the gyro data at higher rates reduces numerical
	 * errors (compared to integration on the MCU at a lower sampling rate).
	 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
	 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
	 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
	 * an event at the four orientations where the screen should rotate.
	 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
	 * no motion.
	 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
	 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
	 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
	 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
	 */
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(
		inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	/*
	 * Known Bug -
	 * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
	 * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
	 * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
	 * there will be a 25Hz interrupt from the MPU device.
	 *
	 * There is a known issue in which if you do not enable DMP_FEATURE_TAP
	 * then the interrupts will be at 200Hz even if fifo rate
	 * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
	 *
	 * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
	 */
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	mpu_set_dmp_state(1);
	hal.dmp_on = 1;
}

void
modm::MpuBase::update()
{
	unsigned long sensor_timestamp;
	int new_data = 0;
	unsigned char new_temp = 0;
	unsigned long timestamp;
#ifdef COMPASS_ENABLED
	unsigned char new_compass = 0;
#endif

	/* Compass reads are handled by scheduler. */
    inv_get_ms(&timestamp);

#ifdef COMPASS_ENABLED
	/* We're not using a data ready interrupt for the compass, so we'll
	 * make our compass reads timer-based instead.
	 */
	if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
		hal.new_gyro && (hal.sensors & COMPASS_ON)) {
		hal.next_compass_ms = timestamp + COMPASS_READ_MS;
		new_compass = 1;
	}
#endif
	/* Temperature data doesn't need to be read with every gyro sample.
	 * Let's make them timer-based like the compass reads.
	 */
	if (timestamp > hal.next_temp_ms) {
		hal.next_temp_ms = timestamp + TEMP_READ_MS;
		new_temp = 1;
	}

	if (hal.motion_int_mode) {
		/* Enable motion interrupt. */
		mpu_lp_motion_interrupt(500, 1, 5);
		/* Notify the MPL that contiguity was broken. */
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		inv_quaternion_sensor_was_turned_off();
		/* Wait for the MPU interrupt. */
		while (!hal.new_gyro) {}
		/* Restore the previous sensor configuration. */
		mpu_lp_motion_interrupt(0, 0, 0);
		hal.motion_int_mode = 0;
	}

	if (hal.sensors and hal.new_gyro)
	{
		if (hal.new_gyro && hal.lp_accel_mode) {
			short accel_short[3];
			long accel[3];
			mpu_get_accel_reg(accel_short, &sensor_timestamp);
			accel[0] = (long)accel_short[0];
			accel[1] = (long)accel_short[1];
			accel[2] = (long)accel_short[2];
			inv_build_accel(accel, 0, sensor_timestamp);
			new_data = 1;
			hal.new_gyro = 0;
		}
		else if (hal.new_gyro && hal.dmp_on) {
			short gyro[3], accel_short[3], sensors;
			unsigned char more;
			long accel[3], quat[4], temperature;
			/* This function gets new data from the FIFO when the DMP is in
			 * use. The FIFO can contain any combination of gyro, accel,
			 * quaternion, and gesture data. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
			 * the FIFO isn't being filled with accel data.
			 * The driver parses the gesture data to determine if a gesture
			 * event has occurred; on an event, the application will be notified
			 * via a callback (assuming that a callback function was properly
			 * registered). The more parameter is non-zero if there are
			 * leftover packets in the FIFO.
			 */
			dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
			if (!more)
				hal.new_gyro = 0;
			if (sensors & INV_XYZ_GYRO) {
				/* Push the new data to the MPL. */
				inv_build_gyro(gyro, sensor_timestamp);
				new_data = 1;
				if (new_temp) {
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					mpu_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
			}
			if (sensors & INV_XYZ_ACCEL) {
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
			if (sensors & INV_WXYZ_QUAT) {
				inv_build_quat(quat, 0, sensor_timestamp);
				new_data = 1;
			}
		}
		else if (hal.new_gyro) {
			short gyro[3], accel_short[3];
			unsigned char sensors, more;
			long accel[3], temperature;
			/* This function gets new data from the FIFO. The FIFO can contain
			 * gyro, accel, both, or neither. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
			 * being filled with accel data. The more parameter is non-zero if
			 * there are leftover packets in the FIFO. The HAL can use this
			 * information to increase the frequency at which this function is
			 * called.
			 */
			hal.new_gyro = 0;
			mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
				&sensors, &more);
			if (more)
				hal.new_gyro = 1;
			if (sensors & INV_XYZ_GYRO) {
				/* Push the new data to the MPL. */
				inv_build_gyro(gyro, sensor_timestamp);
				new_data = 1;
				if (new_temp) {
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					mpu_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
			}
			if (sensors & INV_XYZ_ACCEL) {
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
		}
#ifdef COMPASS_ENABLED
		if (new_compass) {
			short compass_short[3];
			long compass[3];
			new_compass = 0;
			/* For any MPU device with an AKM on the auxiliary I2C bus, the raw
			 * magnetometer registers are copied to special gyro registers.
			 */
			if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
				compass[0] = (long)compass_short[0];
				compass[1] = (long)compass_short[1];
				compass[2] = (long)compass_short[2];
				/* NOTE: If using a third-party compass calibration library,
				 * pass in the compass data in uT * 2^16 and set the second
				 * parameter to INV_CALIBRATED | acc, where acc is the
				 * accuracy from 0 to 3.
				 */
				inv_build_compass(compass, 0, sensor_timestamp);
			}
			new_data = 1;
		}
#endif
		if (new_data) {
			inv_execute_on_data();
			modm_new_data = true;
		}
	}
}

bool
modm::MpuBase::execute()
{
	if (modm_new_data) {
		modm_new_data = false;
		return true;
	}
	return false;
}

void
modm::MpuBase::interrupt()
{
	hal.new_gyro = 1;
}

int
modm::MpuBase::calibrate()
{
	int result;
	long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
	result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
	result = mpu_run_self_test(gyro, accel);
#endif
	if (result == 0x7) {
		MPL_LOGI("Passed!\n");
		MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
		         	inv_q16_to_float(accel[0]),
		         	inv_q16_to_float(accel[1]),
		         	inv_q16_to_float(accel[2]));
		MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
					inv_q16_to_float(gyro[0]),
					inv_q16_to_float(gyro[1]),
					inv_q16_to_float(gyro[2]));
		/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
		/*
		 * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
		 * instead of pushing the cal data to the MPL software library
		 */
		unsigned char i = 0;

		for(i = 0; i<3; i++) {
			gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
			accel[i] *= 2048.f; //convert to +-16G
			accel[i] = accel[i] >> 16;
			gyro[i] = (long)(gyro[i] >> 16);
		}

		mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
		mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
		mpu_set_accel_bias_6050_reg(accel);
#endif
#else
		/* Push the calibrated data to the MPL library.
		 *
		 * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
		unsigned short accel_sens;
		float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
	}
	/* Let MPL know that contiguity was broken. */
	inv_accel_was_turned_off();
	inv_gyro_was_turned_off();
	inv_compass_was_turned_off();
	return result;
}
