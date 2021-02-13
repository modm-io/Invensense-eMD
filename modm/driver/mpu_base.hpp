/*
 * Copyright (c) 2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef INVENSENSE_MODM_MPU_BASE_HPP
#define INVENSENSE_MODM_MPU_BASE_HPP

#include <invensense/eMD.hpp>
#include <modm/math/geometry/quaternion.hpp>
#include <modm/processing/timer/timestamp.hpp>

namespace modm
{

struct mpu_base
{
public:
	enum class
	TapDirection : uint8_t
	{
		XUp   = TAP_X_UP,
		XDown = TAP_X_DOWN,
		YUp   = TAP_Y_UP,
		YDown = TAP_Y_DOWN,
		ZUp   = TAP_Z_UP,
		ZDown = TAP_Z_DOWN,
	};

	enum class
	Orientation : uint8_t
	{
		Portrait = ANDROID_ORIENT_PORTRAIT,
		Landscape = ANDROID_ORIENT_LANDSCAPE,
		ReversePortrait = ANDROID_ORIENT_REVERSE_PORTRAIT,
		ReverseLandscape = ANDROID_ORIENT_REVERSE_LANDSCAPE,
	};

	using tap_handler_t = void(*)(TapDirection direction, uint8_t count);
	using orientation_handler_t = void(*)(Orientation orientation);

public:
	class Quaternion : public modm::Quaternion<float>
	{
		friend class MpuBase;
		uint8_t new_accuracy{0};
		modm::Timestamp timestamp;
	public:
		using modm::Quaternion<float>::Quaternion;
		bool is_new() const { return this->new_accuracy & 0x80; }
		uint8_t accuracy() const { return this->new_accuracy & 0x3; }
		modm::Timestamp time() const { return this->timestamp; }
	};

	class Heading
	{
		friend class MpuBase;
		uint8_t new_accuracy{0};
		modm::Timestamp timestamp;
	public:
		float heading;
		bool is_new() const { return this->new_accuracy & 0x80; }
		uint8_t accuracy() const { return this->new_accuracy & 0x3; }
		modm::Timestamp time() const { return this->timestamp; }
	};
};

/**
 * - MPU6050
 * - MPU6500
 * - MPU9150
 * - MPU9250
 */
class MpuBase : public mpu_base
{
public:
	static void
	configure();

	static int
	calibrate();

	static Quaternion
	getQuaternion();

	static Heading
	getHeading();

public:
	static void
	setTapHandler(tap_handler_t handler);

	static void
	setOrientationHandler(orientation_handler_t handler);

	static void
	setSampleRate(uint8_t hz);

	static void
	enableAccelerometer(bool enable);

	static void
	enableGyroscope(bool enable);

	static void
	enableCompass(bool enable);

	static void
	enableDmp(bool enable);

public:
	static void
	interrupt();

public:
	static void
	update();

	static bool
	execute();
};

}

#endif // INVENSENSE_MODM_MPU_BASE_HPP
