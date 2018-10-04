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

#ifndef INVENSENSE_MODM_MPU_DEVICE_HPP
#define INVENSENSE_MODM_MPU_DEVICE_HPP

#include "mpu_base.hpp"
#include <invensense/modm.hpp>
#include <modm/platform.hpp>

namespace modm
{

template< class I2cMaster, class Int>
class MpuDevice : public MpuBase
{
public:
	static inline void
	configure()
	{
		Int::setInput();
		Int::setInputTrigger(modm::platform::Gpio::InputTrigger::RisingEdge);
		Int::enableExternalInterrupt();
		Int::enableExternalInterruptVector(12);

		invensense::emd::set_i2c_handler([](modm::I2cTransaction *t)
		{
			return I2cMaster::start(t);
		});
		MpuBase::configure();
	}
};

}

#endif // INVENSENSE_MODM_MPU_DEVICE_HPP