/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ina226.cpp
 * @author David Sidrane <david_s5@usa.net>
 *
 * Driver for the I2C attached INA226
 *
 * Shared register definitions, I2C read/write, and common measurement logic
 * are provided by the ina_common library (src/lib/drivers/ina_common).
 */

#include "ina226.h"


INA226::INA226(const I2CSPIDriverConfig &config, int battery_index) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina226_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina226_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "ina226_collection_err")),
	_measure_errors(perf_alloc(PC_COUNT, "ina226_measurement_err")),
	_battery(battery_index, this, INA_COMMON_SAMPLE_INTERVAL_US, battery_status_s::SOURCE_POWER_MODULE),
	_common(i2c_transfer_wrapper, this, _battery, _sample_perf, _comms_errors)
{
	_common.loadParams("INA226_CURRENT", "INA226_SHUNT", "INA226_CONFIG",
			   INA226_MAX_CURRENT, INA226_SHUNT, INA226_DEFAULT_CONFIG);

	// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
	_common.setConnected(false);
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

	I2C::_retries = 5;
}

INA226::~INA226()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_measure_errors);
}

int
INA226::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	ret = _common.init();

	start();

	return ret;
}

int
INA226::force_init()
{
	int ret = init();

	start();

	return ret;
}

int
INA226::probe()
{
	int16_t value{0};

	if (_common.read(INA226_MFG_ID, value) != PX4_OK || value != INA226_MFG_ID_TI) {
		PX4_DEBUG("probe mfgid %d", value);
		return -1;
	}

	if (_common.read(INA226_MFG_DIEID, value) != PX4_OK || value != INA226_MFG_DIE) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}

	return PX4_OK;
}

void
INA226::start()
{
	ScheduleClear();

	_collect_phase = false;

	_measure_interval = INA_COMMON_CONVERSION_INTERVAL;

	ScheduleDelayed(5);
}

void
INA226::RunImpl()
{
	if (_common._initialized) {
		if (_collect_phase) {
			if (_parameter_update_sub.updated()) {
				parameter_update_s parameter_update;
				_parameter_update_sub.copy(&parameter_update);
				updateParams();
			}

			if (_common.collect() != PX4_OK) {
				perf_count(_collection_errors);
				start();
				return;
			}

			_collect_phase = !_common._mode_triggered;

			if (_measure_interval > INA_COMMON_CONVERSION_INTERVAL) {
				ScheduleDelayed(_measure_interval - INA_COMMON_CONVERSION_INTERVAL);
				return;
			}
		}

		if (_common.measure() != PX4_OK) {
			perf_count(_measure_errors);
		}

		_collect_phase = true;

		ScheduleDelayed(INA_COMMON_CONVERSION_INTERVAL);

	} else {
		_common.setConnected(false);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

		if (init() != PX4_OK) {
			ScheduleDelayed(INA_COMMON_INIT_RETRY_INTERVAL_US);
		}
	}
}

void
INA226::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_common._initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		printf("poll interval:  %u \n", _measure_interval);

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until battery is plugged in.",
			 INA_COMMON_INIT_RETRY_INTERVAL_US / 1000);
	}
}
