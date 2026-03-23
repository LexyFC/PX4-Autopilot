/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file ina226.h
 *
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/ina_common/ina_common.h>

using namespace time_literals;

/* INA226-specific constants */
#define INA226_BASEADDR                          0x41 /* 7-bit address */
#define INA226_MAX_CURRENT                       164.0f   /* 164 Amps */
#define INA226_SHUNT                             0.0005f  /* Shunt is 500 uOhm */
#define INA226_DEFAULT_CONFIG  (INA_COMMON_MODE_SHUNT_BUS_CONT | INA_COMMON_VSHCT_588US | INA_COMMON_VBUSCT_588US | INA_COMMON_AVERAGES_64)

/* INA226-specific probe registers */
#define INA226_MFG_ID                            (0xfe)
#define INA226_MFG_DIEID                         (0xff)
#define INA226_MFG_ID_TI                         (0x5449) // TI
#define INA226_MFG_DIE                           (0x2260) // INA2260

class INA226 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA226>
{
public:
	INA226(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~INA226();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void	RunImpl();

	int 		  init() override;

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * INA_COMMON_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
	 *
	 * @return PX4_OK if initialization succeeded on the first try. Negative value otherwise.
	 */
	int force_init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				      print_status() override;

protected:
	int	  		probe() override;

private:
	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t 		_collection_errors;
	perf_counter_t 		_measure_errors;

	unsigned              _measure_interval{0};
	bool                  _collect_phase{false};

	Battery 		  _battery;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	INACommon             _common;

	static int i2c_transfer_wrapper(void *context, const uint8_t *send, unsigned send_len,
					uint8_t *recv, unsigned recv_len)
	{
		return static_cast<INA226 *>(context)->transfer(send, send_len, recv, recv_len);
	}

	/**
	* Initialise the automatic measurement state machine and start it.
	*/
	void				      start();

};
