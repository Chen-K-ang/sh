/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <commander/px4_custom_mode.h>

#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/rc_channels.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>


using namespace time_literals;

static constexpr uint32_t ENGINE_SCHEDULE_INTERVAL{200_ms};    /**< The schedule interval in usec (5 Hz) */


class EngineControl : public ModuleBase<EngineControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EngineControl();
	~EngineControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Parameters
	// DEFINE_PARAMETERS(
	// 	(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
	// 	(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	// )
	
	const float fixed_rpm{5600.0};
	float engine_ctl_trim_f{0};
	const float rc_engine_offset{0.03};
	float ramp_time_f{0};
		
	bool _begin_ctl{false};

	float _no_rpm_time{5.1f};
	float _not_in_ctl_time{0};

	float _start_rpm{0};
	float _target_rpm{0};
	float _delta_rpm{0};

	float engine_ctl_output{-1};
	float engine_ctl_start{-0.63};
	float engine_ctl_offset{0.0f};

	float _ctl_rpm_i{0.0};
	float _ctl_rpm_p{0.0};
	float _ctl_rpm_d{0};
	float _ctl_rpm_cmp{0};
	float _rpm_last{0};
	float _ctl_rpm_p_last{0};

	float rpm_record[4] = {};

	float _ctl_param_p_f{0};
	float _ctl_param_i_f{0};
	float _ctl_param_d_f{0};
	float _ctl_param_d2_f{0};
	float _ctl_param_offset_f{0};

	hrt_abstime _last_time{0};
	hrt_abstime _ctl_start_time{0};

	rpm_s rpm_pcf8583{0};
	vehicle_status_s   vehicle_status;
	rc_channels_s rc_input;
	vehicle_land_detected_s _vehicle_land_detected;

	actuator_controls_s  _actuator_controls_2{0};
	actuator_controls_s  _actuator_controls_0{0};
	float _thrust_ctl_old{0};
	float _thrust_ctl{0};

	// Subscriptions	
		uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
		uORB::Subscription 		   	_actuator_controls_sub{ORB_ID(actuator_controls_0)};
		uORB::Subscription 		   	_rpm_update_sub{ORB_ID(rpm)};
        uORB::Subscription                 	_vehicle_status_sub{ORB_ID(vehicle_status)};
        uORB::Subscription                 	_rc_channels_sub{ORB_ID(rc_channels)};
        uORB::SubscriptionInterval 		_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	
	// Publications
        uORB::Publication<actuator_controls_s>    _actuator_controls_2_pub{ORB_ID(actuator_controls_2)};

	orb_advert_t _mavlink_log_pub{nullptr};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params:: _ECTL_TRIM>) _param_ctl_trim,
		(ParamFloat<px4::params::RAMP_TIME>) _param_ramp_time,
		(ParamFloat<px4::params::_ECTL_P>) _ctl_param_p,
		(ParamFloat<px4::params::_ECTL_I>) _ctl_param_i,
		(ParamFloat<px4::params::_ECTL_D>) _ctl_param_d,
		(ParamFloat<px4::params::_ECTL_DOFFSET>) _ctl_param_offset,
		(ParamFloat<px4::params::_ECTL_SACL_UP>) _ctl_param_scal_up
	)


};
