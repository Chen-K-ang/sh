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

/**
 * @file EngineControl.cpp
 *
 * Control and deal with the engine control for fixed rpm
 *
 * @author Yangy
 */

#include "EngineControl.hpp"

/*static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN, const float param3 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}*/


EngineControl::EngineControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

EngineControl::~EngineControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool EngineControl::init()
{
	//Run on fixed interval
	ScheduleOnInterval(ENGINE_SCHEDULE_INTERVAL);
	stm32_gpiowrite(GPIO_ENG_START,true);
	stm32_gpiowrite(GPIO_ENG_STOP,true);
	return true;
}

void EngineControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	const float dt = math::constrain((hrt_absolute_time() - _last_time) * 1e-6f, 0.05f, 0.5f);
	_last_time = hrt_absolute_time();


	if(_vehicle_status_sub.updated()){
		_vehicle_status_sub.copy(&vehicle_status);
	}

	if(_rc_channels_sub.updated()){
		_rc_channels_sub.copy(&rc_input);
		//_thrust_ctl = rc_input.channels[2];
	}

	if(_vehicle_land_detected_sub.updated()){
		_vehicle_land_detected_sub.copy(&_vehicle_land_detected);
	}

	if(_actuator_controls_sub.updated()){
		_actuator_controls_sub.copy(&_actuator_controls_0);
		_thrust_ctl = _actuator_controls_0.control[actuator_controls_s::INDEX_THROTTLE];
		//float k = 0.2f;
		//_thrust_ctl = k * _thrust_ctl + (1.0f-k) * _thrust_ctl_old;
		//_thrust_ctl_old = _thrust_ctl;
	}

	if(_parameter_update_sub.updated()){

		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		// _ctl_param_p = _param_mc_yawrate_ff.get();
		// _ctl_param_i = _param_mc_rollrate_ff.get();
		// _ctl_param_d = _param_mc_pitchrate_ff.get();
		ramp_time_f = _param_ramp_time.get();
		engine_ctl_trim_f = _param_ctl_trim.get();
		_ctl_param_p_f = _ctl_param_p.get();
		_ctl_param_i_f = _ctl_param_i.get();
		_ctl_param_d2_f = _ctl_param_d.get();
		_ctl_param_offset_f = _ctl_param_offset.get();
		_ctl_param_d_f = _ctl_param_scal_up.get();

	}

	if(_rpm_update_sub.updated()){
		_no_rpm_time = 0;
		_rpm_update_sub.copy(&rpm_pcf8583);

		for(int i = 3; i > 0; i--){
			rpm_record[i] = rpm_record[i-1];
		}

		rpm_record[0]= rpm_pcf8583.indicated_frequency_rpm;

		float _after_rpm = rpm_record[3];
		float _current_rpm = rpm_record[0];

		if(PX4_ISFINITE(_after_rpm) && _after_rpm > 100){

			_target_rpm = 0;
			engine_ctl_output = -1.0f;

			if(rc_input.channels[6] >= engine_ctl_start + rc_engine_offset){

				float _elps_time = hrt_elapsed_time(&_ctl_start_time) * 1e-6f;

				if( _elps_time < ramp_time_f){
					engine_ctl_offset = 0;

					_target_rpm = (fixed_rpm - _start_rpm) * (_elps_time / ramp_time_f) + _start_rpm;

					_delta_rpm = math::constrain((_target_rpm - _after_rpm),-1000.0f,1000.0f) / 1000.0f;


					engine_ctl_output = engine_ctl_start + (engine_ctl_trim_f - engine_ctl_start) * (_elps_time / ramp_time_f);

					if (engine_ctl_output > -0.2f) {
						engine_ctl_output = -0.2f;
					}

					_ctl_rpm_p = 0.0f;//_ctl_param_p * _delta_rpm;
					_ctl_rpm_i = 0.0f;
					_ctl_rpm_d = 0.0f;
					_ctl_rpm_cmp = 0.0f;
					_begin_ctl = false;

				}else{
					_target_rpm = fixed_rpm;// + _ctl_param_set;
					_delta_rpm = math::constrain((_target_rpm - _after_rpm), -1000.0f, 1000.0f) / 1000.0f;
					engine_ctl_output = engine_ctl_trim_f + engine_ctl_offset;

					if(_delta_rpm < 0.15f){ // || _not_in_ctl_time > 10.0f){
						_begin_ctl = true;
					}

					if(_begin_ctl){

						_not_in_ctl_time = 0;

						if(PX4_ISFINITE(_delta_rpm)){

							_ctl_rpm_p = _ctl_param_p_f * _delta_rpm;//_ctl_param_p * _delta_rpm;
							_ctl_rpm_p = math::constrain(_ctl_rpm_p, -0.25f, 0.25f);

							float _delta_rpm_i = _delta_rpm;
							if((float)fabs(_delta_rpm_i) < 0.1f){
								_delta_rpm_i = 0;
							}
							if(!_vehicle_land_detected.landed){ // || _thrust_ctl > 0.5f
								_ctl_rpm_i += _ctl_param_i_f * math::constrain(_delta_rpm_i, -0.4f, 0.4f) * dt;
								_ctl_rpm_i = math::constrain(_ctl_rpm_i, -0.5f, 0.5f);
							}else{
								_ctl_rpm_i = 0;
							}

							if(dt > 0.3f){
								_ctl_rpm_d = math::constrain(-_ctl_param_d2_f * ((_current_rpm - _rpm_last) / dt), -0.1f, 0.1f);
								_rpm_last = _current_rpm;
							}

							// if(_thrust_ctl > 0.00f && _thrust_ctl <= 0.50f){
							// 	_ctl_rpm_cmp = (float)pow(_thrust_ctl, 2) * 3.7f; //0.925
 							if(_thrust_ctl > 0.00f && _thrust_ctl <= 0.35f){
								_ctl_rpm_cmp = (float)pow(_thrust_ctl, 2) * 2.0f; //0.245
							}else if(_thrust_ctl > 0.35f && _thrust_ctl <= 0.40f){
								_ctl_rpm_cmp = _thrust_ctl * 16.0f - 5.355f; //1.045
 							}else if(_thrust_ctl > 0.40f){
	 							_ctl_rpm_cmp = 1.045f;
	 						}

 							_ctl_rpm_cmp =	_ctl_rpm_cmp * _ctl_param_offset_f;
							//if(rc_input.channels[2] > 0.5f){
							//	_ctl_rpm_cmp += _ctl_param_d;//(rc_input.channels[2] - 0.5f) * _ctl_param_d;
							//}
							float _thrust_ctl_second = _thrust_ctl;
							//if (_thrust_ctl_second > 0.4f)  _thrust_ctl_second = 0.4f;
							_ctl_rpm_p = _ctl_rpm_p * (1.0f +  _ctl_param_d_f * _thrust_ctl_second);

						}

					}else{
						_ctl_rpm_i = 0;
						_not_in_ctl_time += dt;
						if(_not_in_ctl_time > 4.0f){
							engine_ctl_offset += 0.015f;
							_not_in_ctl_time = 0;
						}
					}
				}

				engine_ctl_output += _ctl_rpm_p + _ctl_rpm_i + _ctl_rpm_d + _ctl_rpm_cmp;
				//mavlink_log_info(&_mavlink_log_pub,"time_elps: %f ", (double)_elps_time);
				_actuator_controls_2.control[6] = engine_ctl_output;
			}else{
				_start_rpm = _after_rpm;
				_ctl_start_time = hrt_absolute_time();
				_ctl_rpm_i = 0;
				_begin_ctl = false;
				_actuator_controls_2.control[6] = rc_input.channels[6] - rc_engine_offset;
			}

			// mavlink_log_info(&_mavlink_log_pub,"target: %.0f  current %.0f  ctl: %.3f",
					// (double)_target_rpm, (double)_current_rpm, (double)engine_ctl_output);
			// mavlink_log_info(&_mavlink_log_pub,"ctl: %.3f", (double)engine_ctl_output);
		}else{ // for checking engine servor befor start it
			_ctl_start_time = hrt_absolute_time();
			_ctl_rpm_i = 0;
			_begin_ctl = false;
			_actuator_controls_2.control[6] = rc_input.channels[6] - rc_engine_offset;
		}

	}else{
		_no_rpm_time += dt;
		if(_no_rpm_time > 10.0f){
			_no_rpm_time = 10.0f;
		}
	}

	if(_no_rpm_time >= 2.5f){
		mavlink_log_info(&_mavlink_log_pub,"no rpm signal!");
		_actuator_controls_2.control[6] = rc_input.channels[6] - rc_engine_offset;
	}
	//_actuator_controls_2.control[6] = -_actuator_controls_2.control[6];
	_actuator_controls_2.timestamp = hrt_absolute_time();
	_actuator_controls_2_pub.publish(_actuator_controls_2);

	if(vehicle_status.arming_state == 2){
		if(rc_input.channels[9] > 0.25f){
			stm32_gpiowrite(GPIO_ENG_START,false);
		}else{
			stm32_gpiowrite(GPIO_ENG_START,true);
		}

	}

	if(rc_input.channels[10] < -0.25f){
		stm32_gpiowrite(GPIO_ENG_STOP,false);
	}else{
		stm32_gpiowrite(GPIO_ENG_STOP,true);
	}


	/*uint _io_start = stm32_gpioread(GPIO_ENG_START);
	uint _io_stop = stm32_gpioread(GPIO_ENG_STOP);*/

	//mavlink_log_info(&_mavlink_log_pub,"P: %f  I: %f  D: %f", (double)_ctl_param_p, (double)_ctl_param_i, (double)_ctl_param_d);

	perf_end(_loop_perf);
}

int EngineControl::task_spawn(int argc, char *argv[])
{
	EngineControl *instance = new EngineControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int EngineControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int EngineControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EngineControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int engine_control_main(int argc, char *argv[])
{
	return EngineControl::main(argc, argv);
}
