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

#ifndef BATTERY_STATUS_HPP
#define BATTERY_STATUS_HPP

#include <uORB/topics/battery_status.h>
/************************************************/
#include <uORB/topics/rpm.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/actuator_controls.h>
/************************************************/

class MavlinkStreamBatteryStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamBatteryStatus(mavlink); }

	static constexpr const char *get_name_static() { return "BATTERY_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_BATTERY_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_battery = MAVLINK_MSG_ID_BATTERY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return size_per_battery * _battery_status_subs.advertised_count();
	}

private:
	explicit MavlinkStreamBatteryStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

/****************************************************************************/
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};
	uORB::Subscription _actuator_controls_2_sub{ORB_ID(actuator_controls_2)};
	uORB::Subscription _system_power_sub{ORB_ID(system_power)};
/****************************************************************************/

	bool send() override
	{
		/* battery status message with higher resolution */
		mavlink_battery_status_t bat_msg{};
		// TODO: Determine how to better map between battery ID within the firmware and in MAVLink
		bat_msg.time_remaining = (int32_t)NAN;
		bat_msg.temperature = (int16_t)NAN;
		bat_msg.id = 0;
		bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
		bat_msg.type = MAV_BATTERY_TYPE_LIPO;
		bat_msg.current_consumed = -1;
		bat_msg.energy_consumed = -1;
		bat_msg.current_battery = -1;
		bat_msg.battery_remaining = -1;
		bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_OK;

		static constexpr int mavlink_cells_max = (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0]));
			
		for (int cell = 0; cell < mavlink_cells_max; cell++) {
			
			bat_msg.voltages[cell] = UINT16_MAX;
			
		}

		static constexpr int mavlink_cells_ext_max = (sizeof(bat_msg.voltages_ext) / sizeof(bat_msg.voltages_ext[0]));

		for (int cell = mavlink_cells_max; cell < mavlink_cells_max + mavlink_cells_ext_max; cell++) {
			
			bat_msg.voltages_ext[cell - mavlink_cells_max] = UINT16_MAX;
			
		}

		bool _is_rpm_updated = false;
		bool _is_sys_pw_updated = false;

		rpm_s engine_rpm{};
		_rpm_sub.copy(&engine_rpm);

		if (engine_rpm.timestamp > 0) {

			bat_msg.time_remaining = (int32_t)engine_rpm.indicated_frequency_rpm;
			_is_rpm_updated = true;

		}

		system_power_s sys_pow{};

		if (_system_power_sub.update(&sys_pow)) {
			
			bat_msg.temperature = (int16_t)(sys_pow.sensors3v3[0]*100);
			_is_sys_pw_updated = true;
		}
/***************************************************************************************/
		actuator_controls_s  _actuator_controls_2{0};
		_actuator_controls_2_sub.copy(&_actuator_controls_2);
		bat_msg.current_consumed = (_actuator_controls_2.control[6] + 1) / 2 * 100;
/***************************************************************************************/

		bool syspw_timeout = (hrt_elapsed_time(&sys_pow.timestamp) > 50_ms);

		if (_is_sys_pw_updated || (_is_rpm_updated && syspw_timeout)) {

			mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);

			return true;
		}
		
		return false;
	}
};

#endif // BATTERY_STATUS_HPP