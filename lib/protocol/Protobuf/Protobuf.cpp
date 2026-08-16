#include <Arduino.h>
#include "Protobuf.h"

#define HEADER 250
#define TAIL 253 

Protobuf::Protobuf()
{
	msg = Khageswara_init_zero;
};

bool Protobuf::receiveBuffer()
{
	uint8_t byte_in = Serial1.read();
	if (byte_in == HEADER && head_received < 2)
	{
		Serial.print("got head\r\n");
		head_received++;
		tail_received = 0;
		pos = 0;
	} else {
		Serial.print("doesnt get head\r\n");
		head_received = 0;
		tail_received = 0;
		pos = 0;
		return false;
	}
	if(head_received > 1){
		pos = 0;
	}
	while (head_received > 1 && tail_received < 3)
	{
		byte_in = Serial1.read();
		Serial.print("%d\r\n");
		Serial.print(byte_in);
		if (byte_in == TAIL)
		{
			Serial.print("success\r\n");
			tail_received++;
			iBuffer[pos++] = byte_in;
			if(tail_received > 2) {
				head_received = 0;
				pos -= 3;
				return true;
			}
		}else{
			iBuffer[pos++] = byte_in;
			tail_received = 0;
		}
	}
	return false;
}

bool Protobuf::decode()
{
	// decode
	iStream = pb_istream_from_buffer(iBuffer, sizeof(iBuffer));

	return pb_decode(&iStream, Khageswara_fields, &msg);
}

void Protobuf::sendAttitude(float roll, 
							float pitch, 
							float heading,
					  		float alt_now, 
							float airspeed, 
							float gndspeed,
					  		float latitude, 
							float longitude, 
							float distance,
					  		int32_t battery, 
							bool arm_status, 
							int wpCount,
					  		const char *mode, 
							float navroll, 
							float navp, 
					  		float navy, 
							float targetalt, 
							float t_airspeed, 
					  		float bearing, 
							float hdop, 
							uint32_t sat, 
							float turn, 
					  		float bear, 
							float radius, 
							float acc, 
							float climb, 
					  		float ail, 
							float ele, 
							float rud, 
							float throt, 
					  		float rollimit, 
							float value,
							float ctl_roll,
							float nav_roll,
							float nav_thr)
{
	resetBuffer();
	msg.roll = roll;
	msg.has_roll = true;
	msg.pitch = pitch;
	msg.has_pitch = true;
	msg.heading = heading;
	msg.has_heading = true;
	msg.altitude = alt_now;
	msg.has_altitude = true;
	msg.airspeed = airspeed;
	msg.has_airspeed = true;
	msg.gndspeed = gndspeed;
	msg.has_gndspeed = true;
	msg.latitude = latitude;
	msg.has_latitude = true;
	msg.longitude = longitude;
	msg.has_longitude = true;
	msg.distance_to_wp = distance;
	msg.has_distance_to_wp = true;
	msg.battery = battery;
	msg.has_battery = true;
	msg.arm_status = arm_status;
	msg.has_arm_status = true;
	msg.wpCount = wpCount;
	msg.has_wpCount = true;
	for (int i = 0; i < 4; i++)
	{
		msg.mode[i] = mode[i];
	}
	msg.nav_alt = targetalt;
	msg.has_nav_alt = true;
	msg.nav_speed = t_airspeed;
	msg.has_nav_speed = true;
	msg.crosstrack_error = bearing;
	msg.has_crosstrack_error = true;
	msg.hdop = hdop;
	msg.has_hdop = true;
	msg.satellite_count = sat;
	msg.has_satellite_count = true;
	msg.next_turn_angle = turn;
	msg.has_next_turn_angle = true;
	msg.bearing = bear;
	msg.has_bearing = true;
	msg.acceptance_distance = radius;
	msg.has_acceptance_distance = true;
	msg.lateral_acceleration = acc;
	msg.has_lateral_acceleration = true;
	msg.climb_rate = climb;
	msg.has_climb_rate = true;
	msg.srv[0] = ail;
	msg.srv_count= 4;
	msg.srv[1] = ele;
	msg.srv[2] = rud;
	msg.srv[3] = throt;
	msg.roll_limit = rollimit;
	msg.has_roll_limit = true;
	msg.phase = value;
	msg.has_phase = true;
	msg.msg_type = 1;
	msg.has_msg_type = true;
	msg.chan_roll = navroll;
	msg.has_chan_roll = true;
	msg.chan_pitch = navp;
	msg.has_chan_pitch = true;
	msg.chn_yaw = navy;
	msg.has_chn_yaw = true;
	msg.nav_roll = nav_roll;
	msg.has_nav_roll = true;
	msg.ctl_roll = ctl_roll;
	msg.has_ctl_roll = true;
	msg.nav_speed = nav_thr;
	msg.has_nav_speed = true;

	// encode 
	oStream = pb_ostream_from_buffer(oBuffer, sizeof(oBuffer));

	pb_encode(&oStream, Khageswara_fields, &msg);

	Serial2.write(HEADER);
	Serial2.write(HEADER);
	for (size_t i = 0; i < oStream.bytes_written; i++)
	{
		Serial2.write(oBuffer[i]);
	}
	Serial2.write(TAIL);
	Serial2.write(TAIL);
	Serial2.write(TAIL);
}

void Protobuf::SendFWBuffer(float alt_target, 
						float velocity_target, 
						float dist_to_wp,
					  	float roll_sensor, 
						float pitch_sensor, 
						float heading_sensor,
						float altitude, 
						float airspeed, 
						float groundspeed,
				  		float latitude,
						float longitude, 
						int32_t battery_level,
						const char *mode,
						bool arm_status, 
						int32_t mode_phase,
						float aileron,
						float elevator,
						float rudder,
						int wpCount,
						float roll_demand, 
						float pitch_demand, 
				  		float yaw_demand, 
						float alt_demand, 
						float aspd_demamd,
						float roll_control,
						float pitch_control,
						float yaw_control,
						float throttle_control,
						float roll_radio,
						float pitch_radio,
						float yaw_radio,
						float throttle_radio,
						float Xtrack_err,  
						float Hdop, 
						uint32_t satellite_cnt, 
						float NextTurnAngle, 
				  		float Bearing, 
						float AccDist,
						float Lat_Accel,  
						float climbRate, 
				  		float ail_as_servo, 
						float ele_as_servo, 
						float rud_as_servo, 
						float mtr_as_servo, 
				  		float rollimit)
{
	resetBuffer();
	// msg.alt_count = 1;
	// msg.alt[0] = alt_target;

	// msg.vel_target_count = 1;
	// msg.vel_target[0] = velocity_target;

	msg.distance_to_wp = dist_to_wp;
	msg.has_distance_to_wp = true;

	msg.roll = roll_sensor;
	msg.has_roll = true;
	msg.pitch = pitch_sensor;
	msg.has_pitch = true;
	msg.heading = heading_sensor;
	msg.has_heading = true;
	msg.altitude = altitude;
	msg.has_altitude = true;
	msg.airspeed = airspeed;
	msg.has_airspeed = true;
	msg.gndspeed = groundspeed;
	msg.has_gndspeed = true;
	msg.latitude = latitude;
	msg.has_latitude = true;
	msg.longitude = longitude;
	msg.has_longitude = true;
	msg.battery = battery_level;
	msg.has_battery = true;
	
	for (int i = 0; i < 4; i++)
	{
		msg.mode[i] = mode[i];
	}
	msg.arm_status = arm_status;
	msg.has_arm_status = true;
	msg.phase = mode_phase;
	msg.has_phase = true;
	
	msg.ail = aileron;
	msg.has_ail = true;
	// msg.ele = elevator;
	// msg.has_ele = true;
	// msg.rud = rudder;
	// msg.has_rud = true;
	
	msg.wpCount = wpCount;
	msg.has_wpCount = true;
	
	msg.nav_roll = roll_demand;
	msg.has_nav_roll = true;
	msg.nav_pitch =pitch_demand;
	msg.has_nav_pitch = true;
	// msg.nav_yaw = yaw_demand;
	// msg.has_nav_yaw = true;
	msg.nav_alt = alt_demand;
	msg.has_nav_alt = true;
	msg.nav_speed = aspd_demamd;
	msg.has_nav_speed = true;

	msg.ctl_roll = roll_control;
	msg.has_ctl_roll = true;
	// msg.ctl_pitch = pitch_control;
	// msg.has_ctl_pitch = true;
	// msg.ctl_yaw = yaw_control;
	// msg.has_ctl_yaw = true;
	// msg.ctl_speed = throttle_control;
	// msg.has_ctl_speed = true;

	msg.chan_roll = roll_radio;
	msg.has_chan_roll = true;
	msg.chan_pitch = pitch_radio;
	msg.has_chan_pitch = true;
	msg.chn_yaw = yaw_radio;
	msg.has_chn_yaw = true;
	msg.chn_throttle = throttle_radio;
	msg.has_chn_throttle = true;

	// msg.crosstrack_error = Xtrack_err;
	// msg.has_crosstrack_error = true;
	msg.hdop = Hdop;
	msg.has_hdop = true;
	msg.satellite_count = satellite_cnt;
	msg.has_satellite_count = true;
	// msg.next_turn_angle = NextTurnAngle;
	// msg.has_next_turn_angle = true;
	// msg.bearing = Bearing;
	// msg.has_bearing = true;
	// msg.acceptance_distance = AccDist;
	// msg.has_acceptance_distance = true;
	// msg.lateral_acceleration = Lat_Accel;
	// msg.has_lateral_acceleration = true;
	// msg.climb_rate = climbRate;
	// msg.has_climb_rate = true;

	msg.srv_count= 4;
	msg.srv[0] = ail_as_servo;
	msg.srv[1] = ele_as_servo;
	msg.srv[2] = rud_as_servo;
	msg.srv[3] = mtr_as_servo;

	msg.roll_limit = rollimit;
	msg.has_roll_limit = true;

	msg.msg_type = 1;
	msg.has_msg_type = true;

	// encode 
	oStream = pb_ostream_from_buffer(oBuffer, sizeof(oBuffer));

	pb_encode(&oStream, Khageswara_fields, &msg);

	Serial2.write(HEADER);
	Serial2.write(HEADER);
	for (size_t i = 0; i < oStream.bytes_written; i++)
	{
		Serial2.write(oBuffer[i]);
	}
	Serial2.write(TAIL);
	Serial2.write(TAIL);
	Serial2.write(TAIL);
	Serial2.flush();
}

void Protobuf::sendTrimAndGainData(uint16_t ail_trim, uint16_t ele_trim, uint16_t rud_trim,
								   float *roll_gain, float *pitch_gain, float *yaw_gain){
	msg.msg_type = TRIM_AND_GAIN_DATA;
	msg.cmd = READPARAMS;
	msg.has_msg_type = true;
	msg.srv_trim_ail = ail_trim;
	msg.has_srv_trim_ail = true;
	msg.srv_trim_ele = ele_trim;
	msg.has_srv_trim_ele = true;
	msg.srv_trim_rud = rud_trim;
	msg.has_srv_trim_rud = true;
	msg.roll_gain_count = 3;
	msg.pitch_gain_count = 3;
	msg.yaw_gain_count = 3;
	for(int i=0;i<3;i++){
		msg.roll_gain[i] = roll_gain[i];
		msg.pitch_gain[i] = pitch_gain[i];
		msg.yaw_gain[i] = yaw_gain[i];
		// Serial.print("gain r=%.2f p=%.2f y=%.2f\r\n", msg.roll_gain[i], msg.pitch_gain[i], msg.yaw_gain[i]);
	}

	oStream = pb_ostream_from_buffer(oBuffer, sizeof(oBuffer));

	pb_encode(&oStream, Khageswara_fields, &msg);

	Serial2.write(HEADER);
	Serial2.write(HEADER);
	for (size_t i = 0; i < oStream.bytes_written; i++)
	{
		Serial2.write(oBuffer[i]);
	}
	Serial2.write(TAIL);
	Serial2.write(TAIL);
	Serial2.write(TAIL);
}

void Protobuf::resetBuffer()
{
	msg.has_distance_to_wp = false;
	msg.has_battery = false;
	msg.has_arm_status = false;
	msg.has_wpCount = false;
	msg.has_nav_roll = false;
	msg.has_nav_pitch = false;
	msg.has_nav_yaw = false;
	msg.has_nav_alt = false;
	msg.has_nav_speed = false;
	msg.has_crosstrack_error = false;
	msg.has_hdop = false;
	msg.has_satellite_count = false;
	msg.has_next_turn_angle = false;
	msg.bearing = false;
	msg.has_bearing = false;
	msg.has_acceptance_distance = false;
	msg.has_lateral_acceleration = false;
	msg.has_climb_rate = false;
	msg.srv_count = 0;
	msg.has_roll_limit = false;
	msg.has_phase = false;
	msg.has_cmd = false;
	msg.wp_latitude_count = 0;
	msg.wp_longitude_count = 0;
	msg.alt_count = 0;
	msg.vel_target_count = 0;
	msg.roll_gain_count = 0;
	msg.pitch_gain_count = 0;
	msg.yaw_gain_count = 0;
	msg.has_srv_trim_ail = false;
	msg.has_srv_trim_ele = false;
	msg.has_srv_trim_rud = false;
}

void Protobuf::reset_ack(){
	msg.has_ack = false;
	msg.has_id_ack = false;
}

void Protobuf::send_ack(int32_t ack_id){
	msg.ack = true;
	msg.has_ack = true;
	msg.id_ack = ack_id;
	msg.has_id_ack = true;
	oStream = pb_ostream_from_buffer(oBuffer, sizeof(oBuffer));
	pb_encode(&oStream, Khageswara_fields, &msg);
	Serial1.write(HEADER);
	for (size_t i = 0; i < oStream.bytes_written; i++)
	{
		Serial1.write(oBuffer[i]);
	}
	Serial1.write(TAIL);
}

//getter
int32_t Protobuf::getWpLat(int index)
{
	return msg.wp_latitude[index];
}

int32_t Protobuf::getWpLong(int index)
{
	return msg.wp_longitude[index];
}

float Protobuf::alt(int index)
{
	return msg.alt[index];
}

float Protobuf::getVelTarget(int index)
{
	return msg.vel_target[index];
}

float Protobuf::getRollGain(int index)
{
	return msg.roll_gain[index];
}

float Protobuf::getPitchGain(int index)
{
	return msg.pitch_gain[index];
}

float Protobuf::getYawGain(int index)
{
	return msg.yaw_gain[index];
}

float Protobuf::srvTrimAil()
{
	return msg.srv_trim_ail;
}

float Protobuf::srvTrimEle()
{
	return msg.srv_trim_ele;
}

float Protobuf::srvTrimRud()
{
	return msg.srv_trim_rud;
}

int32_t Protobuf::getCmd()
{
	return msg.cmd;
}

float Protobuf::getDistance()
{
	return msg.distance_to_wp;
}

float Protobuf::getRoll()
{
	return msg.roll;
}

float Protobuf::getPitch()
{
	return msg.pitch;
}

float Protobuf::getHeading()
{
	return msg.heading;
}

float Protobuf::getAltNow()
{
	return msg.altitude;
}

float Protobuf::getAirSpeed()
{
	return msg.airspeed;
}

float Protobuf::getGndSpeed()
{
	return msg.gndspeed;
}

float Protobuf::getLatitude()
{
	return msg.latitude;
}

float Protobuf::getLongitude()
{
	return msg.longitude;
}

int32_t Protobuf::getBattery()
{
	return msg.battery;
}

uint8_t *Protobuf::getMode()
{
	return (uint8_t *)msg.mode;
}

bool Protobuf::isArmed()
{
	return msg.arm_status;
}

float Protobuf::getAil()
{
	return msg.ail;
}

float Protobuf::getRud()
{
	return msg.rud;
}

int Protobuf::wpCount()
{
	return msg.wpCount;
}

int Protobuf::getMsgType()
{
	return msg.msg_type;
}

int Protobuf::getCheckSum()
{
	return msg.checksum;
}

bool Protobuf::getWpIsLanding(int index)
{
	return msg.wp_is_landing[index];
}

int Protobuf::getWpAlt(int index)
{
	return msg.alt[index];
}

int32_t Protobuf::getHomeLat()
{
	return msg.home_lat;
}

int32_t Protobuf::getHomeLon()
{
	return msg.home_lon;
}

float Protobuf::getHomeAlt()
{
	return msg.home_alt;
}

Protobuf gcs;
