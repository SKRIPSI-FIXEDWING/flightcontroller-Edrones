#include "Khageswara.pb.h"

#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

class Protobuf
{
public:
    enum gcs_cmd
	{
		/** COMMAND (8 bit) */
		//bits 0
		DISARM = 0,
		ARM = 1,
		//bits 1-8 //change mode
		MANUAL = 2,
		FBWA = 4,
		LOITER = 6,
		AUTO = 8,
		RTL = 10,
		AUTOTUNE = 12,
		HEADLOCK = 14,
		ALTLOCK = 16,
		TAKEOFF = 18,
		LANDING = 20,
		FBWB = 22,
		//bits 9-14 //do command
		DO_CLEAR_WP = 512,
		DO_CHANGE_SPEED = 1024,
		//bits 15 //params read and write
		READPARAMS = 12345,
		WRITEPARAMS = 54321
	};

	/// @brief 
	enum msg_type
	{
		CHANGE_MODE = 0,
		ATT_DATA = 1,
		TRIM_AND_GAIN_DATA = 2,
		WP_DATA = 3,
		HOME_DATA = 4,
		WP_LANDING_DATA = 5
	};
	//for accessing data, type "msg.variable" for non array
	//and "msg.variable[index]" for array;
	//you can see list of variable in file Khageswara.proto
	Protobuf();
	//for initialization
	void init();
	//for receiving buffer in array form
	bool receiveBuffer();
	//to initialize before executing decode or encode
	//parameter:
	//array of bytes from received Buffer
	bool decode();
	//to decode the massage
	//return = true if encode or decode success

	int32_t getWpLat(int index);
	int32_t getWpLong(int index);
	float alt(int index);
	float getVelTarget(int index);
	float getRollGain(int index);
	float getPitchGain(int index);
	float getYawGain(int index);
	float srvTrimAil();
	float srvTrimEle();
	float srvTrimRud();
	int32_t getCmd();
	float getDistance();
	float getRoll();
	float getPitch();
	float getHeading();
	float getAltNow();
	float getAirSpeed();
	float getGndSpeed();
	float getLatitude();
	float getLongitude();
	int32_t getHomeLat();
	int32_t getHomeLon();
	float getHomeAlt();
	int32_t getBattery();
	uint8_t *getMode();
	uint8_t get_num_of_wp()
	{
		return msg.wp_latitude_count;
	}
	int32_t get_ack_id(){ return msg.id_ack;}
	bool isArmed();
	float getAil();
	float getRud();
	int wpCount();
	int getMsgType();
	int getCheckSum();
	int getWpAlt(int index);

	void send_ack(int32_t ack_id);

	void sendAttitude(float roll, 
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
					  float navr, 
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
					  float nav_thr);

	void SendFWBuffer(float alt_target, 
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
				  		float rollimit);
					  
	void sendTrimAndGainData(uint16_t ail_trim, uint16_t ele_trim, uint16_t rud_trim,
							 float *roll_gain, float *pitch_gain, float *yaw_gain);
	void resetBuffer();
	void reset_ack();
	bool getWpIsLanding(int index);

    private:
    
	Khageswara msg;
    uint8_t oBuffer[1100];
    uint8_t iBuffer[1100];
    pb_ostream_t oStream;
    pb_istream_t iStream;

	int pos;
	int head_received = 0, tail_received = 0;
};

extern Protobuf gcs;
