#include <modules/finken_wall_avoid/finken_wall_avoid.h>

#define T 1.0f/FINKEN_WALL_AVOID_UPDATE_FREQ
#define T1 FINKEN_WALL_AVOID_CONTROL_DELAY_TIME
#define Tv FINKEN_WALL_AVOID_CONTROL_HOLD_TIME
#define Kp FINKEN_WALL_AVOID_CONTROL_GAIN

#define a0 (T-2.0f*T1)/(T+2.0f*T1)
#define b0 Kp*(T-2.0f*Tv)/(T+2.0f*T1)
#define b1 Kp*(T+2.0f*Tv)/(T+2.0f*T1)

static const float maxControlRoll  = FINKEN_WALL_AVOID_MAX_CONTROL;
static const float maxControlPitch = FINKEN_WALL_AVOID_MAX_CONTROL;
static const float guardDist       = FINKEN_WALL_AVOID_GUARD_DIST;
static const float goalDist        = FINKEN_WALL_AVOID_GOAL_DIST;
static const float freeDist        = FINKEN_WALL_AVOID_FREE_FACTOR * FINKEN_WALL_AVOID_GOAL_DIST;
static const float maxDist         = FINKEN_WALL_AVOID_MAX_DIST;
static const float rollOffset      = FINKEN_WALL_AVOID_ROLL_OFFSET;
static const float pitchOffset     = FINKEN_WALL_AVOID_PITCH_OFFSET;

float pitchError_k1, pitch_k1, rollError_k1, roll_k1;
float pitchInDamped, rollInDamped;

static float rollControl(float rollError) {
	float roll = -a0*roll_k1 + b1*rollError + b0*rollError_k1;
	roll = (roll < -maxControlPitch) ? -maxControlPitch : roll;
	roll = (roll > maxControlPitch) ? maxControlPitch : roll;
	roll_k1 = roll;
	rollError_k1 = rollError;
	return roll;
}

static float rollWallAvoid(float rollIn, float distXLeft, float distXRight) {
	distXLeft = distXLeft<maxDist?distXLeft:maxDist;
	distXRight  = distXRight<maxDist?distXRight:maxDist;
	//float distXControlLeft = distXLeft>goalDist?freeDist:distXLeft;
	//float distXControlRight  = distXRight>goalDist?freeDist:distXRight;
	float distXControlDiff  = 0.0f;
	//if (distXLeft<goalDist && distXRight <goalDist){
	//	distXControlDiff  = 0.0f-(distXRight-distXLeft)/2.0;
	//}
	//else{
		if (distXLeft<distXRight && distXLeft<goalDist*1.0){
			distXControlDiff = -goalDist+distXLeft;
		}
		if (distXRight<distXLeft && distXRight<goalDist*1.0){
			distXControlDiff = goalDist-distXRight;
		}
	//}
	float newRoll = rollControl(distXControlDiff);
	/*rollIn = (rollIn < -maxRCPitch) ? -maxRCPitch : rollIn;
	rollIn = (rollIn > maxRCPitch)  ?  maxRCPitch : rollIn;
	rollIn = (rollIn< deadRCPitch && rollIn > -deadRCPitch) ? 0.0f : rollIn;*/
  	if (distXLeft < maxDist && rollIn > 0){
		rollIn*=(distXLeft-guardDist)/(maxDist-guardDist);
		rollIn=(rollIn<0.0)?0.0:rollIn;
	}
	if (distXRight < maxDist && rollIn < 0){
		rollIn*=(distXRight-guardDist)/(maxDist-guardDist);
		rollIn=(rollIn>0.0)?0.0:rollIn;
	}
	rollInDamped = rollIn;
	return newRoll + rollIn + rollOffset;
}

static float pitchControl(float pitchError) {
	float pitch = -a0*pitch_k1 + b1*pitchError + b0*pitchError_k1;
	pitch = (pitch < -maxControlPitch) ? -maxControlPitch : pitch;
	pitch = (pitch > maxControlPitch) ? maxControlPitch : pitch;
	pitch_k1 = pitch;
	pitchError_k1 = pitchError;
	return pitch;
}

static float pitchWallAvoid(float pitchIn, float distXFront, float distXBack) {
	distXFront = distXFront<maxDist?distXFront:maxDist;
	distXBack  = distXBack<maxDist?distXBack:maxDist;
	//float distXControlFront = distXFront>goalDist?freeDist:distXFront;
	//float distXControlBack  = distXBack>goalDist?freeDist:distXBack;
	//float distXControlDiff  = 0.0f-(distXControlBack-distXControlFront);
	float distXControlDiff  = 0.0f;
	//if (distXFront<goalDist && distXBack <goalDist){
	//	distXControlDiff  = 0.0f-(distXBack-distXFront)/2.0;
	//}
	//else{
		if (distXFront<distXBack && distXFront<goalDist*1.0){
			distXControlDiff = -goalDist+distXFront;
		}
		if (distXBack<distXFront && distXBack<goalDist*1.0){
			distXControlDiff = goalDist-distXBack;
		}
	//}
	float newPitch = pitchControl(distXControlDiff);
	/*pitchIn = (pitchIn < -maxRCPitch) ? -maxRCPitch : pitchIn;
	pitchIn = (pitchIn > maxRCPitch)  ?  maxRCPitch : pitchIn;
	pitchIn = (pitchIn< deadRCPitch && pitchIn > -deadRCPitch) ? 0.0f : pitchIn;*/
  	if (pitchIn > 0){
		pitchIn*=(distXFront-guardDist)/(maxDist-guardDist);
		pitchIn=(pitchIn)<0.0?0.0:pitchIn;
	}
  	if (pitchIn < 0){
		pitchIn*=(distXBack-guardDist)/(maxDist-guardDist);
		pitchIn=(pitchIn>0.0)?0.0:pitchIn;
	}
	pitchInDamped = pitchIn;
	return newPitch + pitchIn + pitchOffset;
}

void finken_wall_avoid_init() {
	pitchError_k1 = 0.0f;
	pitch_k1      = 0.0f;
	rollError_k1  = 0.0f;
	roll_k1       = 0.0f;
		
	register_periodic_telemetry(DefaultPeriodic, "FINKEN_WALL_AVOID", send_finken_wall_avoid_telemetry);
}

void finken_wall_avoid_periodic() {

	finken_actuators_set_point.pitch = pitchWallAvoid(finken_system_set_point.pitch, finken_sensor_model.distance_d_back/100.0, finken_sensor_model.distance_d_front/100.0);
	finken_actuators_set_point.roll = rollWallAvoid(finken_system_set_point.roll, finken_sensor_model.distance_d_right/100.0, finken_sensor_model.distance_d_left/100.0);

}


void send_finken_wall_avoid_telemetry(struct transport_tx *trans, struct link_device* link) {
	trans = trans;
	link = link;
	float front=finken_sensor_model.distance_d_front/100.0f;
	float back=finken_sensor_model.distance_d_back/100.0f;
	float left=finken_sensor_model.distance_d_left/100.0f;
	float right=finken_sensor_model.distance_d_right/100.0f;
	float z=POS_FLOAT_OF_BFP(finken_sensor_model.pos.z);
	DOWNLINK_SEND_FINKEN_WALL_AVOID(
	DefaultChannel,
	DefaultDevice,
	&z,
	&pitchInDamped,
	&rollInDamped,
	&pitch_k1,
	&roll_k1,
	&rollError_k1,
	&pitchError_k1,
	&front,
	&back,
	&left,
	&right,
	&finken_actuators_set_point.thrust
	);
}
