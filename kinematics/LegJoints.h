#ifndef LEGJOINTS_H
#define LEGJOINTS_H

namespace kinematics {

struct LegJoints {
	const bool isLeftLeg;
	const float hipYawPitch;
	const float hipRoll;
	const float hipPitch;
	const float kneePitch;
	const float anklePitch;
	const float ankleRoll;
	const bool isClipped;

	LegJoints(const bool _isLeftLeg, const float _hipYawPitch, const float _hipRoll,
			const float _hipPitch, const float _kneePitch, const float _anklePitch,
			const float _ankleRoll, const bool _isClipped=false) :
			isLeftLeg(_isLeftLeg), hipYawPitch(_hipYawPitch), hipRoll(_hipRoll), hipPitch(
					_hipPitch), kneePitch(_kneePitch), anklePitch(_anklePitch), ankleRoll(
					_ankleRoll), isClipped(_isClipped) {
	}
};
}

#endif
