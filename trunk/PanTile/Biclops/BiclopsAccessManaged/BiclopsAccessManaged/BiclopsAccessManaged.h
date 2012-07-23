#pragma managed
#pragma once

#include "BiclopsAccess.h"

using namespace System;

namespace BiclopsAccessManaged {

	public ref class BiclopsAccessWrapper
	{

	public:
		BiclopsAccessWrapper();

		int getJointCount() {
			return _biclops->getJointCount();
		}
		
		double getJointPosition(int jnt) {
			return _biclops->getJointPosition(jnt);
		}
		
		double getDesiredJointPosition(int jnt) {
			return _biclops->getJointDesiredPosition(jnt);
		}

		void setJointPosition(double pan, double tilt) {

			// NOTE: in previous code this double array was malloc'ed
			// and then not freed. I have made it static. If it breaks,
			// change it back!
			double jnts[2]; 
			jnts[0] = pan;
			jnts[1] = tilt;
			_biclops->setJointPosition(jnts);
		}

		void setJointPosition(int jnt, double val) {
			_biclops->setJointPosition(jnt, val);
		}


		double getJointLimitMin(int jnt) {
			return _biclops->getJointLimitMin(jnt);
		}

		double getJointLimitMax(int jnt) {
			return _biclops->getJointLimitMax(jnt);
		}

		int getMode() {
			return (int)(_biclops->getMode());
		}

		void setMode(int mode) {
			_biclops->setMode((BiclopsAccess::Mode)mode);
		}

		void park() {
			_biclops->park();
		}

		void home() {
			_biclops->home();
		}

	private:
		BiclopsAccess *_biclops;

	};
}
