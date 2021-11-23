// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.controllers;

import com.revrobotics.*;
/**
 * Sends only new commands to the Talon to reduce CAN usage.
 */
public class LazyCANSparkMax extends CANSparkMax {
	
	private double prevValue = 0;
	

	public LazyCANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);
		//enableVoltageCompensation(true);
		//configVoltageCompSaturation(12, 10);
	
	}

	@Override
	public void set(double outputValue) {
		//return;
			
		if (outputValue != prevValue) {
			super.set(outputValue);
			prevValue = outputValue;
			
		}
	
	}

	public double getSetpoint() {
		return prevValue;
	}

	public String getVelocity() {
		return null;
	}
}
