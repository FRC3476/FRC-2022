// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.controllers;

import com.revrobotics.CANSparkMax;

/**
 * Sends only new commands to the Talon to reduce CAN usage.
 */
public class LazyCANSparkMax extends CANSparkMax {

    private double prevValue = 0;


    public LazyCANSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
        enableVoltageCompensation(10);

    }

    @Override
    public void set(double speed) {
        //return;

        if (speed != prevValue) {
            super.set(speed);
            prevValue = speed;

        }

    }

    public double getSetpoint() {
        return prevValue;
    }
}
