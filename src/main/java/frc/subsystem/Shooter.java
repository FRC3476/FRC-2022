package frc.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.utility.controllers.LazyCANSparkMax;
import frc.utility.controllers.LazyTalonSRX;



public class Shooter extends AbstractSubsystem {

    // Talon500
    private LazyTalonSRX shooterWheelMaster;
    private LazyTalonSRX shooterWheelSlave;

    // 775Pro
    private LazyTalonSRX feederWheel;

    // NEO550
    private LazyCANSparkMax hoodMotor;

    // REV Through Bore Encoder
    private DutyCycle hoodEncoder;

    // Home Switch
    private DigitalInput;

    // Private constructor for singleton
    private Shooter(int period, int loggingInterval)
    {
        super(period, loggingInterval);
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

    @Override
    public void close() throws Exception {

    }
}
