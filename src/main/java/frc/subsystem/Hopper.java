package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.utility.OrangeUtility;
import frc.utility.controllers.LazyCANSparkMax;

public class Hopper extends AbstractSubsystem {
    public static Hopper instance = new Hopper();
    LazyCANSparkMax hopperMotor;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);

    public static Hopper getInstance() {
        return instance;
    }

    enum HopperState {
        ON, OFF, REVERSE
    }

    private Hopper() {
        super(Constants.HOPPER_PERIOD);
        hopperMotor = new LazyCANSparkMax(Constants.HOPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void setHopperState(HopperState hopperState) {
        switch (hopperState) {
            case ON:
                hopperMotor.set(Constants.HOPPER_SPEED);
                break;
            case OFF:
                hopperMotor.set(0);
                break;
            case REVERSE:
                hopperMotor.set(-Constants.HOPPER_SPEED);
                break;
        }
    }

    @Override
    public void update() {
        Color detectedColor = colorSensor.getColor();
        private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
    }

    @Override
    public void selfTest() {
        setHopperState(HopperState.ON);
        OrangeUtility.sleep(5000);
        setHopperState(HopperState.OFF);
    }

    @Override
    public void logData() {

    }

    @Override
    public void close() throws Exception {

    }
}
