package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorMatch;
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

    /**
     * A Rev Color Match object is used to register and detect known colors. This can be calibrated ahead of time or during
     * operation.
     * <p>
     * This object uses a simple euclidian distance to estimate the closest match with given confidence range.
     */
    private final ColorMatch colorMatcher = new ColorMatch();

    /**
     * Note: Any example colors should be calibrated as the user needs, these are here as a basic example.
     */
    private final Color blueTarget = new Color(0.184, 0.427, 0.39);
    private final Color redTarget = new Color(0.38, 0.41, 0.2);

    public static Hopper getInstance() {
        return instance;
    }

    enum HopperState {
        ON, OFF, REVERSE
    }

    private Hopper() {
        super(Constants.HOPPER_PERIOD);
        hopperMotor = new LazyCANSparkMax(Constants.HOPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(redTarget);
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
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", colorMatcher.matchClosestColor(detectedColor).confidence);

        if (colorMatcher.matchClosestColor(detectedColor).color == redTarget) {
            SmartDashboard.putString("Color", "Red");
        } else if (colorMatcher.matchClosestColor(detectedColor).color == blueTarget) {
            SmartDashboard.putString("Color", "Blue");
        }
//        SmartDashboard.putNumber("Raw Color", Double.parseDouble(colorSensor.getRawColor().toString()));
//        SmartDashboard.putNumber("Closest Detected Color", Double.parseDouble(detectedColor.toString()));
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
