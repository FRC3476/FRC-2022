package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.utility.OrangeUtility;
import frc.utility.colorsensor.PicoColorSensor;
import frc.utility.colorsensor.PicoColorSensor.RawColor;
import frc.utility.controllers.LazyCANSparkMax;
import org.jetbrains.annotations.NotNull;

import static frc.robot.Constants.HOPPER_CURRENT_LIMIT;

public final class Hopper extends AbstractSubsystem {
    private static @NotNull Hopper INSTANCE = new Hopper();
    private final @NotNull LazyCANSparkMax hopperMotor;
    private final @NotNull PicoColorSensor colorSensor;

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

    private boolean ballDetected;

    public static Hopper getInstance() {
        return INSTANCE;
    }

    public enum HopperState {
        ON, OFF, REVERSE, SLOW
    }

    HopperState wantedHopperState = HopperState.OFF;

    private Hopper() {
        super(Constants.HOPPER_PERIOD, 5);
        hopperMotor = new LazyCANSparkMax(Constants.HOPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hopperMotor.setSmartCurrentLimit(HOPPER_CURRENT_LIMIT);

        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        hopperMotor.setControlFramePeriodMs(25);

        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(redTarget);
        colorSensor = new PicoColorSensor();
    }

    private Color convertRawColor(RawColor rawColor) {
        return new Color(rawColor.red, rawColor.green, rawColor.blue);
    }

    public void checkBallColor() {
        Color detectedColor = convertRawColor(colorSensor.getRawColor0());
        //Color detectedColorBottom = bottomBall.getColor();

        logData("Ball Red", detectedColor.red);
        logData("Ball Green", detectedColor.green);
        logData("Ball Blue", detectedColor.blue);
        logData("Color Confidence", colorMatcher.matchClosestColor(detectedColor).confidence);

//        logData("Bottom Red", detectedColorBottom.red);
//        logData("Bottom Green", detectedColorBottom.green);
//        logData("Bottom Blue", detectedColorBottom.blue);
//        logData("Bottom Confidence", colorMatcher.matchClosestColor(detectedColorBottom).confidence);

        if (colorMatcher.matchClosestColor(detectedColor).color == redTarget) {
            logData("Ball Color", "Red");
            ballDetected = true;
        } else if (colorMatcher.matchClosestColor(detectedColor).color == blueTarget) {
            logData("Ball Color", "Blue");
            ballDetected = true;
        } else {
            logData("Ball Color", "No Ball Detected");
            ballDetected = false;
        }

//        if (colorMatcher.matchClosestColor(detectedColorBottom).color == redTarget) {
//            logData("Bottom Ball Color", "Red");
//            bottomBallDetected = true;
//        } else if (colorMatcher.matchClosestColor(detectedColorBottom).color == blueTarget) {
//            logData("Bottom Ball Color", "Blue");
//            bottomBallDetected = true;
//        } else {
//            logData("Bottom Ball Color", "No Ball Detected");
//            bottomBallDetected = false;
//        }
    }

    public void setHopperState(HopperState hopperState) {
        switch (hopperState) {
            case ON:
                hopperMotor.set(Constants.HOPPER_SPEED);
                Shooter.getInstance().runFeederWheelReversed = true;
                break;
            case OFF:
                hopperMotor.set(0);
                Shooter.getInstance().runFeederWheelReversed = false;
                break;
            case REVERSE:
                hopperMotor.set(-Constants.HOPPER_SPEED);
                Shooter.getInstance().runFeederWheelReversed = false;
                break;
            case SLOW:
                hopperMotor.set(Constants.HOPPER_SPEED / 2);
                Shooter.getInstance().runFeederWheelReversed = false;
                break;
        }
    }

    public synchronized boolean isBallDetected() {
        return ballDetected;
    }

    @Override
    public void update() {
        setHopperState(wantedHopperState);
        checkBallColor();
    }

    public void setWantedHopperState(HopperState hopperState) {
        wantedHopperState = hopperState;
    }

    @Override
    public void selfTest() {
        setHopperState(HopperState.ON);
        OrangeUtility.sleep(5000);
        setHopperState(HopperState.OFF);
    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("Hopper Motor Current", hopperMotor.getOutputCurrent());
    }

    @Override
    public void close() throws Exception {
        setHopperState(HopperState.OFF);
        INSTANCE = new Hopper();
    }
}
