package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.utility.OrangeUtility;
import frc.utility.controllers.LazyCANSparkMax;

public class Hopper extends AbstractSubsystem {
    public static Hopper instance = new Hopper();
    LazyCANSparkMax hopperMotor;
    private final ColorSensorV3 topBall = new ColorSensorV3(I2C.Port.kMXP);
    private final ColorSensorV3 bottomBall = new ColorSensorV3(I2C.Port.kOnboard);

    private boolean topBallDetected = false;
    private boolean bottomBallDetected = false;


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

    HopperState wantedHopperState = HopperState.OFF;

    private Hopper() {
        super(Constants.HOPPER_PERIOD);
        hopperMotor = new LazyCANSparkMax(Constants.HOPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(redTarget);
    }

    public void checkBallColor() {
        Color detectedColorTop = topBall.getColor();
        Color detectedColorBottom = bottomBall.getColor();

        logData("Top Ball Red", detectedColorTop.red);
        logData("Top Ball Green", detectedColorTop.green);
        logData("Top Ball Blue", detectedColorTop.blue);
        logData("Top Confidence", colorMatcher.matchClosestColor(detectedColorTop).confidence);

        logData("Bottom Red", detectedColorBottom.red);
        logData("Bottom Green", detectedColorBottom.green);
        logData("Bottom Blue", detectedColorBottom.blue);
        logData("Bottom Confidence", colorMatcher.matchClosestColor(detectedColorBottom).confidence);

        if (colorMatcher.matchClosestColor(detectedColorTop).color == redTarget) {
            logData("Top Ball Color", "Red");
            topBallDetected = true;
        } else if (colorMatcher.matchClosestColor(detectedColorTop).color == blueTarget) {
            logData("Top Ball Color", "Blue");
            topBallDetected = true;
        } else {
            logData("Top Ball Color", "No Ball Detected");
            topBallDetected = false;
        }

        if (colorMatcher.matchClosestColor(detectedColorBottom).color == redTarget) {
            logData("Bottom Ball Color", "Red");
            bottomBallDetected = true;
        } else if (colorMatcher.matchClosestColor(detectedColorBottom).color == blueTarget) {
            logData("Bottom Ball Color", "Blue");
            bottomBallDetected = true;
        } else {
            logData("Bottom Ball Color", "No Ball Detected");
            bottomBallDetected = false;
        }
    }

    public void numberOfBalls() {
        if (topBallDetected && bottomBallDetected) {
            logData("Number of Balls", 2);
        } else if (bottomBallDetected || topBallDetected) {
            logData("Number of Balls", 1);
        } else {
            logData("Number of Balls", 0);
        }
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
        setHopperState(wantedHopperState);
        checkBallColor();
        numberOfBalls();
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

    }

    @Override
    public void close() throws Exception {
        setHopperState(HopperState.OFF);
        instance = new Hopper();
    }
}
