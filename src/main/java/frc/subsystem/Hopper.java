package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorMatch;
import frc.robot.Constants;
import frc.subsystem.Outtake.OuttakeState;
import frc.utility.OrangeUtility;
import frc.utility.colorsensor.PicoColorSensor;
import frc.utility.controllers.LazyCANSparkMax;
import org.jetbrains.annotations.NotNull;

import static frc.robot.Constants.HOPPER_CURRENT_LIMIT;

public final class Hopper extends AbstractSubsystem {

    private Outtake outtake = Outtake.getInstance();
    private static @NotNull Hopper INSTANCE = new Hopper();
    private final @NotNull LazyCANSparkMax hopperMotor;
    private final @NotNull PicoColorSensor colorSensor;

    /**
     * A Rev Color Match object is used to register and detect known colors. This can be calibrated ahead of time or during
     * operation.
     * <p>
     * This object uses a simple euclidian distance to estimate the closest match with given confidence range.
     */

    /**
     * Note: Any example colors should be calibrated as the user needs, these are here as a basic example.
     */
    /*private final Color blueTarget = new Color(101, 246, 255);
    private final Color redTarget = new Color(255, 75, 194); */

    private final ColorMatch colorMatch = new ColorMatch();

    public enum ColorSensorStatus {RED, BLUE, NO_BALL}

    private ColorSensorStatus colorSensorStatus = ColorSensorStatus.NO_BALL;

    public static Hopper getInstance() {
        return INSTANCE;
    }

    public enum HopperState {
        ON, OFF, REVERSE, SLOW
    }

    HopperState wantedHopperState = HopperState.OFF;

    private Hopper() {
        super(10, 1);
        hopperMotor = new LazyCANSparkMax(Constants.HOPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hopperMotor.setSmartCurrentLimit(HOPPER_CURRENT_LIMIT);

        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        hopperMotor.setControlFramePeriodMs(25);

        colorSensor = new PicoColorSensor();
        colorSensor.setDebugPrints(Constants.OUTTAKE_DEBUG_PRINTS);

        /*colorMatch.addColorMatch(blueTarget);
        colorMatch.addColorMatch(redTarget); */
    }

    public void checkBallColor() {

        double red = colorSensor.getRawColor0().red / 309d;
        double green = colorSensor.getRawColor0().green;
        double blue = colorSensor.getRawColor0().blue / 567d;
        double proximity = colorSensor.getProximity0();

        // If color sensor is not connected, defaults to no ball
        if (proximity < 70 || !colorSensor.isSensor0Connected()) {
            logData("Ball Color", "No Ball Detected");
            colorSensorStatus = ColorSensorStatus.NO_BALL;
        } else if (red > blue) {
            logData("Ball Color", "Red");
            colorSensorStatus = ColorSensorStatus.RED;
        } else {
            logData("Ball Color", "Blue");
            colorSensorStatus = ColorSensorStatus.BLUE;
        }

        /*double red = colorSensor.getRawColor0().red;
        double green = colorSensor.getRawColor0().green;
        double blue = colorSensor.getRawColor0().blue;
        double proximity = colorSensor.getProximity0();

        double max = red;
        if (green > max) max = green;
        if (blue > max) max = blue;

        red = (255 * red) / max;
        green = (255 * green) / max;
        blue = (255 * blue) / max; */

        logData("Red", red);
        logData("Green", green);
        logData("Blue", blue);
        logData("Color Sensor Proximity", colorSensor.getProximity0());

        logData("Raw Red", colorSensor.getRawColor0().red);
        logData("Raw Blue", colorSensor.getRawColor0().blue);

        /*Color color = new Color(red, green, blue);
        if (proximity > 70) {
            if (colorMatch.matchClosestColor(color).color == redTarget) {
                logData("Ball Color", "Red");
                colorSensorStatus = ColorSensorStatus.RED;
            } else if (colorMatch.matchClosestColor(color).color == blueTarget) {
                logData("Ball Color", "Blue");
                colorSensorStatus = ColorSensorStatus.BLUE;
            } else {
                logData("Bottom Ball Color", "No Ball Detected");
            }
        } else {
            logData("Ball Color", "No Ball Detected");
            colorSensorStatus = ColorSensorStatus.NO_BALL;
        } */
    }

    private void setHopperStatePrivate(HopperState hopperState) {
        // Forces Hopper to run in reverse if outtake is ejecting
        if (outtake.getOuttakeState() == OuttakeState.EJECT) {
            hopperState = HopperState.OFF;
        }

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

    public synchronized ColorSensorStatus getColorSensorStatus() {
        return colorSensorStatus;
    }

    @Override
    public void update() {
        setHopperStatePrivate(wantedHopperState);
        checkBallColor();
    }

    public void setHopperState(HopperState hopperState) {
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
        logData("Hopper Motor Current", hopperMotor.getOutputCurrent());
        logData("Color Sensor Status", colorSensorStatus);
        logData("Is Sensor 1 Connected", colorSensor.isSensor0Connected());
        logData("Is Sensor 2 Connected", colorSensor.isSensor1Connected());
        logData("Color Sensor Last Read Time", colorSensor.getLastReadTimestampSeconds());
    }

    @Override
    public void close() throws Exception {
        setHopperState(HopperState.OFF);
        INSTANCE = new Hopper();
    }
}
