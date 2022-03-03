package frc.subsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.utility.Timer;
import frc.utility.controllers.LazyTalonFX;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;


public class DriveTest {
    public static final double DELTA = 1.0E-2;
    Drive drive;
    RobotTracker robotTracker;
    Random random = new Random();

    private double currentMaxVelocityChange = 0;

    @BeforeEach
    void setUp() {
        drive = Drive.getInstance();
        drive.kill();
        robotTracker = RobotTracker.getInstance();
        robotTracker.kill();
    }

    @AfterEach
    void tearDown() throws Exception {

    }

    @Test
    void testStopMovement() throws NoSuchFieldException, IllegalAccessException {
        Field currentRobotState = RobotTracker.class.getDeclaredField("latencyCompensatedChassisSpeeds");
        currentRobotState.setAccessible(true);
        currentRobotState.set(robotTracker, new ChassisSpeeds(0, 0, 0));

        Timer.setTime(55);
        drive.swerveDrive(new ChassisSpeeds(1, 1, 0.0));
        for (LazyTalonFX swerveDriveMotor : drive.swerveDriveMotors) {
            assertNotEquals(0.0, swerveDriveMotor.getSetpoint(), DELTA);
        }
        Timer.setTime(55.1);

        drive.stopMovement();

        drive.update();
        
        for (int i = 0; i < drive.swerveMotors.length; i++) {
            assertEquals(0.0, drive.swerveDriveMotors[i].getSetpoint(), DELTA);
        }
    }

    @Test
    void testMaxAllowedVelocityChange() throws NoSuchFieldException, IllegalAccessException {
        double testPeriod = 0;
        // Reflection used to set currentRobotState with no velocity
        Field currentRobotState = RobotTracker.class.getDeclaredField("latencyCompensatedChassisSpeeds");
        currentRobotState.setAccessible(true);
        currentRobotState.set(robotTracker, new ChassisSpeeds(0, 0, 0));

        // Makes lastLoopTime accessible
        Field lastLoopTime = Drive.class.getDeclaredField("lastLoopTime");
        lastLoopTime.setAccessible(true);

        for (int i = 0; i < 10; i++) {
            testPeriod = Math.random() * 0.15;
            // Sets last loop time to 50 seconds in a match
            lastLoopTime.set(drive, 50);

            // Sets current time to 50 ms beyond last loop
            Timer.setTime(50.0 + testPeriod);
            currentMaxVelocityChange = drive.getMaxAllowedVelocityChange();
            assertEquals(Constants.MAX_ACCELERATION * testPeriod, currentMaxVelocityChange, DELTA);
        }
    }

    @Test
    void testMaxAllowedVelocityChangeAboveMaxTimeLimit() throws NoSuchFieldException, IllegalAccessException {
        // Reflection used to set currentRobotState with no velocity
        Field currentRobotState = RobotTracker.class.getDeclaredField("latencyCompensatedChassisSpeeds");
        currentRobotState.setAccessible(true);
        currentRobotState.set(robotTracker, new ChassisSpeeds(0, 0, 0));

        // Makes lastLoopTime accessible
        Field lastLoopTime = Drive.class.getDeclaredField("lastLoopTime");
        lastLoopTime.setAccessible(true);

        for(int i = 0; i < 10; i++) {
            double testPeriod = Math.random() + 0.15;
            // Sets last loop time to 50 seconds in a match
            lastLoopTime.set(drive, 50);

            // Sets current time to 50 ms beyond last loop
            Timer.setTime(50.0 + testPeriod);
            currentMaxVelocityChange = drive.getMaxAllowedVelocityChange();
            assertEquals(Constants.MAX_ACCELERATION * 0.05, currentMaxVelocityChange, DELTA);
        }
    }

    @Disabled("Limiting angular acceleration breaks test")
    void testLimitAcceleration() throws Exception {
        ChassisSpeeds commandedVelocity = null;
        ChassisSpeeds limitedVelocity = null;
        double velocity = 0;
        double actualVelocity = 0;

        Field currentRobotState = RobotTracker.class.getDeclaredField("latencyCompensatedChassisSpeeds");
        currentRobotState.setAccessible(true);

        Field lastLoopTime = Drive.class.getDeclaredField("lastLoopTime");
        lastLoopTime.setAccessible(true);

        // Set time to 50 seconds

        lastLoopTime.set(drive, 50);

        Timer.setTime(50.1);
        // Set Current Velocity
        ChassisSpeeds currentRobotStateChassisSpeed = new ChassisSpeeds(20, 10, 3);
        currentRobotState.set(robotTracker, currentRobotStateChassisSpeed);

        // Set Commanded Velocity
        commandedVelocity = new ChassisSpeeds(40, 20, 6);
        // call limitAcceleration
        limitedVelocity = drive.limitAcceleration(commandedVelocity);
        // Gets mag of limitedVelocity
        velocity = Math.sqrt((limitedVelocity.vxMetersPerSecond * limitedVelocity.vxMetersPerSecond)
                    + (limitedVelocity.vyMetersPerSecond * limitedVelocity.vyMetersPerSecond));

        // Gets mag of actualVelocity
        actualVelocity = Math.sqrt(
                (currentRobotStateChassisSpeed.vxMetersPerSecond * currentRobotStateChassisSpeed.vxMetersPerSecond)
                        + (currentRobotStateChassisSpeed.vyMetersPerSecond * currentRobotStateChassisSpeed.vyMetersPerSecond));
        // Check if acceleration is at maximum
        assertEquals(Constants.MAX_ACCELERATION * 0.1, Math.abs(velocity - actualVelocity), DELTA);
        assertEquals(limitedVelocity.omegaRadiansPerSecond, 6, DELTA);
    }

    @Disabled("Limiting angular acceleration breaks test")
    void testLimitAcceleration2() throws Exception {

        Field currentRobotState = RobotTracker.class.getDeclaredField("latencyCompensatedChassisSpeeds");
        currentRobotState.setAccessible(true);

        Field lastLoopTime = Drive.class.getDeclaredField("lastLoopTime");
        lastLoopTime.setAccessible(true);

        // Set time to 50 seconds

        lastLoopTime.set(drive, 50);

        Timer.setTime(50.05);
        // Set Current Velocity
        ChassisSpeeds currentRobotStateChassisSpeed = new ChassisSpeeds(23.22, 11.05, -3.1);
        currentRobotState.set(robotTracker, currentRobotStateChassisSpeed);

        // Set Commanded Velocity
        ChassisSpeeds commandedVelocity = new ChassisSpeeds(-62.3, -23.44, 2.05);
        // call limitAcceleration
        ChassisSpeeds limitedVelocity = drive.limitAcceleration(commandedVelocity);
        // Gets mag of limitedVelocity
        double velocity = Math.sqrt((limitedVelocity.vxMetersPerSecond * limitedVelocity.vxMetersPerSecond)
                + (limitedVelocity.vyMetersPerSecond * limitedVelocity.vyMetersPerSecond));

        // Gets mag of actualVelocity
        double actualVelocity = Math.sqrt(
                (currentRobotStateChassisSpeed.vxMetersPerSecond * currentRobotStateChassisSpeed.vxMetersPerSecond)
                        + (currentRobotStateChassisSpeed.vyMetersPerSecond * currentRobotStateChassisSpeed.vyMetersPerSecond));
        // Check if acceleration is at maximum
        assertEquals(-Constants.MAX_ACCELERATION * 0.05, velocity - actualVelocity, DELTA);
        assertEquals(2.05, limitedVelocity.omegaRadiansPerSecond, DELTA);
    }

    @Disabled("Limiting angular acceleration breaks test")
    void testLimitAcceleration3() throws Exception {

        Field currentRobotState = RobotTracker.class.getDeclaredField("latencyCompensatedChassisSpeeds");
        currentRobotState.setAccessible(true);

        Field lastLoopTime = Drive.class.getDeclaredField("lastLoopTime");
        lastLoopTime.setAccessible(true);

        // Set time to 50 seconds

        lastLoopTime.set(drive, 50);

        Timer.setTime(50.2);
        // Set Current Velocity
        ChassisSpeeds currentRobotStateChassisSpeed = new ChassisSpeeds(-20, -10, 15);
        currentRobotState.set(robotTracker, currentRobotStateChassisSpeed);

        // Set Commanded Velocity
        ChassisSpeeds commandedVelocity = new ChassisSpeeds(-14.4, 20, -15.5);
        // call limitAcceleration
        ChassisSpeeds limitedVelocity = drive.limitAcceleration(commandedVelocity);
        // Gets mag of limitedVelocity
        double velocity = Math.hypot(limitedVelocity.vxMetersPerSecond - currentRobotStateChassisSpeed.vxMetersPerSecond,
                limitedVelocity.vyMetersPerSecond - currentRobotStateChassisSpeed.vyMetersPerSecond);

        // Check if acceleration is at maximum
        assertEquals(Constants.MAX_ACCELERATION * 0.05, velocity, DELTA);
        assertEquals(-15.5, limitedVelocity.omegaRadiansPerSecond, DELTA);
    }

    @Test
    void testAngleDiff1() {
        assertEquals(0, drive.getAngleDiff(0, 0));
    }

    @Test
    void testAngleDiff2() {
        assertEquals(-10, drive.getAngleDiff(0, 10));
    }

    @Test
    void testAngleDiff3() {
        assertEquals(-100, drive.getAngleDiff(0, 100));
    }

    @Test
    void testAngleDiff4() {
        assertEquals(170, drive.getAngleDiff(0, 190));
    }

    @Test
    void testAngleDiff5() {
        assertEquals(0, drive.getAngleDiff(0, 360));
    }

    @Test
    void testAngleDiff6() {
        assertEquals(30, drive.getAngleDiff(90, 60));
    }

    @Test
    void testAngleDif7() {
        assertEquals(-179, drive.getAngleDiff(241, 60));
    }

    @Test
    void testAngleDif8() {
        assertEquals(180, drive.getAngleDiff(240, 60));
    }

    @Test
    void testAngleDif9() {
        assertEquals(-20, drive.getAngleDiff(350, 10));
    }

    @Test
    void testAngleDif10() {
        assertEquals(20, drive.getAngleDiff(10, 350));
    }

    @Test
    void testAngleDif11() {
        assertEquals(10, drive.getAngleDiff(73, 63));
    }

    @Test
    void testAngleDif12() {
        assertEquals(116, drive.getAngleDiff(84, 328));
    }

    @Test
    void testAngleDif13() {
        assertEquals(-27, drive.getAngleDiff(96, 123));
    }

    @Test
    void testAngleDif14() {
        assertEquals(-3, drive.getAngleDiff(359, 2));
    }

    @Test
    void testAngleDif15() {
        assertEquals(133, drive.getAngleDiff(123, -10));
    }

    @Test
    void testAngleDif16() {
        assertEquals(-23, drive.getAngleDiff(240, 263));
    }

    @Test
    void testAngleDif17() {
        assertEquals(19, drive.getAngleDiff(189, 170));
    }
}
