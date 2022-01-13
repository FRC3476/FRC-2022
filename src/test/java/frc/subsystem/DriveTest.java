package frc.subsystem;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.utility.controllers.LazyCANSparkMax;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;


public class DriveTest {
    public static final double DELTA = 1.0E-2;
    Drive drive;
    Random random = new Random();

    REVPhysicsSim sim;

    @BeforeEach
    void setUp() {
        drive = Drive.getInstance();
        sim = new REVPhysicsSim();
        for (int i = 0; i < drive.swerveMotors.length; i++) {
            sim.addSparkMax(drive.swerveDriveMotors[i], DCMotor.getNEO(1));
            sim.addSparkMax(drive.swerveMotors[i], DCMotor.getNeo550(1));
        }
        sim.run();
    }

    @AfterEach
    void tearDown() throws Exception {
        drive.close();
    }

    @Test
    void testFeedforward() {
        for (int i = 0; i < 100; i++) {
            double randomX = random.nextDouble() * 4 - 2;
            double randomY = random.nextDouble() * 4 - 2;
            double expectedSpeed = Math.sqrt(randomX * randomX + randomY * randomY);
            drive.swerveDrive(new ChassisSpeeds(randomX, randomY, 0.0));
            for (int j = 0; j < drive.swerveMotors.length; j++) {
                assertEquals(Math.abs(Constants.DRIVE_FEEDFORWARD[j].calculate(expectedSpeed)),
                        Math.abs(drive.swerveDriveMotors[j].getSetVoltage()), DELTA);
            }
        }
    }

    @Test
    void testFallbackAim() {
        for (int i = 0; i < 10; i++) {
            double randomX = random.nextDouble() * 10 - 5;
            double randomY = random.nextDouble() * 10 - 5;
            drive.fallbackAim(new Translation2d(randomX, randomY));
            double expectedAngle = Math.atan2(Constants.GOAL_POSITION.getY() - randomY, Constants.GOAL_POSITION.getX() - randomX);

            assertEquals(expectedAngle, drive.wantedHeading.getRadians(), DELTA);
        }
    }

    @Test
    void testStopMovement() {
        drive.swerveDrive(new ChassisSpeeds(1, 1, 0.0));
        for (LazyCANSparkMax swerveDriveMotor : drive.swerveDriveMotors) {
            assertNotEquals(0.0, swerveDriveMotor.getSetVoltage());
        }

        drive.stopMovement();
        for (int i = 0; i < drive.swerveMotors.length; i++) {
            assertEquals(0.0, drive.swerveDriveMotors[i].getSetVoltage(), DELTA);
        }
    }
    

}
