package frc.subsystem;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class DriveTest {
    public static final double DELTA = 1.0E-2;
    Drive drive;
    Random random = new Random();

    REVPhysicsSim sim;

    @Before
    public void setUp() {
        drive = Drive.getInstance();
        sim = new REVPhysicsSim();
        for (int i = 0; i < drive.swerveMotors.length; i++) {
            sim.addSparkMax(drive.swerveDriveMotors[i], DCMotor.getNEO(1));
            sim.addSparkMax(drive.swerveMotors[i], DCMotor.getNeo550(1));
        }
        sim.run();
    }

    @After
    public void tearDown() throws Exception {
        drive.close();
    }

    @Test
    public void testFeedforward() {
        for (int i = 0; i < 100; i++) {
            double randomX = random.nextDouble() * 4 - 2;
            double randomY = random.nextDouble() * 4 - 2;
            double expectedSpeed = Math.sqrt(randomX * randomX + randomY * randomY);
            drive.swerveDrive(new ChassisSpeeds(randomX, randomY, 0.0));
            for (int j = 0; j < drive.swerveMotors.length; j++) {
                assertEquals(Math.abs(Constants.DRIVE_FEEDFORWARD[j].calculate(expectedSpeed)),
                        Math.abs(drive.swerveDriveMotors[j].getSetVoltage()),
                        DELTA);
            }
        }
    }


}
