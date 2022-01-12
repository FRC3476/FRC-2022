package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.utility.OrangeUtility;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Random;

public class RobotTrackerTest {
    Drive drive;
    RobotTracker robotTracker;

    Random random = new Random();

    @Before
    public void setUp() throws Exception {
        drive = Drive.getInstance();
        drive.kill();
        robotTracker = RobotTracker.getInstance();
        robotTracker.kill();
        robotTracker.resetPosition(new Pose2d(0, 0, new Rotation2d(90)), new Rotation2d(0));
    }

    @After
    public void tearDown() throws Exception {
        drive.close();
        robotTracker.close();
    }

    @Test
    public void robotTrackerTest() {

        double x = 0;
        double y = 0;
        double theta = 0;
        double period = 0.005;
        for (int i = 0; i < 1000; i++) {
            double randomX = 1;
            double randomY = 0;
            double randomRotation = 0;

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(randomX, randomY, randomRotation);
            drive.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);
            x += randomX * period;
            y += randomY * period;
            theta += randomRotation * period;
            RobotTracker.getInstance().updateOdometry(new Rotation2d(theta),
                    drive.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds));
            OrangeUtility.sleep((long) (period * 1000));
        }
        System.out.println("x: " + x);
        System.out.println("y: " + y);
        System.out.println("theta: " + theta);
        System.out.println("predicted " + robotTracker.getPoseMeters());
    }
}
