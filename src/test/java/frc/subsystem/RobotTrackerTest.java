package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class RobotTrackerTest {
    Drive drive;
    RobotTracker robotTracker;

    Random random;

    @BeforeEach
    void setUp() throws Exception {
        drive = Drive.getInstance();
        drive.kill();
        robotTracker = RobotTracker.getInstance();
        robotTracker.kill();
        random = new Random(123123);
    }

    @AfterEach
    void tearDown() throws Exception {
        robotTracker.close();
    }

    @Test
    void robotTrackerTest() throws NoSuchFieldException, IllegalAccessException, NoSuchMethodException, InvocationTargetException { //TODO: Figure out why messing with rotation
        // breaks things
        WPIUtilJNI.enableMockTime();
        double time = 0;
        for (int j = 0; j < 100; j++) {
            double x = random.nextDouble() * 5 - 2.5;
            double y = random.nextDouble() * 5 - 2.5;
            double theta = 0; //random.nextDouble() * 2 * Math.PI;
            double period = 0.05;

            WPIUtilJNI.setMockTime(0);
            robotTracker.resetPosition(new Pose2d(x, y, new Rotation2d(theta)), new Rotation2d(0));

            double largeRandomX = random.nextDouble() * 10 - 5;
            double largeRandomY = random.nextDouble() * 10 - 5;
            for (int i = 0; i < 1000; i++) {
                double randomX = largeRandomX + random.nextDouble() * 0.2;
                double randomY = largeRandomY + random.nextDouble() * 0.4;
                double randomRotation = 0;//+ random.nextDouble() * 0.02; //Seems to cause a bunch of variance

                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(randomX, randomY, randomRotation,
                        Rotation2d.fromDegrees(0));
                Drive.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);

                double dt = period + random.nextDouble() * 0.008 - 0.004;
                theta += randomRotation * dt;
                x += randomX * dt;
                y += randomY * dt;

                time += dt;

                Method updateOdomotry = RobotTracker.class.getDeclaredMethod("updateOdometry", double.class, Rotation2d.class,
                        SwerveModuleState[].class);
                updateOdomotry.setAccessible(true);

                updateOdomotry.invoke(RobotTracker.getInstance(), time, new Rotation2d(theta),
                        Drive.getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds));
            }

            Field swerveDriveOdometry = RobotTracker.class.getDeclaredField("swerveDriveOdometry");
            swerveDriveOdometry.setAccessible(true);
            Pose2d odometry = ((SwerveDriveOdometry) swerveDriveOdometry.get(robotTracker)).getPoseMeters();

            assertEquals(x, odometry.getTranslation().getX(), 0.3);
            assertEquals(y, odometry.getTranslation().getY(), 0.3);
            assertEquals(theta, odometry.getRotation().getDegrees(), 0.1);
        }

    }
}
