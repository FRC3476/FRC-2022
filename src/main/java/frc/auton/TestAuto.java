package frc.auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.subsystem.Drive;
import frc.subsystem.RobotTracker;

public class TestAuto extends TemplateAuto {

    Trajectory trajectory;
    public TestAuto(){

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 1);
        trajectoryConfig.addConstraint(new SwerveDriveKinematicsConstraint(drive.getSwerveDriveKinematics(), 3));
        
        ArrayList<Translation2d> pointList = new ArrayList<>();
        pointList.add(new Translation2d(0, 2.5));
        // pointList.add(new Translation2d(-4, 5));
        // pointList.add(new Translation2d(0, 0));

        
        trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(), new Rotation2d(180)), pointList, new Pose2d(-5, 2.5, new Rotation2d(90)), trajectoryConfig);
        
    }


    @Override
    public void run() {
        robotTracker.resetOdometry();
        drive.setAutoPath(trajectory);
        while (!drive.isFinished()) if(isDead()) return;

        synchronized (this) {
            done = true;
        }

    }
    
}
