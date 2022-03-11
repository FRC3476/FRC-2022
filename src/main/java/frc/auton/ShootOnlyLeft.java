package frc.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.subsystem.RobotTracker;
import frc.subsystem.VisionManager;

public class ShootOnlyLeft extends TemplateAuto {
    @Override
    public void run() {
        try {
            RobotTracker.getInstance().resetPosition(new Pose2d(new Translation2d(6.6016, 1.6075), Rotation2d.fromDegrees(135)));
            VisionManager.getInstance().shootBalls(20);
        } catch (InterruptedException e) {
            return;
        }
    }
}
