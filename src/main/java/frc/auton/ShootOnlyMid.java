package frc.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.subsystem.RobotTracker;
import frc.subsystem.VisionManager;

public class ShootOnlyMid extends TemplateAuto {
    @Override
    public void run() {
        try {
            RobotTracker.getInstance().resetPosition(new Pose2d(new Translation2d(5.9699, -0.0481),
                    Rotation2d.fromDegrees(-178.4657)));
            VisionManager.getInstance().shootBalls(20);
        } catch (InterruptedException e) {
            return;
        }
    }
}
