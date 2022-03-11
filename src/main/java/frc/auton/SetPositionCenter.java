package frc.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SetPositionCenter extends TemplateAuto {
    @Override
    public void run() {
        robotTracker.resetPosition(new Pose2d(new Translation2d(5.9699, -0.0481), Rotation2d.fromDegrees(180)));

        synchronized (this) {
            done = true;
        }
    }
}
