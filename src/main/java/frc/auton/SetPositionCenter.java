package frc.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SetPositionCenter extends TemplateAuto {
    @Override
    public void run() {
        robotTracker.resetPosition(new Pose2d(new Translation2d(5.91, 0), new Rotation2d(0)));

        synchronized (this) {
            done = true;
        }
    }
}
