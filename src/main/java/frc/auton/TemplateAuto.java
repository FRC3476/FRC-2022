package frc.auton;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.subsystem.RobotTracker;

public abstract class TemplateAuto implements Runnable {
    boolean killSwitch = false;
    protected boolean done = false; 
    //Translation 2D is in inches.

    public TemplateAuto() {
    }

    public Translation2d here() {
        return RobotTracker.getInstance().getPoseMeters().getTranslation();
    }
    
    public Rotation2d dir() {
        return RobotTracker.getInstance().getPoseMeters().getRotation();
    }

    synchronized public void killSwitch() {
        killSwitch = true;
    }

    synchronized public boolean isDead() {
        return killSwitch;
    }

    synchronized public boolean isFinished() {
        return done; 
    }

    public void reset() {
        this.killSwitch = false;
        this.done = false; 
    }
}
