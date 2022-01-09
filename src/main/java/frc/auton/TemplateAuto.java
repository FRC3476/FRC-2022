package frc.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.auton.guiauto.serialization.command.SendableScript;
import frc.subsystem.Drive;
import frc.subsystem.RobotTracker;

import java.util.ArrayList;

public abstract class TemplateAuto implements Runnable {
    boolean killSwitch = false;
    protected boolean done = false;
    Drive drive = Drive.getInstance();

    /**
     * A function that should be executed at a certain time in the autonomous period.
     */
    ArrayList<SendableScript> delayedExecutions;

    RobotTracker robotTracker = RobotTracker.getInstance();


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
        delayedExecutions.clear();
    }

    synchronized public boolean isDead() {
        return killSwitch;
    }

    synchronized public boolean isFinished() {
        return done;
    }

    public synchronized void reset() {
        this.killSwitch = false;
        this.done = false;
        delayedExecutions.clear();
    }
}
