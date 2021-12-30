package frc.auton.guiauto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.AbstractAutonomousStep;
import frc.auton.guiauto.serialization.Autonomous;
import frc.auton.guiauto.serialization.TrajectoryAutonomousStep;
import frc.subsystem.RobotTracker;
import frc.utility.Serializer;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

import java.io.File;
import java.io.IOException;

//If your autos don't have a superclass that they extend you can replace TemplateAuto with Runnable
public abstract class AbstractGuiAuto extends TemplateAuto {

    private Autonomous autonomous;
    Pose2d initialPose;

    /**
     * Ensure you are creating the objects for your auto on robot init. The roborio will take multiple seconds to initalize the
     * auto.
     *
     * @param autonomousFile File location of the auto
     */
    public AbstractGuiAuto(File autonomousFile) {
        try {
            autonomous = (Autonomous) Serializer.deserializeFromFile(autonomousFile, Autonomous.class);
        } catch (IOException e) {
            DriverStation.reportError("Failed to deserialize auto", e.getStackTrace());
        }
        init();
    }

    /**
     * Ensure you are creating the objects for your auto before you run them. The roborio will take multiple seconds to initalize
     * the auto.
     *
     * @param autonomousJson String of the autonomous
     */
    public AbstractGuiAuto(String autonomousJson) {
        try {
            autonomous = (Autonomous) Serializer.deserialize(autonomousJson, Autonomous.class);
        } catch (IOException e) {
            DriverStation.reportError("Failed to deserialize auto", e.getStackTrace());
        }
        init();
    }

    private void init() {
        //Find and save the initial pose
        for (AbstractAutonomousStep autonomousStep : autonomous.getAutonomousSteps()) {
            if (autonomousStep instanceof TrajectoryAutonomousStep) {
                TrajectoryAutonomousStep trajectoryAutonomousStep = (TrajectoryAutonomousStep) autonomousStep;
                Trajectory.State initialState = trajectoryAutonomousStep.getStates().get(0);
                initialPose = initialState.poseMeters;
                break;
            }
        }
    }

    @Override
    public void run() {
        System.out.println("Started Running: " + Timer.getFPGATimestamp());
        //Set our initial pose in our robot tracker
        RobotTracker.getInstance().setInitialRotation(Rotation2D.fromWPIRotation2d(initialPose.getRotation()));
        RobotTracker.getInstance().setInitialTranslation(Translation2D.fromWPITranslation2d(initialPose.getTranslation()));

        //Loop though all the steps and execute them
        for (AbstractAutonomousStep autonomousStep : autonomous.getAutonomousSteps()) {
            System.out.println("doing a step: " + Timer.getFPGATimestamp());
            autonomousStep.execute(this);
        }

        System.out.println("finished: " + Timer.getFPGATimestamp());

        synchronized (this) {
            done = true;
        }
    }
}