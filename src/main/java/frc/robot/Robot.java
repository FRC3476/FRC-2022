// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.OsUtil;
import frc.auton.guiauto.serialization.reflection.ClassInformationSender;
import frc.subsystem.BlinkinLED;
import frc.subsystem.Drive;
import frc.subsystem.Hopper;
import frc.subsystem.Intake;
import frc.subsystem.RobotTracker;
import frc.utility.Controller;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.OrangeUtility;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.File;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public class Robot extends TimedRobot {

    public boolean useFieldRelative = false;

    //GUI
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable autoDataTable = instance.getTable("autodata");
    NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");
    NetworkTableEntry enabled = autoDataTable.getEntry("enabled");
    NetworkTableEntry pathProcessingStatusEntry = autoDataTable.getEntry("processing");
    NetworkTableEntry pathProcessingStatusIdEntry = autoDataTable.getEntry("processingid");

    NetworkAuto networkAuto;
    @Nullable String lastAutoPath = null;

    @NotNull ExecutorService deserializerExecutor = Executors.newSingleThreadExecutor();

    //Auto
    @Nullable TemplateAuto selectedAuto;
    @Nullable Thread autoThread;
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "My Auto";
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    //Subsystems
    private final RobotTracker robotTracker = RobotTracker.getInstance();
    private final Drive drive = Drive.getInstance();
    private final BlinkinLED blinkinLED = BlinkinLED.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Intake intake = Intake.getInstance();
//    private final Shooter shooter = Shooter.getInstance();
//    private final Climber climber = Climber.getInstance();

    //Inputs
    private final Controller xbox = new Controller(0);
    private final Controller stick = new Controller(1);
    private final Controller buttonPanel = new Controller(2);


    //Control loop states
    boolean limelightTakeSnapshots;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        autoChooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        autoChooser.addOption("My Auto", CUSTOM_AUTO);
        SmartDashboard.putData("Auto choices", autoChooser);

        startSubsystems();
        robotTracker.resetGyro();
        OrangeUtility.sleep(50);
        robotTracker.resetPosition(new Pose2d());
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        //Listen changes in the network auto
        if (autoPath.getString(null) != null && !autoPath.getString(null).equals(lastAutoPath)) {
            lastAutoPath = autoPath.getString(null);
            deserializerExecutor.execute(() -> { //Start deserializing on another thread
                System.out.println("start parsing autonomous");
                //Set networktable entries for the gui notifications
                pathProcessingStatusEntry.setDouble(1);
                pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);

                networkAuto = new NetworkAuto(); //Create the auto object which will start deserializing the json and the auto
                // ready to be run
                System.out.println("done parsing autonomous");
                //Set networktable entries for the gui notifications
                pathProcessingStatusEntry.setDouble(2);
                pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
            });
        }

        //TODO: Debug why this does not work
        if (buttonPanel.getRisingEdge(9)) {
            limelightTakeSnapshots = !limelightTakeSnapshots;
            limelight.takeSnapshots(limelightTakeSnapshots);
            System.out.println("limelight taking snapshots " + limelightTakeSnapshots);
        }
    }

    @Override
    public void autonomousInit() {
        enabled.setBoolean(true);
        startSubsystems();

        if (networkAuto == null) {
            System.out.println("Using normal autos");
            selectedAuto = null; //TODO put an actual auto here
            //TODO put autos here
        } else {
            System.out.println("Using autos from network tables");
            selectedAuto = networkAuto;
        }
        //Since autonomous objects can be reused they need to be reset them before we can reuse them again 
        selectedAuto.reset();

        //We then create a new thread to run the auto and run it
        autoThread = new Thread(selectedAuto);
        autoThread.start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        killAuto();
        enabled.setBoolean(true);
        startSubsystems();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        xbox.update();
        stick.update();
        buttonPanel.update();
        if (useFieldRelative && drive.useFieldRelative) {
            if (xbox.getRawButton(3)) {
                //Increase the deadzone so that we drive straight
                drive.swerveDriveFieldRelative(new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0),
                        -xbox.getRawAxis(4)).applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs());
            } else {
                drive.swerveDriveFieldRelative(new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0),
                        -xbox.getRawAxis(4)).applyDeadZone(0.05, 0.05, 0.2, 0.2).squareInputs());
            }
        } else {
            if (xbox.getRawButton(3)) {
                //Increase the deadzone so that we drive straight
                drive.swerveDrive(new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0),
                        -xbox.getRawAxis(4)).applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs());
            } else {
                drive.swerveDrive(new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0),
                        -xbox.getRawAxis(4)).applyDeadZone(0.05, 0.05, 0.2, 0.2).squareInputs());
            }
        }

        if (xbox.getRisingEdge(Controller.XboxButtons.B) || buttonPanel.getRisingEdge(7)) {
            intake.setIntakeSolState(intake.getIntakeSolState() == Intake.IntakeSolState.OPEN ?
                    Intake.IntakeSolState.CLOSE : Intake.IntakeSolState.OPEN);
        }

        if (xbox.getRawAxis(3) > 0.1) {
            // Intake balls
            intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
        } else if (buttonPanel.getRawButton(10)) {
            // Eject everything
            intake.setWantedIntakeState(Intake.IntakeState.EJECT);
        } else {
            intake.setWantedIntakeState(Intake.IntakeState.OFF);
        }


        if (xbox.getRisingEdge(1)) {
            //Resets the current robot heading to zero. Useful if the heading drifts for some reason
            robotTracker.resetPosition(new Pose2d(robotTracker.getLastEstimatedPoseMeters().getTranslation(), new Rotation2d(0)));
        }

        if (xbox.getRisingEdge(Controller.XboxButtons.BACK)) {
            drive.useRelativeEncoderPosition = !drive.useRelativeEncoderPosition;
        }

        if (xbox.getRisingEdge(Controller.XboxButtons.START)) {
            useFieldRelative = !useFieldRelative;
        }

    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        killAuto();
        enabled.setBoolean(false);
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        startSubsystems();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    private void startSubsystems() {
        System.out.println("Starting Subsystems");
        robotTracker.start();
        drive.start();
        intake.start();
        hopper.start();
    }

    public synchronized void killAuto() {
        if (selectedAuto != null) {
            selectedAuto.killSwitch();
        }

        if (selectedAuto != null) {
            //auto.interrupt();
            //while(!auto.isInterrupted());
            assert autoThread != null;
            while (autoThread.getState() != Thread.State.TERMINATED) OrangeUtility.sleep(10);

            drive.stopMovement();
            drive.setTeleop();
        }
    }

    @Override
    public void simulationInit() {
        ClassInformationSender.updateReflectionInformation(
                new File(OsUtil.getUserConfigDirectory("AutoBuilder") + "/robotCodeData.json"));
    }
}
