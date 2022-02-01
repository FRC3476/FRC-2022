// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.OsUtil;
import frc.auton.guiauto.serialization.reflection.ClassInformationSender;
import frc.subsystem.*;
import frc.utility.*;
import frc.utility.Controller.XboxButtons;
import frc.utility.shooter.visionlookup.ShooterConfig;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.File;
import java.io.IOException;
import java.util.Collections;
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

    NetworkTableEntry shooterConfigEntry = instance.getTable("limelightgui").getEntry("shooterconfig");
    NetworkTableEntry shooterConfigStatusEntry = instance.getTable("limelightgui").getEntry("shooterconfigStatus");
    NetworkTableEntry shooterConfigStatusIdEntry = instance.getTable("limelightgui").getEntry("shooterconfigStatusId");

    NetworkAuto networkAuto;
    @Nullable String lastAutoPath = null;
    @Nullable String lastShooterConfig = null;

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
    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();
    private final VisionManager visionManager = VisionManager.getInstance();

    //Inputs
    private final Controller xbox = new Controller(0);
    private final Controller stick = new Controller(1);
    private final Controller buttonPanel = new Controller(2);


    //Control loop states
    boolean limelightTakeSnapshots;
    private double hoodPosition = 55;
    private double shooterSpeed = 4000;
    private boolean visionOn = true;
    private int shooterMode = 1;
    private boolean targetFound = false;

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
        limelight.setLedMode(Limelight.LedMode.OFF);
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

        if (shooterConfigEntry.getString(null) != null && !shooterConfigEntry.getString(null).equals(lastShooterConfig)) {
            lastShooterConfig = shooterConfigEntry.getString(null);
            deserializerExecutor.execute(() -> {
                System.out.println("start parsing shooter config");
                //Set networktable entries for the loading circle
                shooterConfigStatusEntry.setDouble(2);
                shooterConfigStatusIdEntry.setDouble(shooterConfigStatusIdEntry.getDouble(0) + 1);
                try {
                    ShooterConfig shooterConfig = (ShooterConfig) Serializer.deserialize(shooterConfigEntry.getString(null),
                            ShooterConfig.class);
                    Collections.sort(shooterConfig.getShooterConfigs());
                    visionManager.setShooterConfig(shooterConfig);
                    System.out.println(shooterConfig.getShooterConfigs());
                } catch (IOException e) {
                    //Should never happen. The gui should never upload invalid data.
                    DriverStation.reportError("Failed to deserialize shooter config from networktables", e.getStackTrace());
                }

                System.out.println("done parsing shooter config");
                //Set networktable entries for the loading circle
                shooterConfigStatusEntry.setDouble(1);
                shooterConfigStatusIdEntry.setDouble(shooterConfigStatusIdEntry.getDouble(0) + 1);
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

        if (buttonPanel.getRisingEdge(1)) {
            hoodPosition = 25;
            shooterSpeed = 5500;
            visionOn = true;
            shooterMode = 1;
        } else if (buttonPanel.getRisingEdge(2)) {
            hoodPosition = 33;
            visionOn = true;
            shooterSpeed = 5400;
            shooterMode = 2;
        } else if (buttonPanel.getRisingEdge(3)) {
            hoodPosition = 55;
            shooterSpeed = 5000;
            visionOn = false;
            shooterMode = 3;
        }

        if (xbox.getRawAxis(2) > 0.1 || stick.getRawButton(1)) {
            visionManager.forceVisionOn(true);

            if (!visionOn || stick.getRawButton(1)) { //If vision is off, or we're requesting to do a no vision shot
                shooter.setFiring(true);
                hopper.setHopperState(Hopper.HopperState.ON);
                doNormalDriving();
            } else {
                if (buttonPanel.getRawButton(5)) {
                    visionManager.shootAndMove(getControllerDriveInputs());
                } else {
                    visionManager.autoTurnRobotToTarget(getControllerDriveInputs(), useFieldRelative);
                }
            }
        } else {
            shooter.setFiring(false);
            if (!buttonPanel.getRawButton(6) && !buttonPanel.getRawButton(13)) {
                // We're not trying to run the flywheel and not trying to force update the pose
                visionManager.forceVisionOn(false);
            }
            doNormalDriving();
        }

        if (xbox.getRawAxis(3) > 0.1 || stick.getRawButton(2)) {
            if (buttonPanel.getRawButton(7)) {
                // Turns Shooter flywheel on considering a moving robot
                visionManager.forceVisionOn(true);
                visionManager.updateShooterState();
            } else if (buttonPanel.getRawButton(6)) {
                //Turn Shooter Flywheel On and sets the flywheel speed considering a stationary robot
                visionManager.forceVisionOn(true);
                visionManager.updateShooterStateStaticPose();
            } else if (buttonPanel.getRawButton(5)) {
                //Turn shooter flywheel on with manuel settings
                shooter.setShooterSpeed(shooterSpeed);
                shooter.setHoodPosition(hoodPosition);
            } else {
                shooter.setShooterSpeed(0); //Turns off shooter flywheel
                targetFound = false;
            }
        }

        if (xbox.getRisingEdge(Controller.XboxButtons.B) || buttonPanel.getRisingEdge(7)) {
            intake.setIntakeSolState(intake.getIntakeSolState() == Intake.IntakeSolState.OPEN ?
                    Intake.IntakeSolState.CLOSE : Intake.IntakeSolState.OPEN);
        }

        if (xbox.getRawAxis(3) > 0.1) {
            // Intake balls
            intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
            hopper.setHopperState(Hopper.HopperState.ON);
        } else if (buttonPanel.getRawButton(10)) {
            // Eject everything
            intake.setWantedIntakeState(Intake.IntakeState.EJECT);
            hopper.setHopperState(Hopper.HopperState.REVERSE);
        } else {
            intake.setWantedIntakeState(Intake.IntakeState.OFF);
            if (!(xbox.getRawAxis(2) > 0.1 || stick.getRawButton(1))) { // Only turn off the hopper if we're not shooting
                hopper.setHopperState(Hopper.HopperState.OFF);
            }
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

        if (buttonPanel.getRawButton(13)) {
            visionManager.forceVisionOn(true);
            visionManager.forceUpdatePose();
        }

        if ((shooter.getShooterState() == Shooter.ShooterState.OFF)) {
            if (limelight.isConnected()) { // Simple status indicator that shows if the limelight is connected or not
                blinkinLED.setColor(0.77);
            } else {
                blinkinLED.setColor(0.61);
            }
        }

        if (xbox.getRawButton(XboxButtons.LEFT_BUMPER)) {
            visionManager.adjustShooterHoodBias(-0.5);
        } else if (xbox.getRawButton(XboxButtons.RIGHT_BUMPER)) {
            visionManager.adjustShooterHoodBias(0.5);
        }
    }

    private void doNormalDriving() {
        ControllerDriveInputs controllerDriveInputs = getControllerDriveInputs();
        if (useFieldRelative) {
            drive.swerveDriveFieldRelative(controllerDriveInputs);
        } else {
            drive.swerveDrive(controllerDriveInputs);
        }
    }

    private ControllerDriveInputs getControllerDriveInputs() {
        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            return new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0),
                    -xbox.getRawAxis(4)).applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs();
        } else {
            return new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0),
                    -xbox.getRawAxis(4)).applyDeadZone(0.05, 0.05, 0.2, 0.2).squareInputs();
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
