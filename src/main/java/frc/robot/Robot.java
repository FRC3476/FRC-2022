// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.reflection.ClassInformationSender;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import com.dacubeking.AutoBuilder.robot.serialization.Serializer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystem.*;
import frc.subsystem.Climber.ClimbState;
import frc.subsystem.Hopper.HopperState;
import frc.utility.Controller;
import frc.utility.Controller.XboxButtons;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.Limelight.LedMode;
import frc.utility.Limelight.StreamingMode;
import frc.utility.OrangeUtility;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.ShooterPreset;
import frc.utility.shooter.visionlookup.VisionLookUpTable;
import orangeloggerlib.OrangeLogger;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.util.Collections;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;

import static frc.robot.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public class Robot extends TimedRobot {

    public boolean useFieldRelative = false;
    private double hoodEjectUntilTime = 0;

    //GUI
    final @NotNull NetworkTableInstance instance = NetworkTableInstance.getDefault();
    final @NotNull NetworkTable autoDataTable = instance.getTable("autodata");
    final @NotNull NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");
    final @NotNull NetworkTableEntry enabled = autoDataTable.getEntry("enabled");

    final @NotNull NetworkTableEntry shooterConfigEntry = instance.getTable("limelightgui").getEntry("shooterconfig");
    final @NotNull NetworkTableEntry shooterConfigStatusEntry = instance.getTable("limelightgui").getEntry("shooterconfigStatus");
    final @NotNull NetworkTableEntry shooterConfigStatusIdEntry = instance.getTable("limelightgui").getEntry(
            "shooterconfigStatusId");



    final @NotNull ExecutorService deserializerExecutor = Executors.newSingleThreadExecutor();

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public static final SendableChooser<String> sideChooser = new SendableChooser<>();

    //Inputs
    private final static Controller xbox = new Controller(0);
    private final static Controller stick = new Controller(1);
    private final static Controller buttonPanel = new Controller(2);

    public static void setRumble(RumbleType type, double value) {
        xbox.setRumble(type, value);
    }


    //Control loop states
    boolean limelightTakeSnapshots = false;
    private ShooterPreset shooterPreset = ShooterManager.getInstance().visionLookUpTable.getShooterPreset(0);

    // Input Control
    private double firstPressTime = 0;
    private double lastPressTime = 0;


    Consumer<EntryNotification> shooterGuiListener = event ->
            deserializerExecutor.execute(() -> {
                        //Set networktable entries for the loading circle
                        shooterConfigStatusEntry.setDouble(2);
                        shooterConfigStatusIdEntry.setDouble(shooterConfigStatusIdEntry.getDouble(0) + 1);
                        try {
                            ShooterConfig shooterConfig = (ShooterConfig) Serializer.deserialize(
                                    shooterConfigEntry.getString(null),
                                    ShooterConfig.class);
                            if (shooterConfig.getShooterConfigs().size() < 3) {
                                System.out.println(
                                        "Shooter config was too small to load size=" + shooterConfig.getShooterConfigs().size());
                                return;
                            }
                            Collections.sort(shooterConfig.getShooterConfigs());
                            ShooterManager.getInstance().setShooterConfig(shooterConfig);
                        } catch (IOException e) {
                            //Should never happen. The gui should never upload invalid data.
                            DriverStation.reportError("Failed to deserialize shooter config from networktables", e.getStackTrace());
                        }
                        //Set networktable entries for the loading circle
                        shooterConfigStatusEntry.setDouble(1);
                        shooterConfigStatusIdEntry.setDouble(shooterConfigStatusIdEntry.getDouble(0) + 1);
                    }
            );

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        final RobotTracker robotTracker = RobotTracker.getInstance();
        final Limelight limelight = Limelight.getInstance();
        final Limelight intakeLimelight = Limelight.getInstance(Constants.INTAKE_LIMELIGHT_NAME);
        final Drive drive = Drive.getInstance();

        shooterConfigEntry.addListener(shooterGuiListener,
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);


        SmartDashboard.putBoolean("Field Relative Enabled", useFieldRelative);

        robotTracker.resetGyro();
        OrangeUtility.sleep(50);
        robotTracker.resetPosition(new Pose2d());

        startSubsystems();
        AutonomousContainer.getInstance().initialize(
                true,
                new CommandTranslator(
                        drive::setAutoPath,
                        drive::stopMovement,
                        drive::setAutoRotation,
                        drive::isFinished,
                        drive::getAutoElapsedTime,
                        robotTracker::resetPosition,
                        false

                ),
                false,
                null
        );
        AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

        sideChooser.setDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        SmartDashboard.putData("Auto choices", autoChooser);
        SmartDashboard.putData("Red or Blue", sideChooser);

        NetworkTableInstance.getDefault().setUpdateRate(0.05);
        startSubsystems();
        limelight.setLedMode(LedMode.OFF);
        intakeLimelight.setLedMode(LedMode.OFF);
        limelight.setStreamingMode(StreamingMode.STANDARD);

        if (IS_PRACTICE) {
            for (int i = 0; i < 10; i++) {
                System.out.println("USING PRACTICE BOT CONFIG");
            }
        }

//        shooter.homeHood();
//        shooter.setHoodPositionMode(HoodPositionMode.RELATIVE_TO_HOME);

        // Initialize OrangeLogger with following instances
        OrangeLogger.getInstance().initialize(new Object[]{RobotTracker.getInstance(), Drive.getInstance(),
                Intake.getInstance(), Intake.getInstance(), Hopper.getInstance(), Shooter.getInstance(), VisionManager.getInstance(),
                DashboardHandler.getInstance()});
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
        VisionManager visionManager = VisionManager.getInstance();
        SmartDashboard.putNumber("Match Timestamp", DriverStation.getMatchTime());
        if (!DriverStation.isEnabled()) {
            xbox.update();
            stick.update();
            buttonPanel.update();
        }

        if (buttonPanel.getRawButton(8)) {
            visionManager.forceVisionOn(resettingPoseVisionOn);
            visionManager.forceUpdatePose();
        } else {
            visionManager.unForceVisionOn(resettingPoseVisionOn);
        }
    }


    @Override
    public void autonomousInit() {
        final Drive drive = Drive.getInstance();
        startSubsystems();

        if (!Constants.GRAPPLE_CLIMB) {
            Climber.getInstance().configBrake();
        }

        enabled.setBoolean(true);
        drive.configBrake();

        String autoName = autoChooser.getSelected();
        if (autoName == null) {
            autoName = "1ball"; //Default auto if none is selected
        }
        AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.getSelected(), true);
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
        final Drive drive = Drive.getInstance();
        final Hopper hopper = Hopper.getInstance();

        startSubsystems();

        if (!Constants.GRAPPLE_CLIMB) {
            Climber.getInstance().configBrake();
        } else {
            GrappleClimber.getInstance().resetSolenoids();
        }

        enabled.setBoolean(true);
        useFieldRelative = true;
        SmartDashboard.putBoolean("Field Relative Enabled", true);
        drive.useFieldRelative = true;
        SmartDashboard.putBoolean("Drive Field Relative Allowed", true);
        hopper.setEjectOverride(true);
        drive.configBrake();
    }

    private final Object driverForcingVisionOn = new Object();
    private final Object buttonPanelForcingVisionOn = new Object();
    private final Object resettingPoseVisionOn = new Object();

    private double lastDriverToggleIntakeTime = 0;
    private double lastOperatorToggleIntakeTime = 0;

    /*
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        final RobotTracker robotTracker = RobotTracker.getInstance();
        final Drive drive = Drive.getInstance();
        final Hopper hopper = Hopper.getInstance();
        final Intake intake = Intake.getInstance();
        final Shooter shooter = Shooter.getInstance();
        final ShooterManager shooterManager = ShooterManager.getInstance();
        final VisionManager visionManager = VisionManager.getInstance();

        xbox.update();
        stick.update();
        buttonPanel.update();


        // Will terminate climb auto thread if any stick movement happens
        if (!getControllerDriveInputs().equals(NO_MOTION_CONTROLLER_INPUTS)) {
            killAuto();
        }

        // Deploys grapple lineup
        if (Constants.GRAPPLE_CLIMB
                && (stick.getRisingEdge(7) || stick.getRisingEdge(8)) //We just pressed one of the two buttons
                && (stick.getRawButton(7) || stick.getRawButton(8))) // Both buttons are pressed
        {
            GrappleClimber.getInstance().armGrappleClimb();
        }

        // Shooting / Moving control block
        if (xbox.getRawButton(XboxButtons.LEFT_BUMPER)) {
            // If trying to shoot with left bumper (stop and shoot)
            if (isTryingToRunShooterFromButtonPanel()) {
                doManualShot(drive, hopper, shooter, visionManager);
            } else {
                drive.accelerationLimit = AccelerationLimits.STOP_AND_SHOOT;
                shooter.setFeederChecksDisabled(false);
                hopper.setHopperState(Hopper.HopperState.ON);
                shooterManager.stopAndShoot(NO_MOTION_CONTROLLER_INPUTS, useFieldRelative);
            }
        } else if (buttonPanel.getRawButton(6)) {
            // If trying to Hood Eject
            drive.accelerationLimit = AccelerationLimits.NORMAL_DRIVING;
            doShooterEject();
            doNormalDriving();
        } else if (xbox.getRawAxis(2) > 0.1) {
            // If attempting to shoot with left trigger
            shooter.setFeederChecksDisabled(false);
            if (isTryingToRunShooterFromButtonPanel()) {
                //If shooting from button (no vision)
                doManualShot(drive, hopper, shooter, visionManager);
            } else {
                // Do Shooting while moving (using vision)
                visionManager.forceVisionOn(driverForcingVisionOn);
                if (getControllerDriveInputs().equals(NO_MOTION_CONTROLLER_INPUTS)) {
                    drive.accelerationLimit = AccelerationLimits.SHOOT_AND_MOVE;
                    shooterManager.stopAndShoot(getControllerDriveInputs(), useFieldRelative);
                } else {
                    drive.accelerationLimit = AccelerationLimits.STOP_AND_SHOOT;
                    shooterManager.shootAndMove(getControllerDriveInputs(), useFieldRelative);
                }
            }
        } else {
            drive.accelerationLimit = AccelerationLimits.NORMAL_DRIVING;
            if (Timer.getFPGATimestamp() - hopper.getLastBeamBreakOpenTime() > Constants.BEAM_BREAK_EJECT_TIME ||
                    Timer.getFPGATimestamp() < hoodEjectUntilTime) {
                // Eject a ball if the there has been a 3rd ball detected in the hopper for a certain amount of time
                if (Timer.getFPGATimestamp() - hopper.getLastBeamBreakOpenTime() > Constants.BEAM_BREAK_EJECT_TIME
                        && !(Timer.getFPGATimestamp() < hoodEjectUntilTime)) {
                    hoodEjectUntilTime = Timer.getFPGATimestamp() + Constants.MIN_AUTO_EJECT_TIME;
                    hopper.resetBeamBreakOpenTime();
                }
                shooter.setFeederChecksDisabled(false);
                doShooterEject();
            } else {
                // Not trying to do anything else with shooter will stop all action with it
                shooter.setFeederChecksDisabled(false);
                shooter.setFiring(false);
                shooter.setSpeed(0);
            }

            visionManager.unForceVisionOn(driverForcingVisionOn);
            if (GRAPPLE_CLIMB || Climber.getInstance().getClimbState() == ClimbState.IDLE || Climber.getInstance().isPaused()) {
                // If we're climbing don't allow the robot to be driven
                if (usingDPad) {
                    drive.updateTurn(getControllerDriveInputs(), Constants.CLIMB_LINEUP_ANGLE, useFieldRelative, 0);
                } else {
                    doNormalDriving();
                }
            } else {
                // Stop the robot from moving if we're not issuing other commands to the drivebase
                drive.swerveDrive(new ChassisSpeeds(0, 0, 0));
            }
        }

        updateShooterButtonPanels();

        // Intake solenoid control
        if (xbox.getRisingEdge(XboxButtons.B)
                && Timer.getFPGATimestamp() - lastOperatorToggleIntakeTime > Constants.INTAKE_TOGGLE_DEBOUNCE_TIME) {
            intake.setIntakeSolState(intake.getIntakeSolState().invert());
            lastDriverToggleIntakeTime = Timer.getFPGATimestamp();
        }

        if (buttonPanel.getRisingEdge(4)
                && Timer.getFPGATimestamp() - lastDriverToggleIntakeTime > Constants.INTAKE_TOGGLE_DEBOUNCE_TIME) {
            intake.setIntakeSolState(intake.getIntakeSolState().invert());
            lastOperatorToggleIntakeTime = Timer.getFPGATimestamp();
        }

        // Intake and hopper motor control
        if (xbox.getRawAxis(3) > 0.02) {
            // Intake balls
            intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
            if (Timer.getFPGATimestamp() > hoodEjectUntilTime) {
                hopper.setHopperState(Hopper.HopperState.ON);
            } else {
                hopper.setHopperState(HopperState.OFF);
            }
        } else if (xbox.getRawButton(XboxButtons.RIGHT_BUMPER)) {
            // Sets intake to eject without controlling hopper
            intake.setWantedIntakeState(Intake.IntakeState.EJECT);
            hopper.setHopperState(Hopper.HopperState.OFF);
        } else if (stick.getRawButton(1) || buttonPanel.getRawButton(7)) {
            // Eject everything
            intake.setWantedIntakeState(Intake.IntakeState.EJECT);
            hopper.setHopperState(Hopper.HopperState.REVERSE);
            shooter.reverseShooterWheel();
        } else {
            intake.setWantedIntakeState(Intake.IntakeState.OFF);
            if (!((xbox.getRawAxis(2) > 0.02) || xbox.getRawButton(XboxButtons.LEFT_BUMPER))) {
                // Only turn off the hopper if we're not shooting
                hopper.setHopperState(Hopper.HopperState.OFF);
            }
        }

        if (xbox.getRisingEdge(XboxButtons.A)) {
            //Resets the current robot heading to zero. Useful if the heading drifts for some reason
            robotTracker.resetPosition(new Pose2d(robotTracker.getLastEstimatedPoseMeters().getTranslation(), new Rotation2d(0)));
        }

        // Override outtake to always intake
        if (buttonPanel.getRisingEdge(5)) {
            hopper.toggleEjectOverride();
        }

        if (xbox.getRisingEdge(XboxButtons.LEFT_CLICK) || xbox.getRisingEdge(Controller.XboxButtons.START)) {
            useFieldRelative = !useFieldRelative;
            System.out.println("Field relative: " + useFieldRelative);
            SmartDashboard.putBoolean("Field Relative Enabled", useFieldRelative);
        }

//        if (xbox.getRisingEdge(XboxButtons.LEFT_BUMPER)) {
//            shooterManager.adjustShooterHoodBias(-0.5);
//        } else if (xbox.getRisingEdge(XboxButtons.RIGHT_BUMPER)) {
//            shooterManager.adjustShooterHoodBias(0.5);
//        }


        if (!Constants.GRAPPLE_CLIMB) {
            Climber climber = Climber.getInstance();

            if (stick.getRisingEdge(9)) {
                climber.toggleClaw();
            }

            if (stick.getRisingEdge(10)) {
                climber.togglePivot();
            }

            if (stick.getRawButton(11)) {
                climber.setClimberMotor(CLIMBER_MOTOR_MAX_OUTPUT * 0.5);
            } else if (stick.getRawButton(12)) {
                climber.setClimberMotor(-CLIMBER_MOTOR_MAX_OUTPUT * 0.5);
            } else if (stick.getFallingEdge(11) || stick.getFallingEdge(12)) {
                climber.setClimberMotor(0);
            }

            if (buttonPanel.getRisingEdge(12) && buttonPanel.getRawButton(9)) {
                climber.forceAdvanceStep();
            }

            if (buttonPanel.getRisingEdge(9)) {
                if (climber.getClimbState() == ClimbState.IDLE) {
                    climber.startClimb();
                } else {
                    climber.resumeClimb();
                    climber.advanceStep();
                }
            } else if (buttonPanel.getFallingEdge(9)) {
                climber.pauseClimb();
            } else if (buttonPanel.getRawButton(9)) {
                //drive.setSwerveModuleStates(Constants.SWERVE_MODULE_STATE_FORWARD, true);
            }

            if (buttonPanel.getRisingEdge(11)) {
                climber.stopClimb();
            }


            if (buttonPanel.getRisingEdge(10)) {
                climber.setStepByStep(!climber.isStepByStep());
            }

            if (stick.getRisingEdge(3)) {
                climber.deployClimb();
            }
        }

        if (stick.getRisingEdge(8)) {
            hopper.setBeamBreakEnabled(!hopper.isBeamBreakEnabled());
        }

        //TODO: Debug why this does not work
//        if (buttonPanel.getRisingEdge(9)) {
//            limelightTakeSnapshots = !limelightTakeSnapshots;
//            limelight.takeSnapshots(limelightTakeSnapshots);
//            System.out.println("limelight taking snapshots " + limelightTakeSnapshots);
//        }

        // Feeder wheel will not check for shooter speed and hood angle to be correct before
        // enabling when stick button 2 is held down
        if (stick.getRisingEdge(2)) {
            shooter.setFeederChecksDisabled(!shooter.isFeederChecksDisabled());
        }
    }

    private void doManualShot(Drive drive, Hopper hopper, Shooter shooter, VisionManager visionManager) {
        drive.accelerationLimit = AccelerationLimits.NORMAL_DRIVING;
        shooter.setHoodPosition(shooterPreset.getHoodEjectAngle());
        shooter.setSpeed(shooterPreset.getFlywheelSpeed());
        shooter.setFiring(true);
        hopper.setHopperState(HopperState.ON);
        doNormalDriving();

        visionManager.unForceVisionOn(driverForcingVisionOn);
    }

    // Utility methods

    private void doShooterEject() {
        final Shooter shooter = Shooter.getInstance();
        shooter.setFeederChecksDisabled(true);
        shooter.setSpeed(Constants.SHOOTER_EJECT_SPEED);
        shooter.setHoodPosition(Constants.HOOD_EJECT_ANGLE);
        shooter.setFiring(true);
    }

    private void updateShooterButtonPanels() {
        ShooterManager shooterManager = ShooterManager.getInstance();
        if (buttonPanel.getRisingEdge(1)) {
            //driver station shot
            shooterPreset = shooterManager.visionLookUpTable.getShooterPreset(299);
        } else if (buttonPanel.getRisingEdge(2)) {
            // Wall shot (closer wall)
            shooterPreset = shooterManager.visionLookUpTable.getShooterPreset(139);
        } else if (buttonPanel.getRisingEdge(3)) {
            // Fender shot
            shooterPreset = shooterManager.visionLookUpTable.getShooterPreset(40);
        }
    }

    private boolean isTryingToRunShooterFromButtonPanel() {
        return buttonPanel.getRawButton(1) || buttonPanel.getRawButton(2) || buttonPanel.getRawButton(
                3) || buttonPanel.getRawButton(6);
    }

    private static final ControllerDriveInputs NO_MOTION_CONTROLLER_INPUTS = new ControllerDriveInputs(0, 0, 0);

    private boolean usingDPad = false;

    private void doNormalDriving() {
        final Drive drive = Drive.getInstance();

        ControllerDriveInputs controllerDriveInputs = getControllerDriveInputs();

        if (controllerDriveInputs.getX() == 0 && controllerDriveInputs.getY() == 0 && controllerDriveInputs.getRotation() == 0
                && drive.getSpeedSquared() < 0.1) {
            if (xbox.getRawButton(XboxButtons.Y)) {
                drive.doHold();
            } else {
                drive.swerveDrive(NO_MOTION_CONTROLLER_INPUTS);
            }
        } else {
            if (useFieldRelative) {
                drive.swerveDriveFieldRelative(controllerDriveInputs);
            } else {
                drive.swerveDrive(controllerDriveInputs);
            }
        }
    }

    private ControllerDriveInputs getControllerDriveInputs() {
        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            usingDPad = false;
            return new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0), -xbox.getRawAxis(4))
                    .applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs();
        } else if (xbox.getPOV() != -1) {
            double povRads = Math.toRadians(xbox.getPOV() + 180);
            usingDPad = true;
            return new ControllerDriveInputs(-Math.cos(povRads), Math.sin(povRads) * 0.8, -xbox.getRawAxis(0))
                    .applyDeadZone(0, 0, 0.2, 0)
                    .squareInputs().scaleInputs(DRIVE_LOW_SPEED_MOD);
        } else {
            usingDPad = false;
            return new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0), -xbox.getRawAxis(4))
                    .applyDeadZone(0.05, 0.05, 0.2, 0.2).squareInputs();
        }
    }

    private double disabledTime = 0;
    private boolean hasKilledAuto = false;

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        disabledTime = Timer.getFPGATimestamp();
        hasKilledAuto = false;
        Drive.getInstance().configCoast();
        enabled.setBoolean(false);
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
        if (Timer.getFPGATimestamp() - disabledTime > 2.5 && !hasKilledAuto) {
            killAuto();
            hasKilledAuto = true;
        }
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        Drive.getInstance().configCoast();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        xbox.update();
        stick.update();
        buttonPanel.update();

        final Drive drive = Drive.getInstance();
        final Hopper hopper = Hopper.getInstance();
        final Intake intake = Intake.getInstance();
        final Shooter shooter = Shooter.getInstance();
        
        if (!Constants.GRAPPLE_CLIMB) {
            Climber climber = Climber.getInstance();


            // Climber Test Controls
            if (stick.getRisingEdge(9)) {
                climber.toggleClaw();
            }

            if (stick.getRisingEdge(10)) {
                climber.togglePivot();
            }

            if (stick.getRawButton(11)) {
                climber.setClimberMotor(CLIMBER_MOTOR_MAX_OUTPUT * 0.5);
            } else if (stick.getRawButton(12)) {
                climber.setClimberMotor(-CLIMBER_MOTOR_MAX_OUTPUT * 0.5);
            } else if (buttonPanel.getRawButton(9)) {
                climber.stallIntoBottom();
            } else if ((stick.getFallingEdge(11) || stick.getFallingEdge(12) ||
                    buttonPanel.getFallingEdge(9))) {
                climber.setClimberMotor(0);
            }

            if (buttonPanel.getRisingEdge(11)) {
                climber.stopClimb();
            }
        }

        if (xbox.getRawButton(XboxButtons.X) && xbox.getRawButton(XboxButtons.B)) {

            // Makes sure that these two buttons are held down for one second before running process
            if (firstPressTime == 0) {
                // Initializes check
                firstPressTime = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - lastPressTime > Constants.MAX_TIME_NOT_HELD_SEC) {
                // Checks to see if buttons have been let go for longer than max allowed time
                firstPressTime = 0;
            } else if (Timer.getFPGATimestamp() - firstPressTime > Constants.HELD_BUTTON_TIME_THRESHOLD_SEC) {
                // Checks to see if buttons have been held for required amount of time to run process
                firstPressTime = 0;
                drive.setAbsoluteZeros();
            }

            // Updates last press time
            lastPressTime = Timer.getFPGATimestamp();
        }

        if (xbox.getRisingEdge(XboxButtons.A)) {
            shooter.setHoodZero();
        }

        if (buttonPanel.getRisingEdge(5)) {
            intake.selfTest();
        }

        if (buttonPanel.getRisingEdge(6)) {
            hopper.selfTest();
        }

        if (buttonPanel.getRisingEdge(7)) {
            shooter.selfTest();
        }

        if (buttonPanel.getRisingEdge(8) && !Constants.GRAPPLE_CLIMB) {
            Climber.getInstance().selfTest();
        }
    }

    private void startSubsystems() {
        System.out.println("Starting Subsystems");
        RobotTracker.getInstance().start();
        Drive.getInstance().start();
        Intake.getInstance().start();
        Hopper.getInstance().start();
        Shooter.getInstance().start();
        VisionManager.getInstance().start();
        DashboardHandler.getInstance().start();

        if (Constants.GRAPPLE_CLIMB) {
            GrappleClimber.getInstance();
        } else {
            Climber.getInstance().start();
        }
    }

    public void killAuto() {
        AutonomousContainer.getInstance().killAuto();
    }

    @Override
    public void simulationInit() {
        System.out.println("Simulation Init");
        ClassInformationSender.updateReflectionInformation("frc");
        VisionLookUpTable.getInstance().printShooterConfig();
    }
}
