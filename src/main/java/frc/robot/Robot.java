// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.*;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.OsUtil;
import frc.auton.guiauto.serialization.reflection.ClassInformationSender;
import frc.subsystem.*;
import frc.subsystem.Climber.ClimbState;
import frc.utility.*;
import frc.utility.Controller.XboxButtons;
import frc.utility.Limelight.StreamingMode;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.ShooterPreset;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.File;
import java.io.IOException;
import java.lang.Thread.State;
import java.util.Collections;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public class Robot extends TimedRobot {

    public boolean useFieldRelative = false;

    //GUI
    final @NotNull NetworkTableInstance instance = NetworkTableInstance.getDefault();
    final @NotNull NetworkTable autoDataTable = instance.getTable("autodata");
    final @NotNull NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");
    final @NotNull NetworkTableEntry enabled = autoDataTable.getEntry("enabled");
    final @NotNull NetworkTableEntry pathProcessingStatusEntry = autoDataTable.getEntry("processing");
    final @NotNull NetworkTableEntry pathProcessingStatusIdEntry = autoDataTable.getEntry("processingid");

    final @NotNull NetworkTableEntry shooterConfigEntry = instance.getTable("limelightgui").getEntry("shooterconfig");
    final @NotNull NetworkTableEntry shooterConfigStatusEntry = instance.getTable("limelightgui").getEntry("shooterconfigStatus");
    final @NotNull NetworkTableEntry shooterConfigStatusIdEntry = instance.getTable("limelightgui").getEntry(
            "shooterconfigStatusId");

    private final @NotNull Lock networkAutoLock = new ReentrantLock();
    NetworkAuto networkAuto = null;

    final @NotNull ExecutorService deserializerExecutor = Executors.newSingleThreadExecutor();

    //Auto
    @Nullable TemplateAuto selectedAuto;
    @Nullable Thread autoThread;

    //We block the robot from starting until these are initalized
    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private ShootAndMoveHigh shootAndMoveHigh;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private ShootAndMoveMid shootAndMoveMid;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private ShootAndMoveLow shootAndMoveLow;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private FourBall fourBall;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private FiveBall fiveBall;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private SixBall sixBall;

    @NotNull private static final String SHOOT_AND_MOVE_HIGH = "Shoot and Move High";
    @NotNull private static final String SHOOT_AND_MOVE_MID = "Shoot and Move Mid";
    @NotNull private static final String SHOOT_AND_MOVE_LOW = "Shoot and Move Low";
    @NotNull private static final String FOUR_BALL = "Four Ball";
    @NotNull private static final String FIVE_BALL = "Five Ball";
    @NotNull private static final String SIX_BALL = "Six Ball";

    private static final String RESET_POSE = "Reset Pose";

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
    boolean limelightTakeSnapshots = false;
    private ShooterPreset shooterPreset = VisionManager.getInstance().visionLookUpTable.getShooterPreset(0);
    private boolean autoAimRobot = true;

    // Input Control
    private double firstPressTime = 0;
    private double lastPressTime = 0;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Initialize the autonomous asynchronously so that we can have both threads of the roborio being used to deserialize
        // the autos
        System.out.println("Loading autos");
        CompletableFuture.runAsync(() -> shootAndMoveHigh = new ShootAndMoveHigh()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> shootAndMoveMid = new ShootAndMoveMid()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> shootAndMoveLow = new ShootAndMoveLow()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> fourBall = new FourBall()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> fiveBall = new FiveBall()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> sixBall = new SixBall()).thenRun(this::incrementLoadedAutos);

        SmartDashboard.putBoolean("Field Relative Enabled", useFieldRelative);
        autoChooser.setDefaultOption(SHOOT_AND_MOVE_HIGH, SHOOT_AND_MOVE_HIGH);
        autoChooser.addOption(SHOOT_AND_MOVE_MID, SHOOT_AND_MOVE_MID);
        autoChooser.addOption(SHOOT_AND_MOVE_LOW, SHOOT_AND_MOVE_LOW);
        autoChooser.addOption(FOUR_BALL, FOUR_BALL);
        autoChooser.addOption(FIVE_BALL, FIVE_BALL);
        autoChooser.addOption(SIX_BALL, SIX_BALL);
        autoChooser.addOption(RESET_POSE, RESET_POSE);

        SmartDashboard.putData("Auto choices", autoChooser);
        Limelight.getInstance().setStreamingMode(StreamingMode.PIP_SECONDARY);

        startSubsystems();
        robotTracker.resetGyro();
        OrangeUtility.sleep(50);
        robotTracker.resetPosition(new Pose2d());
        limelight.setLedMode(Limelight.LedMode.OFF);

        while (loadingAutos) {
            Thread.onSpinWait();
        }
        System.out.println("Finished loading autos");

        autoPath.addListener(event ->
                deserializerExecutor.execute(() -> { //Start deserializing on another thread
                            System.out.println("starting to parse autonomous");
                            //Set networktable entries for the gui notifications
                            pathProcessingStatusEntry.setDouble(1);
                            pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
                            networkAutoLock.lock();
                            try {
                                networkAuto = new NetworkAuto(); //Create the auto object which will start deserializing the json and the auto
                            } finally {
                                networkAutoLock.unlock();
                            }

                            // ready to be run
                            System.out.println("done parsing autonomous");
                            //Set networktable entries for the gui notifications
                            pathProcessingStatusEntry.setDouble(2);
                            pathProcessingStatusIdEntry.setDouble(pathProcessingStatusIdEntry.getDouble(0) + 1);
                        }
                ), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        shooterConfigEntry.addListener(event ->
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
                        }
                ), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

//        shooter.homeHood();
//        shooter.setHoodPositionMode(HoodPositionMode.RELATIVE_TO_HOME);

        NetworkTableInstance.getDefault().setUpdateRate(0.05);
    }


    private final AtomicInteger loadedAutos = new AtomicInteger(0);
    volatile boolean loadingAutos = true;

    public void incrementLoadedAutos() {
        if (loadedAutos.incrementAndGet() == 6) {
            loadingAutos = false;
        }
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
        SmartDashboard.putNumber("Match Timestamp", DriverStation.getMatchTime());
    }


    @Override
    public void autonomousInit() {
        enabled.setBoolean(true);
        drive.configBrake();

        networkAutoLock.lock();
        try {
            if (networkAuto == null) {
                System.out.println("Using normal autos");
                String auto = autoChooser.getSelected();
                switch (auto) {
                    case SHOOT_AND_MOVE_HIGH:
                        selectedAuto = shootAndMoveHigh;
                        break;
                    case SHOOT_AND_MOVE_MID:
                        selectedAuto = shootAndMoveMid;
                        break;
                    case FOUR_BALL:
                        selectedAuto = fourBall;
                        break;
                    case FIVE_BALL:
                        selectedAuto = fiveBall;
                        break;
                    case SIX_BALL:
                        selectedAuto = sixBall;
                        break;
                    case RESET_POSE:
                        selectedAuto = new SetPositionCenter();
                        break;
                    default:
                        selectedAuto = shootAndMoveLow;
                        break;
                }
            } else {
                System.out.println("Using autos from network tables");
                selectedAuto = networkAuto;
            }
        } finally {
            networkAutoLock.unlock();
        }

        assert selectedAuto != null;
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
        enabled.setBoolean(true);
        useFieldRelative = true;
        SmartDashboard.putBoolean("Field Relative Enabled", true);
        drive.useFieldRelative = true;
        SmartDashboard.putBoolean("Drive Field Relative Allowed", true);
        drive.configBrake();
    }

    private final Object driverForcingVisionOn = new Object();
    private final Object buttonPanelForcingVisionOn = new Object();
    private final Object resettingPoseVisionOn = new Object();

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        xbox.update();
        stick.update();
        buttonPanel.update();

        if (xbox.getRawAxis(2) > 0.1) {
            if (!autoAimRobot && isTryingToRunShooterFromButtonPanel()) { //If vision is off
                shooter.setFiring(true);
                hopper.setHopperState(Hopper.HopperState.ON);
                doNormalDriving();
                visionManager.unForceVisionOn(driverForcingVisionOn);
            } else {
                visionManager.forceVisionOn(driverForcingVisionOn);
                if (shooterControlState == ShooterControlState.VELOCITY_COMPENSATED) {
                    visionManager.shootAndMove(getControllerDriveInputs(), useFieldRelative);
                } else {
                    visionManager.autoTurnRobotToTarget(getControllerDriveInputs(), useFieldRelative);
                }
            }
        } else {
            shooter.setFiring(false);
            visionManager.unForceVisionOn(driverForcingVisionOn);
            if (!buttonPanel.getRawButton(11)) { // If we're climbing don't allow the robot to be driven
                doNormalDriving();
            }
        }

        runShooter();

        if (xbox.getRisingEdge(Controller.XboxButtons.B) || buttonPanel.getRisingEdge(4)) {
            intake.setIntakeSolState(intake.getIntakeSolState() == Intake.IntakeSolState.OPEN ?
                    Intake.IntakeSolState.CLOSE : Intake.IntakeSolState.OPEN);
        }

        if (xbox.getRawAxis(3) > 0.1) {
            // Intake balls
            intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
            hopper.setHopperState(Hopper.HopperState.ON);
        } else if (buttonPanel.getRawButton(8)) {
            // Sets intake to eject without controlling hopper
            intake.setWantedIntakeState(Intake.IntakeState.EJECT);
            hopper.setHopperState(Hopper.HopperState.OFF);
        } else if (stick.getRawButton(1)) {
            // Eject everything
            intake.setWantedIntakeState(Intake.IntakeState.EJECT);
            hopper.setHopperState(Hopper.HopperState.REVERSE);
            shooter.reverseShooterWheel();
        } else {
            intake.setWantedIntakeState(Intake.IntakeState.OFF);
            if (!(xbox.getRawAxis(2) > 0.1)) { // Only turn off the hopper if we're not shooting
                hopper.setHopperState(Hopper.HopperState.OFF);
            }
        }

        if (xbox.getRisingEdge(XboxButtons.A)) {
            //Resets the current robot heading to zero. Useful if the heading drifts for some reason
            robotTracker.resetPosition(new Pose2d(robotTracker.getLastEstimatedPoseMeters().getTranslation(), new Rotation2d(0)));
        }

        if (xbox.getRisingEdge(Controller.XboxButtons.START)) {
            useFieldRelative = !useFieldRelative;
            System.out.println("Field relative: " + useFieldRelative);
            SmartDashboard.putBoolean("Field Relative Enabled", useFieldRelative);
        }

        if (stick.getRawButton(4)) {
            visionManager.forceVisionOn(resettingPoseVisionOn);
            visionManager.forceUpdatePose();
        } else {
            visionManager.unForceVisionOn(resettingPoseVisionOn);
        }

        if ((shooter.getShooterState() == Shooter.ShooterState.OFF)) {
            if (limelight.isConnected()) { // Simple status indicator that shows if the limelight is connected or not
                blinkinLED.setColor(0.77);
            } else {
                blinkinLED.setColor(0.61);
            }
        }

//        if (xbox.getRisingEdge(XboxButtons.LEFT_BUMPER)) {
//            visionManager.adjustShooterHoodBias(-0.5);
//        } else if (xbox.getRisingEdge(XboxButtons.RIGHT_BUMPER)) {
//            visionManager.adjustShooterHoodBias(0.5);
//        }

        if (stick.getRisingEdge(9)) {
            climber.toggleClaw();
        }

        if (stick.getRisingEdge(10)) {
            climber.togglePivot();
        }

        if (stick.getRawButton(11)) {
            System.out.println("going up");
            climber.setClimberMotor(Constants.CLIMBER_MOTOR_MAX_OUTPUT);
        } else if (stick.getRawButton(12)) {
            System.out.println("going down");
            climber.setClimberMotor(-Constants.CLIMBER_MOTOR_MAX_OUTPUT);
        } else if (stick.getFallingEdge(11) || stick.getFallingEdge(12)) {
            climber.setClimberMotor(0);
        }

        if (stick.getRisingEdge(7) && stick.getRawButton(8)) {
            climber.forceAdvanceStep();
        }

        if (buttonPanel.getRisingEdge(9)) {
            if (climber.getClimbStatePair() == ClimbState.IDLE) {
                climber.startClimb();
            } else {
                climber.resumeClimb();
                climber.advanceStep();
            }
        } else if (buttonPanel.getFallingEdge(9)) {
            climber.pauseClimb();
        } else if (buttonPanel.getRawButton(9)) {
            drive.setSwerveModuleStates(Constants.SWERVE_MODULE_STATE_FORWARD, true);
        }

        if (buttonPanel.getRisingEdge(12)) {
            climber.stopClimb();
        }

//        if (buttonPanel.getRisingEdge(10)) {
//            climber.setStepByStep(!climber.isStepByStep());
//        }

        if (stick.getRisingEdge(3)) {
            climber.deployClimb();
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

    private enum ShooterControlState {
        VELOCITY_COMPENSATED, STATIC_POSE, MANUAL
    }

    ShooterControlState shooterControlState = ShooterControlState.STATIC_POSE;

    private void runShooter() {
        if (buttonPanel.getRisingEdge(1)) {
            shooterPreset = visionManager.visionLookUpTable.getShooterPreset(400);
            autoAimRobot = true;
        } else if (buttonPanel.getRisingEdge(2)) {
            shooterPreset = visionManager.visionLookUpTable.getShooterPreset(150);
            autoAimRobot = true;
        } else if (buttonPanel.getRisingEdge(3)) {
            shooterPreset = visionManager.visionLookUpTable.getShooterPreset(0);
            autoAimRobot = false;
        }

        if (buttonPanel.getRawButton(7)) {
            shooterControlState = ShooterControlState.VELOCITY_COMPENSATED;
        } else if (buttonPanel.getRawButton(6)) {
            shooterControlState = ShooterControlState.STATIC_POSE;
        } else if (buttonPanel.getRawButton(5)) {
            shooterControlState = ShooterControlState.MANUAL;
        }

        SmartDashboard.putString("Shooter Control State", shooterControlState.toString());

        if (isTryingToRunShooterFromButtonPanel()) {
            // We want the flywheel to be on
            switch (shooterControlState) {
                case VELOCITY_COMPENSATED:
                    // Turns Shooter flywheel on considering a moving robot
                    visionManager.forceVisionOn(buttonPanelForcingVisionOn);
                    visionManager.updateShooterState();
                    break;
                case STATIC_POSE:
                    //Turn Shooter Flywheel On and sets the flywheel speed considering a stationary robot
                    visionManager.forceVisionOn(buttonPanelForcingVisionOn);
                    visionManager.updateShooterStateStaticPose();
                    break;
                case MANUAL:
                    //Turn shooter flywheel on with manuel settings
                    if (buttonPanel.getRawButton(7) || buttonPanel.getRawButton(6) || buttonPanel.getRawButton(5)) {
                        visionManager.unForceVisionOn(buttonPanelForcingVisionOn);
                        shooter.setHoodPosition(shooterPreset.getHoodEjectAngle());
                        shooter.setSpeed(shooterPreset.getFlywheelSpeed());
                    } else {
                        visionManager.forceVisionOn(buttonPanelForcingVisionOn);
                        visionManager.updateShooterStateStaticPose();
                    }
                    break;
            }
        } else {
            visionManager.unForceVisionOn(buttonPanelForcingVisionOn);
            shooter.setSpeed(0); //Turns off shooter flywheel
        }
    }

    private boolean isTryingToRunShooterFromButtonPanel() {
        return xbox.getRawAxis(2) > 0.1 || buttonPanel.getRawButton(7) || buttonPanel.getRawButton(6)
                || buttonPanel.getRawButton(5);
    }

    private static final ControllerDriveInputs NO_MOTION_CONTROLLER_INPUTS = new ControllerDriveInputs(0, 0, 0);

    private void doNormalDriving() {
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
                if (xbox.getRawButton(XboxButtons.BACK)) {
                    double angle = Math.atan2(controllerDriveInputs.getY(), controllerDriveInputs.getX());
                    drive.updateTurn(controllerDriveInputs, new Rotation2d(angle), true);
                } else {
                    drive.swerveDriveFieldRelative(controllerDriveInputs);
                }
            } else {
                drive.swerveDrive(controllerDriveInputs);
            }
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
        drive.configCoast();
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
        drive.configCoast();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        xbox.update();
        stick.update();
        buttonPanel.update();

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

        if (buttonPanel.getRisingEdge(8)) {
            climber.selfTest();
        }
    }

    private void startSubsystems() {
        System.out.println("Starting Subsystems");
        robotTracker.start();
        drive.start();
        intake.start();
        hopper.start();
        shooter.start();
        climber.start();
        visionManager.start();
    }

    public void killAuto() {
        System.out.println("Killing Auto");
        if (selectedAuto != null) {
            assert autoThread != null;
            System.out.println("2");
            autoThread.interrupt();
            System.out.println("3");
            double nextStackTracePrint = Timer.getFPGATimestamp() + 1;
            while (!(selectedAuto.isFinished() || autoThread.getState() == State.TERMINATED)) {
                if (Timer.getFPGATimestamp() > nextStackTracePrint) {
                    Exception throwable = new Exception(
                            "Waiting for auto to die. selectedAuto.isFinished() = " + selectedAuto.isFinished() +
                                    " autoThread.getState() = " + autoThread.getState());
                    throwable.setStackTrace(autoThread.getStackTrace());
                    throwable.printStackTrace();
                    nextStackTracePrint = Timer.getFPGATimestamp() + 5;
                }


                OrangeUtility.sleep(10);
            }
            System.out.println("4");
            drive.stopMovement();
            System.out.println("5");
            drive.setTeleop();
            System.out.println("6");
        } else {
            System.out.println("Auto is null");
        }
    }

    @Override
    public void simulationInit() {
        ClassInformationSender.updateReflectionInformation(
                new File(OsUtil.getUserConfigDirectory("AutoBuilder") + "/robotCodeData.json"));
    }
}
