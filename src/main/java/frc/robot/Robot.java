// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.*;
import frc.auton.guiauto.NetworkAuto;
import frc.auton.guiauto.serialization.OsUtil;
import frc.auton.guiauto.serialization.reflection.ClassInformationSender;
import frc.subsystem.*;
import frc.subsystem.Climber.ClimbState;
import frc.subsystem.Hopper.HopperState;
import frc.utility.*;
import frc.utility.Controller.XboxButtons;
import frc.utility.Limelight.LedMode;
import frc.utility.Limelight.StreamingMode;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.ShooterPreset;
import frc.utility.shooter.visionlookup.VisionLookUpTable;
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
    final @NotNull NetworkTableEntry pathProcessingStatusEntry = autoDataTable.getEntry("processing");
    final @NotNull NetworkTableEntry pathProcessingStatusIdEntry = autoDataTable.getEntry("processingid");

    final @NotNull NetworkTableEntry shooterConfigEntry = instance.getTable("limelightgui").getEntry("shooterconfig");
    final @NotNull NetworkTableEntry shooterConfigStatusEntry = instance.getTable("limelightgui").getEntry("shooterconfigStatus");
    final @NotNull NetworkTableEntry shooterConfigStatusIdEntry = instance.getTable("limelightgui").getEntry(
            "shooterconfigStatusId");

    final @NotNull NetworkTableEntry selectedAutoFeedback = SmartDashboard.getEntry("Selected Auto Feedback");


    private final @NotNull Lock networkAutoLock = new ReentrantLock();
    NetworkAuto networkAuto = null;

    final @NotNull ExecutorService deserializerExecutor = Executors.newSingleThreadExecutor();

    //Auto
    @Nullable TemplateAuto selectedAuto;
    @Nullable Thread autoThread;

    //We block the robot from starting until these are initialized
    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private ShootAndMoveLeft shootAndMoveLeft;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private ShootAndMoveMid shootAndMoveMid;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private ShootAndMoveRight shootAndMoveRight;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private FourBallBlue fourBallBlue;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private FiveBallBlue fiveBallBlue;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private SixBallBlue sixBallBlue;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private FourBallRed fourBallRed;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private FiveBallRed fiveBallRed;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private SixBallRed sixBallRed;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private BuddyAutoLeft buddyAutoLeft;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private BuddyAutoRight buddyAutoRight;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private HideCargoBlue hideCargoBlue;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private HideCargoRed hideCargoRed;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private BuddyAutoLeftHideBlue buddyAutoLeftHideBlue;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private BuddyAutoLeftHideRed buddyAutoLeftHideRed;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private HangerCargoBlue hangerCargoBlue;

    @SuppressWarnings("NotNullFieldNotInitialized")
    @NotNull
    private HangerCargoRed hangerCargoRed;

    @NotNull private static final String SHOOT_AND_MOVE_LEFT = "Shoot and Move Left";
    @NotNull private static final String SHOOT_AND_MOVE_MID = "Shoot and Move Mid";
    @NotNull private static final String SHOOT_AND_MOVE_RIGHT = "Shoot and Move Right";
    @NotNull private static final String FOUR_BALL = "Four Ball";
    @NotNull private static final String FIVE_BALL = "Five Ball";
    @NotNull private static final String SIX_BALL = "Six Ball";
    @NotNull private static final String BUDDY_AUTO_LEFT = "Buddy Auto Left";
    @NotNull private static final String BUDDY_AUTO_RIGHT = "Buddy Auto Right";
    @NotNull private static final String SHOOT_ONLY_RIGHT = "Shoot Only Right";
    @NotNull private static final String SHOOT_ONLY_MID = "Shoot Only Mid";
    @NotNull private static final String SHOOT_ONLY_LEFT = "Shoot Only Left";
    @NotNull private static final String HIDE_CARGO = "Hide Cargo";
    @NotNull private static final String BUDDY_AUTO_LEFT_HIDE = "Buddy Auto Left Hide";
    @NotNull private static final String HANGER_CARGO = "Hanger Cargo";

    private static final String RESET_POSE = "Reset Pose";

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    @NotNull public static final String RED = "RED";
    @NotNull public static final String BLUE = "BLUE";

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
    private ShooterPreset shooterPreset = VisionManager.getInstance().visionLookUpTable.getShooterPreset(0);

    // Input Control
    private double firstPressTime = 0;
    private double lastPressTime = 0;


    Consumer<EntryNotification> autoPathListener = (event ->
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
            ));

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
                            VisionManager.getInstance().setShooterConfig(shooterConfig);
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


        if (autoPath.getString(null) != null) {
            autoPathListener.accept(new EntryNotification(NetworkTableInstance.getDefault(), 1, 1, "", null, 12));
        }

        if (shooterConfigEntry.getString(null) != null) {
            shooterGuiListener.accept(new EntryNotification(NetworkTableInstance.getDefault(), 1, 1, "", null, 12));
        }

        autoPath.addListener(autoPathListener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        shooterConfigEntry.addListener(shooterGuiListener, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Initialize the autonomous asynchronously so that we can have both threads of the roborio being used to deserialize
        // the autos
        System.out.println("Loading autos");
        long startDeserializeTime = System.currentTimeMillis();
        CompletableFuture.runAsync(() -> shootAndMoveLeft = new ShootAndMoveLeft()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> shootAndMoveMid = new ShootAndMoveMid()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> shootAndMoveRight = new ShootAndMoveRight()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> fourBallBlue = new FourBallBlue()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> fiveBallBlue = new FiveBallBlue()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> sixBallBlue = new SixBallBlue()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> fourBallRed = new FourBallRed()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> fiveBallRed = new FiveBallRed()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> sixBallRed = new SixBallRed()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> buddyAutoLeft = new BuddyAutoLeft()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> hideCargoBlue = new HideCargoBlue()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> hideCargoRed = new HideCargoRed()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> buddyAutoLeftHideBlue = new BuddyAutoLeftHideBlue()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> buddyAutoLeftHideRed = new BuddyAutoLeftHideRed()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> hangerCargoBlue = new HangerCargoBlue()).thenRun(this::incrementLoadedAutos);
        CompletableFuture.runAsync(() -> hangerCargoRed = new HangerCargoRed()).thenRun(this::incrementLoadedAutos);

        SmartDashboard.putBoolean("Field Relative Enabled", useFieldRelative);
        autoChooser.setDefaultOption(SHOOT_AND_MOVE_LEFT, SHOOT_AND_MOVE_LEFT);
        autoChooser.addOption(SHOOT_AND_MOVE_MID, SHOOT_AND_MOVE_MID);
        autoChooser.addOption(SHOOT_AND_MOVE_RIGHT, SHOOT_AND_MOVE_RIGHT);
        autoChooser.addOption(FIVE_BALL, FIVE_BALL);
        autoChooser.addOption(SIX_BALL, SIX_BALL);
        autoChooser.addOption(RESET_POSE, RESET_POSE);
        autoChooser.addOption(BUDDY_AUTO_LEFT, BUDDY_AUTO_LEFT);
        autoChooser.addOption(SHOOT_ONLY_LEFT, SHOOT_ONLY_LEFT);
        autoChooser.addOption(SHOOT_ONLY_MID, SHOOT_ONLY_MID);
        autoChooser.addOption(SHOOT_ONLY_RIGHT, SHOOT_ONLY_RIGHT);
        autoChooser.addOption(HIDE_CARGO, HIDE_CARGO);
        autoChooser.addOption(HANGER_CARGO, HANGER_CARGO);
        autoChooser.addOption(BUDDY_AUTO_LEFT_HIDE, BUDDY_AUTO_LEFT_HIDE);

        sideChooser.setDefaultOption(BLUE, BLUE);
        sideChooser.addOption(RED, RED);

        SmartDashboard.putData("Auto choices", autoChooser);
        SmartDashboard.putData("Red or Blue", sideChooser);

        robotTracker.resetGyro();
        OrangeUtility.sleep(50);
        robotTracker.resetPosition(new Pose2d());

        while (loadingAutos) {
            Thread.onSpinWait();
        }

        System.out.println(
                "Finished loading autos in " + ((double) (System.currentTimeMillis() - startDeserializeTime)) / 1000.0);

        NetworkTableInstance.getDefault().setUpdateRate(0.05);
        Limelight.getInstance().setStreamingMode(StreamingMode.PIP_SECONDARY);
        startSubsystems();
        limelight.setLedMode(LedMode.OFF);
        intakeLimelight.setLedMode(LedMode.OFF);

        if (IS_PRACTICE) {
            for (int i = 0; i < 10; i++) {
                System.out.println("USING PRACTICE BOT CONFIG");
            }
        }

        limelight.setStreamingMode(StreamingMode.STANDARD);
//        shooter.homeHood();
//        shooter.setHoodPositionMode(HoodPositionMode.RELATIVE_TO_HOME);
    }


    private final AtomicInteger loadedAutos = new AtomicInteger(0);
    volatile boolean loadingAutos = true;

    public void incrementLoadedAutos() {
        if (loadedAutos.incrementAndGet() == 16) {
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
        VisionManager visionManager = VisionManager.getInstance();
        SmartDashboard.putNumber("Match Timestamp", DriverStation.getMatchTime());
        if (!DriverStation.isEnabled()) {
            xbox.update();
            stick.update();
            buttonPanel.update();
        }

        if (stick.getRawButton(4)) {
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

        networkAutoLock.lock();
        try {
            if (networkAuto == null) {
                System.out.println("Using normal autos: " + sideChooser.getSelected());
                String auto = autoChooser.getSelected();
                if (sideChooser.getSelected().equals(BLUE)) {
                    switch (auto) {
                        case SHOOT_AND_MOVE_LEFT:
                            selectedAuto = shootAndMoveLeft;
                            break;
                        case SHOOT_AND_MOVE_MID:
                            selectedAuto = shootAndMoveMid;
                            break;
                        case FOUR_BALL:
                            selectedAuto = fourBallBlue;
                            break;
                        case FIVE_BALL:
                            selectedAuto = fiveBallBlue;
                            break;
                        case SIX_BALL:
                            selectedAuto = sixBallBlue;
                            break;
                        case RESET_POSE:
                            selectedAuto = new SetPositionCenter();
                            break;
                        case SHOOT_ONLY_LEFT:
                            selectedAuto = new ShootOnlyLeft();
                            break;
                        case SHOOT_ONLY_MID:
                            selectedAuto = new ShootOnlyMid();
                            break;
                        case SHOOT_ONLY_RIGHT:
                            selectedAuto = new ShootOnlyRight();
                            break;
                        case BUDDY_AUTO_LEFT:
                            selectedAuto = buddyAutoLeft;
                            break;
                        case HIDE_CARGO:
                            selectedAuto = hideCargoBlue;
                            break;
                        case BUDDY_AUTO_LEFT_HIDE:
                            selectedAuto = buddyAutoLeftHideBlue;
                            break;
                        case HANGER_CARGO:
                            selectedAuto = hangerCargoBlue;
                            break;
                        default:
                            selectedAuto = shootAndMoveRight;
                            break;
                    }
                } else {
                    switch (auto) {
                        case SHOOT_AND_MOVE_LEFT:
                            selectedAuto = shootAndMoveLeft;
                            break;
                        case SHOOT_AND_MOVE_MID:
                            selectedAuto = shootAndMoveMid;
                            break;
                        case FOUR_BALL:
                            selectedAuto = fourBallRed;
                            break;
                        case FIVE_BALL:
                            selectedAuto = fiveBallRed;
                            break;
                        case SIX_BALL:
                            selectedAuto = sixBallRed;
                            break;
                        case RESET_POSE:
                            selectedAuto = new SetPositionCenter();
                            break;
                        case SHOOT_ONLY_LEFT:
                            selectedAuto = new ShootOnlyLeft();
                            break;
                        case SHOOT_ONLY_MID:
                            selectedAuto = new ShootOnlyMid();
                            break;
                        case SHOOT_ONLY_RIGHT:
                            selectedAuto = new ShootOnlyRight();
                            break;
                        case BUDDY_AUTO_LEFT:
                            selectedAuto = buddyAutoLeft;
                            break;
                        case HIDE_CARGO:
                            selectedAuto = hideCargoRed;
                            break;
                        case BUDDY_AUTO_LEFT_HIDE:
                            selectedAuto = buddyAutoLeftHideRed;
                            break;
                        case HANGER_CARGO:
                            selectedAuto = hangerCargoRed;
                            break;
                        default:
                            selectedAuto = shootAndMoveRight;
                            break;
                    }
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
        hopper.setEjectOverride(false);
        drive.configBrake();
    }

    private final Object driverForcingVisionOn = new Object();
    private final Object buttonPanelForcingVisionOn = new Object();
    private final Object resettingPoseVisionOn = new Object();

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

        VisionManager visionManager = VisionManager.getInstance();
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
                visionManager.stopAndShoot(NO_MOTION_CONTROLLER_INPUTS, useFieldRelative);
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
                drive.accelerationLimit = AccelerationLimits.SHOOT_AND_MOVE;
                visionManager.forceVisionOn(driverForcingVisionOn);
                visionManager.shootAndMove(getControllerDriveInputs(), useFieldRelative);
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

        runShooter();

        // Intake solenoid control
        if (xbox.getRisingEdge(Controller.XboxButtons.B) || buttonPanel.getRisingEdge(4)) {
            intake.setIntakeSolState(intake.getIntakeSolState() == Intake.IntakeSolState.OPEN ?
                    Intake.IntakeSolState.CLOSE : Intake.IntakeSolState.OPEN);
        }

        // Intake and hopper motor control
        if (xbox.getRawAxis(3) > 0.1) {
            // Intake balls
            intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
            if (Timer.getFPGATimestamp() > hoodEjectUntilTime) {
                hopper.setHopperState(Hopper.HopperState.ON);
            } else {
                hopper.setHopperState(HopperState.OFF);
            }
        } else if (buttonPanel.getRawButton(8) || xbox.getRawButton(XboxButtons.RIGHT_BUMPER)) {
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
            if (!((xbox.getRawAxis(2) > 0.1) || xbox.getRawButton(XboxButtons.LEFT_BUMPER))) {
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
//            visionManager.adjustShooterHoodBias(-0.5);
//        } else if (xbox.getRisingEdge(XboxButtons.RIGHT_BUMPER)) {
//            visionManager.adjustShooterHoodBias(0.5);
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
        drive.doHold();
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

    private void runShooter() {
        VisionManager visionManager = VisionManager.getInstance();
        if (buttonPanel.getRisingEdge(1)) {
            shooterPreset = visionManager.visionLookUpTable.getShooterPreset(299);
        } else if (buttonPanel.getRisingEdge(2)) {
            shooterPreset = visionManager.visionLookUpTable.getShooterPreset(139);
        } else if (buttonPanel.getRisingEdge(3)) {
            shooterPreset = visionManager.visionLookUpTable.getShooterPreset(40);
        }
    }

    private boolean isTryingToRunShooterFromButtonPanel() {
        return buttonPanel.getRawButton(1) || buttonPanel.getRawButton(2) || buttonPanel.getRawButton(
                3) || buttonPanel.getRawButton(6);
    }

    private static final ControllerDriveInputs NO_MOTION_CONTROLLER_INPUTS = new ControllerDriveInputs(0, 0, 0);

    private boolean usingDPad = false;

    private void doNormalDriving() {

        if (autoThread == null || !autoThread.isAlive()) {
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
    }

    private ControllerDriveInputs getControllerDriveInputs() {
        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            usingDPad = false;
            return new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0), -xbox.getRawAxis(4))
                    .applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs();
        } else if (xbox.getPOV() != -1) {
            double povRads = Math.toRadians(xbox.getPOV() + 180);
            usingDPad = true;
            return new ControllerDriveInputs(-Math.cos(povRads), Math.sin(povRads), -xbox.getRawAxis(0))
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
            GrappleClimber.getInstance().start();
        } else {
            Climber.getInstance().start();
        }
    }

    public void killAuto() {
        final Drive drive = Drive.getInstance();

        if (selectedAuto != null) {
            assert autoThread != null;
            autoThread.interrupt();
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
            drive.stopMovement();
            drive.setTeleop();
        }
    }

    @Override
    public void simulationInit() {
        ClassInformationSender.updateReflectionInformation(
                new File(OsUtil.getUserConfigDirectory("AutoBuilder") + "/robotCodeData.json"));
        VisionLookUpTable.getInstance().printShooterConfig();
    }
}
