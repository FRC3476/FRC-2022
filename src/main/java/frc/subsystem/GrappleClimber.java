package frc.subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.utility.Pneumatics.getPneumaticsHub;

public class GrappleClimber extends AbstractSubsystem {
    // Singleton Setup
    private static GrappleClimber instance;

    private static final ReentrantReadWriteLock GRAPPLE_INSTANCE_LOCK = new ReentrantReadWriteLock();

    // Safe Lazy Initialization. Initializes itself when first called
    public static @NotNull GrappleClimber getInstance() {

        GRAPPLE_INSTANCE_LOCK.readLock().lock();
        try {
            if (instance != null) {
                return instance;
            }
        } finally {
            GRAPPLE_INSTANCE_LOCK.readLock().unlock();
        }

        GRAPPLE_INSTANCE_LOCK.writeLock().lock();
        try {
            if (!Constants.GRAPPLE_CLIMB) {
                throw new IllegalStateException();
            }

            return Objects.requireNonNullElseGet(instance, () -> instance = new GrappleClimber());
        } finally {
            GRAPPLE_INSTANCE_LOCK.writeLock().unlock();
        }
    }

    /**
     * Comprises the big solenoid that lifts the robot and the small solenoid that lets go of the grapple
     */
    private final @NotNull Solenoid triggerSolenoid;

    /**
     * The small solenoid that retracts to launch the grapple
     */
    private final @NotNull Solenoid armSolenoid;

    /**
     * The small solenoid that retracts to launch the grapple
     */
    private final @NotNull Solenoid bigClimbingSolenoid;

    private GrappleClimber() {
        super(Constants.GRAPPLE_CLIMBER_PERIOD, 1);

        // Creating solenoids
        triggerSolenoid = getPneumaticsHub().makeSolenoid(Constants.GRAPPLE_TRIGGER_SOL_ID);
        armSolenoid = getPneumaticsHub().makeSolenoid(Constants.GRAPPLE_ARM_SOL_ID);
        bigClimbingSolenoid = getPneumaticsHub().makeSolenoid(Constants.BIG_CLIMBER_SOLENOID_ID);
    }

    /**
     * Sets solenoids to opposite state so they can return back to default when about to launch
     */
    public void resetSolenoids() {
        triggerSolenoid.set(true);
        armSolenoid.set(false);
        bigClimbingSolenoid.set(true);
    }

    /**
     * deploys lineup bar and arms grapple
     */
    public void armGrappleClimb() {
        armSolenoid.set(true);
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

    @Override
    public void close() throws Exception {

    }
}
