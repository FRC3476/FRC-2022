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

    private Solenoid openSolenoid;
    private Solenoid closedSolenoid;
    private Solenoid lineupSolenoid;
    private Solenoid ropeCoilSolenoid;

    private GrappleClimber() {
        super(Constants.GRAPPLE_CLIMBER_PERIOD, 1);

        openSolenoid = getPneumaticsHub().makeSolenoid(Constants.OPEN_GRAPPLE_SOL_ID);
        closedSolenoid = getPneumaticsHub().makeSolenoid(Constants.CLOSED_GRAPPLE_SOL_ID);
        lineupSolenoid = getPneumaticsHub().makeSolenoid(Constants.LINEUP_GRAPPLE_SOL_ID);
        ropeCoilSolenoid = getPneumaticsHub().makeSolenoid(Constants.ROPE_COIL_GRAPPLE_SOL_ID);
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
