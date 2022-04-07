package frc.subsystem;

import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class GrappleClimber extends AbstractSubsystem {
    // Singleton Setup
    private static GrappleClimber instance;

    private static final ReentrantReadWriteLock GRAPPLE_INSTANCE_LOCK = new ReentrantReadWriteLock();

    // Safe Lazy Initialization. Initializes itself when first called
    public static @NotNull GrappleClimber getInstance() {
        if (!Constants.GRAPPLE_CLIMB) {
            throw new IllegalStateException();
        }

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
            return Objects.requireNonNullElseGet(instance, () -> instance = new GrappleClimber());
        } finally {
            GRAPPLE_INSTANCE_LOCK.writeLock().unlock();
        }
    }

    private GrappleClimber() {
        super(Constants.GRAPPLE_CLIMBER_PERIOD, 1);
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
