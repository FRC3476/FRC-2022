package frc.utility.shooter;

public final class ShooterState {
    public ShooterState(double speed, double angle) {
        this.flywheelSpeed = speed;
        this.angle = angle;
        this.ejectionVelocity = 0;
    }

    final double flywheelSpeed;
    final double angle;
    final double ejectionVelocity;
}
