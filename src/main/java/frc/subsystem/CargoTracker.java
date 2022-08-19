package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.utility.cargotracking.CargoPairing;
import frc.utility.cargotracking.CargoPosition;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.joml.AxisAngle4d;
import org.joml.Vector3d;

import java.util.*;

import static frc.robot.Constants.*;

public class CargoTracker extends AbstractSubsystem {

    public CargoTracker() {
        super(1000);
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

    @NotNull NetworkTableEntry blueCargoTable = NetworkTableInstance.getDefault().getEntry("/realsense/pos_blue");
    @NotNull NetworkTableEntry redCargoTable = NetworkTableInstance.getDefault().getEntry("/realsense/pos_red");
    @NotNull NetworkTableEntry latencyTable = NetworkTableInstance.getDefault().getEntry("/realsense/latency");

    private double lastTimestamp = 0;

    {
        latencyTable.addListener(event -> updateCargoPositions(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    ArrayList<CargoPosition> blueCargoPositions = new ArrayList<>(20);

    private static final double[] NO_DETECTION = {0};
    private static final Vector3d CAMERA_RELATIVE_POSITION = new Vector3d(0, 0, 0);
    public static final double CARGO_RADIUS = 5;

    private static final double CARGO_RADIUS_SQUARED = CARGO_RADIUS * CARGO_RADIUS;

    private void updateCargoPositions() {
        double timestamp = Timer.getFPGATimestamp() - (latencyTable.getDouble(0) / 1000) - Constants.REALSENSE_NETWORK_LATENCY;

        Optional<Pose2d> optionalRobotPose = RobotTracker.getInstance().getPoseAtTime(timestamp);
        if (optionalRobotPose.isEmpty()) {
            return;
        }

        final @NotNull Vector3d[] detectedBlueCargoPositions = getCargoPositions(blueCargoTable.getDoubleArray(NO_DETECTION));
        final @NotNull Vector3d[] detectedRedCargoPositions = getCargoPositions(redCargoTable.getDoubleArray(NO_DETECTION));
        Pose2d robotPose2d = optionalRobotPose.get();
        Vector3d robotPose = new Vector3d(robotPose2d.getTranslation().getX(), 0, robotPose2d.getTranslation().getY());

        AxisAngle4d robotRotation = new AxisAngle4d(robotPose2d.getRotation().getRadians(), 0, 1, 0);
        Vector3d cameraPosition = new Vector3d(CAMERA_RELATIVE_POSITION);
        robotRotation.transform(cameraPosition);
        cameraPosition.add(robotPose);

        // Make the cargo positions relative to the field

        updateCargoPositions(timestamp, detectedBlueCargoPositions, robotRotation, cameraPosition, blueCargoPositions);
        lastTimestamp = timestamp;
    }

    private void updateCargoPositions(double timestamp, @NotNull Vector3d @NotNull [] detectedCargoPositions,
                                      @NotNull AxisAngle4d robotRotation, @NotNull Vector3d cameraPosition,
                                      @NotNull ArrayList<CargoPosition> cargoPositions) {
        for (int i = 0; i < detectedCargoPositions.length; i++) {
            detectedCargoPositions[i] =
                    INTAKE_CAMERA_ROTATION.transform(robotRotation.transform(detectedCargoPositions[i])).add(cameraPosition);
        }

        ArrayList<CargoPairing> cargoPairings = new ArrayList<>(detectedCargoPositions.length * cargoPositions.size());

        ArrayList<Vector3d> unpairedCargoPositions = new ArrayList<>(detectedCargoPositions.length);
        // Update the cargo positions
        for (Vector3d detectedCargoPos : detectedCargoPositions) {
            for (CargoPosition cargoPosition : cargoPositions) {
                if (cargoPosition.position.distanceSquared(detectedCargoPos) < CARGO_RADIUS_SQUARED) {
                    cargoPairings.add(new CargoPairing(cargoPosition, detectedCargoPos));
                }
            }

            unpairedCargoPositions.add(detectedCargoPos);
        }
        Collections.sort(cargoPairings);

        ArrayList<Vector3d> pairedDetectedCargoPositions = new ArrayList<>(detectedCargoPositions.length);
        ArrayList<CargoPosition> pairedMappedCargoPositions = new ArrayList<>(cargoPositions.size());

        for (CargoPairing cargoPairing : cargoPairings) {
            if (pairedDetectedCargoPositions.contains(cargoPairing.detectedCargoPosition) ||
                    pairedMappedCargoPositions.contains(cargoPairing.mappedCargoPosition)) {
                continue; // Either the mapped cargo position or the detected cargo position has already been paired
            }

            pairedDetectedCargoPositions.add(cargoPairing.detectedCargoPosition);
            pairedMappedCargoPositions.add(cargoPairing.mappedCargoPosition);

            // Update the cargo position
            correctPosition(cargoPairing.mappedCargoPosition.position, cargoPairing.detectedCargoPosition, 0.2);
            cargoPairing.mappedCargoPosition.lastUpdateTime = timestamp;
            cargoPairing.mappedCargoPosition.framesTillDeletion = -1;

            // Remove the detected cargo position from the list of unpaired cargo positions
            unpairedCargoPositions.remove(cargoPairing.detectedCargoPosition);
        }

        // Add all the unpaired cargo positions to the list of cargo positions
        for (Vector3d unpairedCargoPosition : unpairedCargoPositions) {
            CargoPosition cargoPosition = new CargoPosition(unpairedCargoPosition, null, timestamp);
            cargoPositions.add(cargoPosition);
        }

        Iterator<CargoPosition> cargoPositionIterator = cargoPositions.iterator();
        while (cargoPositionIterator.hasNext()) {
            CargoPosition cargoPosition = cargoPositionIterator.next();

            if (cargoPosition.framesTillDeletion == 0 || cargoPosition.lastUpdateTime < timestamp - MAX_NO_VISIBILITY_TIME_S) {
                cargoPositionIterator.remove();
                continue;
            }

            if (true) { // cargo is within view of the camera
                // Raytrace to determine occlusion with other cargo/field?
                cargoPosition.framesTillDeletion -= 1;

                if (!pairedMappedCargoPositions.contains(cargoPosition) && cargoPosition.framesTillDeletion < 0) {
                    cargoPosition.framesTillDeletion = MAX_NO_VISIBILITY_FRAMES_WHEN_SHOULD_BE_VISIBLE;
                }
            }
        }

        lastTimestamp = timestamp;
    }

    @Contract(mutates = "param1")
    private static void correctPosition(@NotNull Vector3d originalPosition, @NotNull Vector3d detectedPosition,
                                        double amountToCorrect) {
        double x = originalPosition.x + (detectedPosition.x - originalPosition.y) * amountToCorrect;
        double y = originalPosition.y + (detectedPosition.y - originalPosition.y) * amountToCorrect;
        double z = originalPosition.z + (detectedPosition.z - originalPosition.z) * amountToCorrect;
        originalPosition.set(x, y, z);
    }


    @Contract(pure = true)
    public static Vector3d @NotNull [] getCargoPositions(double @NotNull [] cargoPositions) {
        if (cargoPositions.length % 3 != 1) {
            throw (IllegalArgumentException) new IllegalArgumentException("Invalid cargo position array: " +
                    Arrays.toString(cargoPositions)).fillInStackTrace();
        }
        Vector3d[] cargoPositionsVec = new Vector3d[cargoPositions.length / 3]; // Should round down

        for (int i = 1; i < cargoPositions.length; i += 3) {
            cargoPositionsVec[i / 3] = new Vector3d(cargoPositions[i], cargoPositions[i + 1], cargoPositions[i + 2]);
        }
        return cargoPositionsVec;
    }
}
