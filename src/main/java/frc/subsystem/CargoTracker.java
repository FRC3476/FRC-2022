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
import org.joml.Vector2d;
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

    ArrayList<CargoPosition> blueCargoPositions = new ArrayList<>(30);
    ArrayList<CargoPosition> redCargoPositions = new ArrayList<>(30);
    private static final double[] NO_DETECTION = {0};
    private static final Vector3d CAMERA_RELATIVE_POSITION = new Vector3d(0.2, 0.5, 0);
    public static final double CARGO_RADIUS = 5;

    private static final double CARGO_RADIUS_SQUARED = CARGO_RADIUS * CARGO_RADIUS;

    private static final Vector2d[] CENTER_FIELD_GEOMETRY = {
            new Vector2d(6.9270, -0.3846),
            new Vector2d(7.2446, -0.2393),
            new Vector2d(7.6512, 0.84890),
            new Vector2d(7.5067, 1.18470),
            new Vector2d(7.8602, 1.33040),
            new Vector2d(8.0020, 1.00600),
            new Vector2d(9.1144, 0.59000),
            new Vector2d(9.4292, 0.72970),
            new Vector2d(9.5957, 0.39600),
            new Vector2d(9.2665, 0.25170),
            new Vector2d(8.8336, -0.8476),
            new Vector2d(8.9712, -1.1572),
            new Vector2d(8.6237, -1.3166),
            new Vector2d(8.4980, -1.0302),
            new Vector2d(7.3787, -0.5838)
    };


    private void updateCargoPositions() {
        double timestamp = Timer.getFPGATimestamp() - (latencyTable.getDouble(0) / 1000) - Constants.REALSENSE_NETWORK_LATENCY;

        Optional<Pose2d> optionalRobotPose = RobotTracker.getInstance().getPoseAtTime(timestamp);
        if (optionalRobotPose.isEmpty()) {
            return;
        }

        final @NotNull Vector3d[] detectedBlueCargoPositions = getCargoPositions(blueCargoTable.getDoubleArray(NO_DETECTION));
        final @NotNull Vector3d[] detectedRedCargoPositions = getCargoPositions(redCargoTable.getDoubleArray(NO_DETECTION));
        Pose2d robotPose2d = optionalRobotPose.get();
        Vector3d robotTranslation = new Vector3d(robotPose2d.getTranslation().getX(), 0, robotPose2d.getTranslation().getY());

        AxisAngle4d robotRotation = new AxisAngle4d(robotPose2d.getRotation().getRadians(), 0, 1, 0);
        Vector3d cameraPosition = new Vector3d(CAMERA_RELATIVE_POSITION); // copy
        robotRotation.transform(cameraPosition); // rotate
        cameraPosition.add(robotTranslation); // make relative to the field

        updateCargoPositions(timestamp, detectedBlueCargoPositions, robotRotation, cameraPosition, blueCargoPositions);
        updateCargoPositions(timestamp, detectedRedCargoPositions, robotRotation, cameraPosition, redCargoPositions);
        lastTimestamp = timestamp;
    }

    private void updateCargoPositions(double timestamp, @NotNull Vector3d @NotNull [] detectedCargoPositions,
                                      @NotNull AxisAngle4d robotRotation, @NotNull Vector3d cameraPositionRelativeToField,
                                      @NotNull ArrayList<CargoPosition> cargoPositions) {
        // Make the cargo positions relative to the field
        for (Vector3d detectedCargoPosition : detectedCargoPositions) {
            robotRotation.transform(INTAKE_CAMERA_ROTATION_INVERSE.transform(detectedCargoPosition))
                    .add(cameraPositionRelativeToField);
        }

        // Initial size to avoid resizing
        ArrayList<CargoPairing> cargoPairings = new ArrayList<>(detectedCargoPositions.length * cargoPositions.size());

        ArrayList<Vector3d> unpairedCargoPositions = new ArrayList<>(detectedCargoPositions.length);
        // Update the cargo positions
        for (Vector3d detectedCargoPosition : detectedCargoPositions) {
            if (detectedCargoPosition.y < MAX_CARGO_Y) {
                for (CargoPosition cargoPosition : cargoPositions) {
                    if (cargoPosition.position.distanceSquared(detectedCargoPosition) < CARGO_RADIUS_SQUARED) {
                        cargoPairings.add(new CargoPairing(cargoPosition, detectedCargoPosition));
                    }
                }

                unpairedCargoPositions.add(detectedCargoPosition);
            }
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


    /**
     * Gets the cargo positions from the network table array of floats.
     *
     * @param cargoPositions The array of floats from the network table. The first index should be either 0 or 1, where 0 means
     *                       that there is no cargo detected and 1 means that there is cargo detected. The rest of the array
     *                       should be in groups of 3, where each group is the x, y, and z position of the cargo.
     * @return The array of cargo positions as Vector3d objects. The array will be empty if there is no cargo detected.
     */
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
