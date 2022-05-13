package frc.subsystem;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VisionManagerTest {

    @Test
    void getVelocityAdjustedRelativeTranslation() {
        int failures = 0;

        Random random = new Random(719479);
        for (int i = 0; i < 100000; i++) {
            Translation2d goalPos = new Translation2d((random.nextDouble() * 20) - 10, (random.nextDouble() * 20) - 10);
            Translation2d robotVelocity = new Translation2d((random.nextDouble() * 10) - 5, (random.nextDouble() * 10) - 5);
            Translation2d fakeGoal = VisionManager.getInstance().getVelocityAdjustedRelativeTranslation(goalPos, robotVelocity);


            Translation2d recalculatedActual = fakeGoal.minus(
                    robotVelocity.times(VisionManager.getInstance().getTimeOfFlight(fakeGoal)));

            if (recalculatedActual.minus(goalPos).getNorm() > 0.1) {
                failures++;
                System.out.println("FAILURE: " + i + " " +
                        "goalPos: " + goalPos.toString() + " " +
                        "robotVelocity: " + robotVelocity.toString() + " " +
                        "fakeGoal: " + fakeGoal.toString() + " " +
                        "recalculatedActual: " + recalculatedActual.toString() + " " +
                        "difference: " + recalculatedActual.minus(goalPos).toString());
            }

            if (failures > 20) {
                break;
            }
        }
        assertEquals(0, failures);
    }
}