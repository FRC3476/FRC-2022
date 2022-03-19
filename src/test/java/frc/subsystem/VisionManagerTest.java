package frc.subsystem;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utility.OrangeUtility;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VisionManagerTest {

    @Test
    void getVelocityAdjustedRelativeTranslation() {

        Random random = new Random();
        for (int i = 0; i < 100; i++) {
            Translation2d goalPos = new Translation2d((random.nextDouble() * 20) - 10, (random.nextDouble() * 20) - 10);
            Translation2d robotVelocity = new Translation2d((random.nextDouble() * 10) - 5, (random.nextDouble() * 10) - 5);
            Translation2d fakeGoal = VisionManager.getInstance().getVelocityAdjustedRelativeTranslation(goalPos, robotVelocity);


            Translation2d recalculatedActual = fakeGoal.plus(
                    robotVelocity.times(VisionManager.getInstance().getTimeOfFlight(fakeGoal)));

            //System.out.println("Calculated Goal Pos " + recalculatedActual);

            assertEquals(recalculatedActual.getX(), goalPos.getX(), 0.01);
            assertEquals(recalculatedActual.getY(), goalPos.getY(), 0.01);

            OrangeUtility.sleep(100);
        }
    }
}