package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.Disabled;

public class ClimberTest {


    @Disabled
    void profileTest() {
        Climber climber = Climber.getInstance();
        climber.kill();

        double time = Timer.getFPGATimestamp();
        System.out.println("starting test");

        for (int i = 0; i < 1000000; i++) {
            climber.logData();
            //climber.pushLog();
        }

        System.out.println("Time: " + (Timer.getFPGATimestamp() - time));
        //OrangeUtility.sleep(1000);
    }
}