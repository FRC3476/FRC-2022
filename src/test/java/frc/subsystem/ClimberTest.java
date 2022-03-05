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

    @Disabled
    void waitConditionLogTest() {
        frc.utility.Timer.setTime(1000);
        System.out.println("test\n");
        System.out.println(Climber.getInstance().getClimbState());
        System.out.println(Climber.getInstance().getClimbState().stepByStep.waitCondition.apply(Climber.getInstance()));
    }
}
