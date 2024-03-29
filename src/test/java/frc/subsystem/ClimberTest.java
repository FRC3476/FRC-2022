package frc.subsystem;

import edu.wpi.first.util.WPIUtilJNI;
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
        WPIUtilJNI.enableMockTime();
        WPIUtilJNI.setMockTime((long) (1000.0d * 1.0e+9));
        System.out.println("test\n");
        System.out.println(Climber.getInstance().getClimbState());
        System.out.println(Climber.getInstance().getClimbState().climbStep.waitCondition.apply(Climber.getInstance()));
    }
}
