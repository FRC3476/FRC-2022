package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.command.SendableScript;
import frc.subsystem.Drive;

import java.util.List;

@JsonIgnoreProperties(ignoreUnknown = true)
public class TrajectoryAutonomousStep extends AbstractAutonomousStep {
    private final List<State> states;

    private final List<TimedRotation> rotations;

    @JsonCreator
    public TrajectoryAutonomousStep(@JsonProperty(required = true, value = "states") List<State> states,
                                    @JsonProperty(required = true, value = "rotations") List<TimedRotation> rotations) {
        this.states = states;
        this.rotations = rotations;
    }

    public Trajectory getTrajectory() {
        return new Trajectory(states);
    }

    public List<TimedRotation> getRotations() {
        return rotations;
    }

    @Override
    public void execute(TemplateAuto templateAuto,
                        List<SendableScript> scriptsToExecuteByTime,
                        List<SendableScript> scriptsToExecuteByPercent) {
        //This part of the code will likely need to be customized. This takes the trajectory (output TrajectoryGenerator
        // .generateTrajectory())
        //and sends it to our drive class to be executed.
        //If this is not how your autonomous code work you can change the implementation to fit your needs.
        //You just need to ensure that this thread will be blocked until the path is finished being driven.
        if (!templateAuto.isDead()) { //Check that the auto is not dead
            Drive.getInstance().setAutoPath(getTrajectory()); //Send the auto to our drive class to be executed
            Drive.getInstance().setAutoRotation(rotations.get(0).rotation);
            int rotationIndex = 1; //Start at the second rotation (the first is the starting rotation)
            while (!Drive.getInstance().isFinished()) {
                if (templateAuto.isDead()) {
                    return; //Wait till the auto is done (or exit early if it is killed)
                } else {
                    if (rotationIndex < rotations.size() &&
                            Drive.getInstance().getAutoElapsedTime() > rotations.get(rotationIndex).time) {
                        // We've passed the time for the next rotation
                        Drive.getInstance().setAutoRotation(rotations.get(rotationIndex).rotation); //Set the rotation
                    }
                    Thread.yield(); //Wait for the auto to finish
                }
            }
            Drive.getInstance().stopMovement();
        }
    }

    @Override
    public String toString() {
        return "TrajectoryAutonomousStep{" +
                "m_states=" + states +
                '}';
    }

    @JsonProperty("states")
    public List<State> getStates() {
        return states;
    }
}