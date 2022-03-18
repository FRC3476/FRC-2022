package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.auton.guiauto.serialization.command.CommandExecutionFailedException;
import frc.auton.guiauto.serialization.command.SendableScript;

import java.util.List;

@JsonTypeInfo(use = JsonTypeInfo.Id.NAME,
        include = JsonTypeInfo.As.PROPERTY,
        property = "type")
@JsonSubTypes({
        @Type(value = TrajectoryAutonomousStep.class, name = "trajectory"),
        @Type(value = ScriptAutonomousStep.class, name = "script"),
})
@JsonIgnoreProperties(ignoreUnknown = true)
public abstract class AbstractAutonomousStep {

    @JsonCreator
    protected AbstractAutonomousStep() {
    }

    public abstract void execute(List<SendableScript> scriptsToExecuteByTime,
                                 List<SendableScript> scriptsToExecuteByPercent) throws InterruptedException, CommandExecutionFailedException;


}