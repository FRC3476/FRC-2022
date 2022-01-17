package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.jetbrains.annotations.NotNull;

import java.util.List;


@JsonIgnoreProperties(ignoreUnknown = true)
public class Autonomous {
    private final List<AbstractAutonomousStep> autonomousSteps;

    @JsonCreator
    public Autonomous(@JsonProperty(required = true, value = "autonomousSteps") List<AbstractAutonomousStep> autonomousSteps) {
        this.autonomousSteps = autonomousSteps;
    }

    @JsonProperty
    public List<AbstractAutonomousStep> getAutonomousSteps() {
        return autonomousSteps;
    }

    @Override
    public @NotNull String toString() {
        return "Autonomous{" +
                "autonomousSteps=" + autonomousSteps +
                '}';
    }
}