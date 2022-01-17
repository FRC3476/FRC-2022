package frc.auton.guiauto.serialization.command;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.jetbrains.annotations.NotNull;

import java.util.List;


/**
 * Note: this class has a natural ordering that is inconsistent with equals.
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public class SendableScript implements Comparable<SendableScript> {

    /**
     * @return false if the script fails to execute
     */
    public boolean execute() {
        for (SendableCommand command : commands) {
            if (!command.execute()) {
                return false;
            }
        }
        return true;
    }

    public enum DelayType {
        NONE,
        TIME,
        PERCENT
    }

    private DelayType delayType;
    private double delay;

    private final List<SendableCommand> commands;


    @JsonCreator
    public SendableScript(@JsonProperty("delayType") DelayType delayType,
                          @JsonProperty("delay") double delay,
                          @JsonProperty("commands") List<SendableCommand> commands) {
        this.delayType = delayType;
        this.delay = delay;
        this.commands = commands;
    }

    @JsonProperty("delayType")
    public DelayType getDelayType() {
        return delayType;
    }

    @JsonProperty("delay")
    public double getDelay() {
        return delay;
    }

    @JsonProperty("commands")
    public List<SendableCommand> getCommands() {
        return commands;
    }

    public void setDelay(double delay) {
        this.delay = delay;
    }

    public void setDelayType(DelayType delayType) {
        this.delayType = delayType;
    }

    @Override
    public int compareTo(@NotNull SendableScript o) {
        return Double.compare(delay, o.delay);
    }
}
