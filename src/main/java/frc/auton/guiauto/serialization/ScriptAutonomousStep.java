package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.command.SendableScript;
import org.jetbrains.annotations.NotNull;

import java.util.List;


@JsonIgnoreProperties(ignoreUnknown = true)
public class ScriptAutonomousStep extends AbstractAutonomousStep {

    private final SendableScript sendableScript;

    @JsonCreator
    public ScriptAutonomousStep(@JsonProperty(required = true, value = "sendableScript") SendableScript sendableScript) {
        this.sendableScript = sendableScript;
    }

    @Override
    public @NotNull String toString() {
        return "ScriptAutonomousStep{" + "sendableScript='" + sendableScript + '\'' + '}';
    }

    @JsonProperty
    public SendableScript getSendableScript() {
        return sendableScript;
    }

    /**
     * Runs the script
     */
    @Override
    public void execute(@NotNull TemplateAuto templateAuto,
                        @NotNull List<SendableScript> scriptsToExecuteByTime,
                        @NotNull List<SendableScript> scriptsToExecuteByPercent) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;

        if (sendableScript.getDelayType() == SendableScript.DelayType.TIME) {
            scriptsToExecuteByTime.add(sendableScript);
            return;
        }

        if (sendableScript.getDelayType() == SendableScript.DelayType.PERCENT) {
            scriptsToExecuteByPercent.add(sendableScript);
            return;
        }

        if (!sendableScript.execute()) {
            //The sendableScript failed to execute; kill the auto
            Thread.currentThread().interrupt(); // Will kill the auto
        }
    }
}