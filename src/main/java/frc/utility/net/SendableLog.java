package frc.utility.net;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.Map;

public class SendableLog {
    @JsonProperty
    public Map<String, String> log;

    @JsonCreator
    public SendableLog(@JsonProperty Map<String, String> log) {
        this.log = log;
    }
}
