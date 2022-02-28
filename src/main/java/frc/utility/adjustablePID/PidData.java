package frc.utility.adjustablePID;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.jetbrains.annotations.NotNull;

public class PidData implements Comparable<PidData> {
    @JsonProperty
    @NotNull String key;

    @JsonProperty
    double p;

    @JsonProperty
    double i;

    @JsonProperty
    double d;

    @JsonProperty
    double f;

    @JsonProperty
    double iZone;

    @JsonCreator
    public PidData(String key, double p, double i, double d, double f, double iZone) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.iZone = iZone;
    }

    public String getKey() {
        return key;
    }

    public void setKey(String key) {
        this.key = key;
    }

    public double getP() {
        return p;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double getI() {
        return i;
    }

    public void setI(double i) {
        this.i = i;
    }

    public double getD() {
        return d;
    }

    public void setD(double d) {
        this.d = d;
    }

    public double getF() {
        return f;
    }

    public void setF(double f) {
        this.f = f;
    }

    public double getiZone() {
        return iZone;
    }

    public void setiZone(double iZone) {
        this.iZone = iZone;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        PidData pidData = (PidData) o;

        return key.equals(pidData.key);
    }

    @Override
    public int hashCode() {
        return key.hashCode();
    }

    @Override
    public int compareTo(@NotNull PidData pidData) {
        return String.CASE_INSENSITIVE_ORDER.compare(key, pidData.key);
    }
}
