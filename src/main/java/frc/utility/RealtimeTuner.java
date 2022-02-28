package frc.utility;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

/**
 * ArrayList wrapper for serialization
 */
class TunerData extends ArrayList<Object> {}

/**
 * Allows for modification and saving of values through shuffleboard Make sure to log the data to shuffleboard in order to modify
 * it
 */
public class RealtimeTuner {

    private static final String filepath = "RealtimeTunerData/TunerData.json";
    private static final File file = new File(filepath);
    private static TunerData tunerData;
    private static boolean initializeArraylist = true;

    static final @NotNull NetworkTableInstance instance = NetworkTableInstance.getDefault();
    static final @NotNull NetworkTable tunerDataTable = instance.getTable("tunerData");

    /**
     * adds an object to the ArrayList and saves to json returns the index it was placed in
     */
    public static int addValue(Object addObj) {
        tunerData.add(addObj);
        try {
            Serializer.serializeToFile(tunerData, file);
        } catch (IOException e) {
            System.out.println("Could not serialize TunerData to " + filepath);
            e.printStackTrace();
        }

        return tunerData.indexOf(addObj);
    }

    /**
     * updates the value of object in arrayList, should be put in subsystem update loops
     */
    public static void updateValue(Object updateObj, int index) {
        // Will only change value if it is different
        if (tunerData.get(index) != updateObj) {
            tunerData.set(index, updateObj);
            try {
                Serializer.serializeToFile(tunerData, file);
            } catch (IOException e) {
                System.out.println("Could not serialize TunerData to " + filepath);
                e.printStackTrace();
            }
        }
    }

    /**
     * retrieves value from ArrayList from json and returns it
     */
    public static Object getTunerData(int index) {

        // On first
        if (initializeArraylist) {
            try {
                tunerData = (TunerData) (Serializer.deserializeFromFile(file, TunerData.class));
            } catch (IOException e) {
                System.out.println("Could not deserialize TunerData.json from " + filepath);
                e.printStackTrace();
            }
            initializeArraylist = false;
        }
        return tunerData.get(index);
    }
}
