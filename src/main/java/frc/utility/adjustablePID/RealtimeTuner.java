package frc.utility.adjustablePID;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utility.Serializer;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * ArrayList wrapper for serialization
 */
class TunerData {
    ArrayList<PidData> data = new ArrayList<>();
}

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

    private static final ExecutorService executorService = Executors.newSingleThreadExecutor();


    /**
     * adds an object to the ArrayList and saves to json returns the index it was placed in
     */
    public static int addValue(double addObj) {
        tunerData.add(addObj);
        return tunerData.indexOf(addObj);
    }

    public static void saveData() {
        CompletableFuture.runAsync(() -> {
            try {
                Serializer.serializeToFile(tunerData, file);
            } catch (IOException e) {
                System.out.println("Could not serialize TunerData to " + filepath);
                e.printStackTrace();
            }
        }, executorService);
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
    public static PidData getTunerData(String key) {
        int index = Collections.binarySearch(tunerData.data, new PidData(key, 0, 0, 0, 0, 0));
        if (index < 0) {
            index = -index - 1;
            tunerData.data.add(index, new PidData(key, 0, 0, 0, 0, 0));
        }
        return tunerData.data.get(index);
    }

    public static void initializeTunerData() {
        if (tunerData == null) {
            try {
                tunerData = (TunerData) (Serializer.deserializeFromFile(file, TunerData.class));
            } catch (IOException e) {
                System.out.println("Could not deserialize TunerData.json from " + filepath);
                tunerData.add(0);
                tunerData.add(0);
                tunerData.add(0);
                tunerData.add(0);
                tunerData.add(0);
            }
        }
    }
}
