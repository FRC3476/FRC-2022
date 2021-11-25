package frc.subsystem;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;


import edu.wpi.first.wpilibj.Timer;

public abstract class AbstractSubsystem implements Runnable {
    int period = 50;

    FileWriter logWriter;
    FileReader logReader; 
    ThreadSignal signal = ThreadSignal.PAUSED;
    public String subsystemName;

    public enum ThreadSignal {
        ALIVE, PAUSED, DEAD
    }

    protected static AbstractSubsystem instance;
    /**
     * 
     * @param period The period when calling update
     */
    public AbstractSubsystem(int period) {
        this.period = period;
        if (period != -1)
            new Thread(this).start();
        try {
            String fileTime = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());
            subsystemName = this.getClass().getSimpleName();
            String fileName = subsystemName+""+fileTime;
            logWriter = new FileWriter(fileName);
            logReader = new FileReader(fileName);
            
        } catch (IOException e) {
            //e.printStackTrace();
        }
    }

    public abstract void selfTest();

    public abstract void logData();

    public static AbstractSubsystem getInstance() {
        return instance;
    }
    
    public void pause() {
        signal = ThreadSignal.PAUSED;
    }

    public void kill() {
        signal = ThreadSignal.DEAD;
    }

    public void start() {
        signal = ThreadSignal.ALIVE;
    }
    /**
     * This fuction will be called repeartedly when the thread is alive 
     * The period will be whatever you defined when creating the object
     */
    public abstract void update();

    

    public void run() {
        while(signal != ThreadSignal.DEAD) {
            double startTime = Timer.getFPGATimestamp();
            if(signal == ThreadSignal.ALIVE) {
                update();
                logData();
            }
            double executionTimeMS = (Timer.getFPGATimestamp()-startTime)*1000;
            try { 
                if(period-executionTimeMS > 0){
                    Thread.sleep((long) (period-executionTimeMS));
                }
            } catch(Exception e) {
                System.out.println("Thread sleep failing "  + this.getClass().getName() + " message: " + e.getMessage());
            }
            
        }
    }
}