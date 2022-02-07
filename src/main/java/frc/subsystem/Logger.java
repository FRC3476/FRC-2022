package frc.subsystem;

import frc.robot.Constants;
import frc.utility.Serializer;

import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class Logger extends AbstractSubsystem {

    // Makes Socket, Output stream, and Data Output Stream
    private Socket logSocket;
    private OutputStream outputStream;
    private DataOutputStream dataOutputStream;

    private static final Map<String, Object> logDataMap = new ConcurrentHashMap<>();

    // Singleton setup
    private static Logger instance = new Logger(Constants.WEB_DASHBOARD_SEND_PERIOD_MS);

    public static Logger getInstance() {
        return instance;
    }

    private Logger(int period) {
        super(period);

        // Sets up logging socket
        try {
            // Creates Socket
            logSocket = new Socket(Constants.WEB_DASHBOARD_HOSTNAME, Constants.WEB_DASHBOARD_PORT);

            // Gets the socket's output stream
            outputStream = logSocket.getOutputStream();
        } catch (IOException e) {
            System.out.println("Could Not Connect To Web Dashboard");
            e.printStackTrace();
        }

        // create a data output stream from the output stream so we can send data through it
        dataOutputStream = new DataOutputStream(outputStream);

        System.out.println("Connected To Web Dashboard");
    }

    public void log(String key, Object value) {
        logDataMap.put(key, value);
    }

    public void pushLog() throws IOException {
        // Serializes hashmap into json string
        String json = Serializer.serializeToString(logDataMap);

        // Writes json to data output as a String
        dataOutputStream.writeUTF(json);

        // send the message
        dataOutputStream.flush();
    }

    @Override
    public void update() throws IOException {
        pushLog();
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        
    }

    @Override
    public void close() throws Exception {

    }
}
