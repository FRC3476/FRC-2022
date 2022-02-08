package frc.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.utility.OrangeUtility;
import frc.utility.Serializer;
import frc.utility.Timer;
import frc.utility.net.DashboardConnection;
import frc.utility.net.PacketHandler;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.net.*;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public final class DashboardHandler extends AbstractSubsystem {

    /**
     * HashMap of IntetAdress to DashboardConnection
     */
    private final HashMap<InetAddress, DashboardConnection> dashboardConnections = new HashMap<>();

    private final HashMap<Character, PacketHandler> packetHandlerHashMap = new HashMap<>();

    {
        packetHandlerHashMap.put('k', packet -> {
            InetAddress address = packet.getAddress();
            if (dashboardConnections.containsKey(address)) {
                synchronized (dashboardConnections) {
                    dashboardConnections.get(address).keepAlive(); // Update the keepalive time
                }
            } else {
                // This dashboard hasn't been connected too yet. We should add it
                try {
                    DatagramSocket socket = new DatagramSocket();
                    socket.connect(address, 5802);
                    synchronized (dashboardConnections) {
                        dashboardConnections.put(address, new DashboardConnection(socket));
                    }
                } catch (SocketException e) {
                    DriverStation.reportError("Failed to create socket for " + address, false);
                }
            }
        });
    }

    private static final Map<String, Object> LOG_DATA_MAP = new ConcurrentHashMap<>();

    private static DashboardHandler instance = new DashboardHandler(Constants.WEB_DASHBOARD_SEND_PERIOD_MS);

    private @Nullable DatagramSocket receiveingSocket;

    public static DashboardHandler getInstance() {
        return instance;
    }

    double nextAllowedErrorTime = 0;

    private DashboardHandler(int period) {
        super(period);

        try {
            receiveingSocket = new DatagramSocket(5802); //Limelight uses port 5800 & 5801
            receiveingSocket.setSoTimeout(10);
        } catch (SocketException e) {
            DriverStation.reportError("Could not create socket for listening for data from web dashboard", false);
        }
    }

    public void log(String key, Object value) {
        synchronized (LOG_DATA_MAP) {
            LOG_DATA_MAP.put(key, value);
        }
    }

    public void pushLog() {
        try {
            String json;
            synchronized (LOG_DATA_MAP) {
                json = Serializer.serializeToString(LOG_DATA_MAP);
            }
            synchronized (dashboardConnections) {
                Iterator<DashboardConnection> iterator = dashboardConnections.values().iterator();
                while (iterator.hasNext()) {
                    DashboardConnection entry = iterator.next();
                    if (entry.timeoutTime > Timer.getFPGATimestamp()) {
                        // If we haven't received a keepalive in a while, remove the connection
                        entry.close();
                        iterator.remove();
                    } else {
                        byte[] bytes = json.getBytes();
                        try {
                            entry.datagramSocket.send(new DatagramPacket(bytes, bytes.length));
                        } catch (PortUnreachableException e) {
                            iterator.remove(); // Remove the connection if it's unreachable
                        }
                    }
                }
            }
        } catch (IOException e) {
            if (nextAllowedErrorTime > Timer.getFPGATimestamp()) { // Don't spam the driver station
                DriverStation.reportError("Could not send data to web dashboard", false);
            } else {
                nextAllowedErrorTime = Timer.getFPGATimestamp() + 5;
            }
        }
    }

    byte[] receivedBytes = new byte[65535];

    private void getDashboardConnections() {
        if (receiveingSocket == null) return;

        try {
            DatagramPacket receivedPacket = new DatagramPacket(receivedBytes, receivedBytes.length);
            try {
                receiveingSocket.receive(receivedPacket); // blocking call
            } catch (SocketTimeoutException e) { // We don't care about timeouts
                return;
            }
            char packetType = (char) receivedPacket.getData()[0];

            if (packetHandlerHashMap.containsKey(packetType)) { // Find a handler for this packet type and call it
                packetHandlerHashMap.get(packetType).handlePacket(receivedPacket);
            }
        } catch (IOException e) {
            if (Timer.getFPGATimestamp() > nextAllowedErrorTime) { // Only report errors once per 5 second
                DriverStation.reportError("Encountered an error while receiving data from the web dashboard",
                        false);
                nextAllowedErrorTime = Timer.getFPGATimestamp() + 5;
            }
        }
        OrangeUtility.sleep(50); //Do we need this?
    }

    @Override
    public void update() {
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
