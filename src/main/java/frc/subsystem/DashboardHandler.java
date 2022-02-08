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
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.robot.Constants.WEB_DASHBOARD_PORT;

public final class DashboardHandler extends AbstractSubsystem {

    private @Nullable DatagramSocket receiveingSocket;
    private double nextAllowedErrorTime = 0;

    /**
     * HashMap of IntetAdress to DashboardConnection
     */
    private final HashMap<InetAddress, DashboardConnection> dashboardConnections = new HashMap<>();
    private static final Map<String, Object> LOG_DATA_MAP = new ConcurrentHashMap<>();
    private static final ReadWriteLock LOG_DATA_MAP_LOCK = new ReentrantReadWriteLock();
    private final HashMap<Character, PacketHandler> packetHandlerMap = new HashMap<>();

    {
        packetHandlerMap.put('k', packet -> {
            InetAddress address = packet.getAddress();
            if (dashboardConnections.containsKey(address)) {
                synchronized (dashboardConnections) {
                    dashboardConnections.get(address).keepAlive(); // Update the keepalive time
                }
            } else {
                // This dashboard hasn't been connected too yet. We should add it to the list of connected dashboards
                try {
                    DatagramSocket socket = new DatagramSocket();
                    socket.connect(address, WEB_DASHBOARD_PORT);
                    synchronized (dashboardConnections) {
                        dashboardConnections.put(address, new DashboardConnection(socket));
                    }
                } catch (SocketException e) {
                    DriverStation.reportError("Failed to create socket for " + address, false);
                }
            }
        });
    }

    private static DashboardHandler instance = new DashboardHandler(Constants.WEB_DASHBOARD_SEND_PERIOD_MS);

    public static DashboardHandler getInstance() {
        return instance;
    }


    private DashboardHandler(int period) {
        super(period);

        try {
            receiveingSocket = new DatagramSocket(WEB_DASHBOARD_PORT);
            receiveingSocket.setSoTimeout(10);
        } catch (SocketException e) {
            DriverStation.reportError("Could not create socket for listening for data from web dashboard", false);
        }
    }
    public void log(String key, Object value) {
        LOG_DATA_MAP_LOCK.readLock().lock();
        // We're adding a read lock here because there can only be one writer, but there can be many readers. The
        // ConcurrentHashMap is thread-safe and will handle the synchronization with multiple writes for us. We then only put a
        // write lock when reading it to prevent any other writes from happening while we're serializing.
        try {
            LOG_DATA_MAP.put(key, value);
        } finally {
            LOG_DATA_MAP_LOCK.readLock().unlock();
        }
    }

    public void pushLog() {
        try {
            String json;
            LOG_DATA_MAP_LOCK.writeLock().lock(); // Ensure that no other writes happen while we're serializing
            try {
                json = Serializer.serializeToString(LOG_DATA_MAP);
            } finally {
                LOG_DATA_MAP_LOCK.writeLock().unlock();
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

            if (packetHandlerMap.containsKey(packetType)) { // Find a handler for this packet type and call it
                packetHandlerMap.get(packetType).handlePacket(receivedPacket);
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
        getDashboardConnections();
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
