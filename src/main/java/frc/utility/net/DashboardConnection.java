package frc.utility.net;

import java.io.Closeable;
import java.net.DatagramSocket;

public class DashboardConnection implements Closeable {
    public final DatagramSocket datagramSocket;
    public long timeoutTime;

    public DashboardConnection(DatagramSocket datagramSocket, long timeoutTime) {
        this.datagramSocket = datagramSocket;
        this.timeoutTime = timeoutTime;
    }

    public DashboardConnection(DatagramSocket datagramSocket) {
        this(datagramSocket, System.currentTimeMillis() + 3000);
    }

    public void keepAlive() {
        timeoutTime = System.currentTimeMillis() + 3000;
    }

    @Override
    public void close() {
        datagramSocket.close();
    }
}
