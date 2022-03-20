package frc.utility.net;

import java.io.IOException;
import java.net.DatagramPacket;


public interface PacketHandler {
    void handlePacket(DatagramPacket packet) throws IOException;
}
