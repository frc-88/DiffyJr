package frc.robot.tunnel;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.HashMap;
import java.util.Objects;

import frc.robot.tunnel.TunnelUtil;

import frc.team88.swerve.SwerveController;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.motion.state.OdomState;

public class TunnelThread extends Thread {
    protected Socket socket;
    private SwerveController m_swerve;
    private TunnelProtocol protocol;

    InputStream input = null;
    OutputStream output = null;

    VelocityState swerveCommand = new VelocityState(0.0, 0.0, 0.0, false);;

    public TunnelThread(SwerveController swerve, Socket clientSocket) {
        this.socket = clientSocket;
        this.m_swerve = swerve;

        protocol = new TunnelProtocol(new HashMap<String, String>(){
            private static final long serialVersionUID = 1L; {
            put("ping", "f");
            put("cmd", "fff");
        }});

        System.out.println("Opening client");
        try {
            input = socket.getInputStream();
            output = socket.getOutputStream();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void sendOdometry()
    {
        OdomState odom = m_swerve.getOdometry();
        writePacket("odom",
            odom.getXPosition(), odom.getYPosition(), odom.getTheta(),
            odom.getXVelocity(), odom.getYVelocity(), odom.getThetaVelocity()
        );
    }

    public boolean isCommandActive()
    {
        return false;
    }

    public VelocityState getCommand() {
        return swerveCommand;
    }

    private void writePacket(String category, Object... objects) {
        byte[] packet = protocol.makePacket(category, objects);
        
        try {
            writeBuffer(packet);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Failed while writing packet: " + TunnelUtil.packetToString(packet));
        }
    }

    private void writeBuffer(byte[] buffer) throws IOException
    {
        if (!Objects.nonNull(output)) {
            System.out.println("Output stream is null! Skipping write.");
            return;
        }
        output.write(buffer, 0, buffer.length);
        output.flush();
    }

    private void packetCallback(PacketResult result) {
        String category = result.getCategory();
        if (category.equals("cmd")) {
            swerveCommand = new VelocityState(
                (double)result.get(0),
                (double)result.get(1),
                (double)result.get(2),
                false
            );
        }
        else if (category.equals("ping")) {
            writePacket("ping", (double)result.get(0));
        }
    }

    public void run()
    {
        if (!Objects.nonNull(input)) {
            return;
        }
        
        int buffer_size = 1024;
        byte[] buffer = new byte[buffer_size];
        int buffer_index = 0;
        
        while (true) {
            try {
                int charsIn = input.read(buffer, buffer_index, buffer_size - buffer_index);
                System.out.println("Read " + charsIn + " characters");
                System.out.println("Received: " + TunnelUtil.packetToString(buffer, charsIn));
                if (charsIn == -1) {
                    System.out.println("Closing client");
                    socket.close();
                    return;
                }
                buffer_index = protocol.parseBuffer(buffer);
                PacketResult result;
                do {
                    result = protocol.popResult();
                    if (protocol.isCodeError(result.getErrorCode())) {
                        System.out.println(String.format("Encountered error code %d. Buffer: %s",
                            result.getErrorCode(),
                            TunnelUtil.packetToString(buffer))
                        );
                        continue;
                    }
                    packetCallback(result);
                }
                while (result.getErrorCode() != -1);

            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
        }
    }
}
