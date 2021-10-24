package frc.robot.tunnel;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.net.SocketException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.tunnel.TunnelUtil;

import frc.team88.swerve.SwerveController;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.motion.state.OdomState;

public class TunnelThread extends Thread {
    protected Socket socket;
    private SwerveController m_swerve;
    private TunnelProtocol protocol;

    private InputStream input = null;
    private OutputStream output = null;
    private boolean isOpen = false;

    private int buffer_size = 1024;
    private byte[] buffer = new byte[buffer_size];
    private int unparsed_index = 0;
    
    private long last_command_time = 0;
    private final long ACTIVE_TIME_THRESHOLD = 1_000_000;  // microseconds
    private VelocityState swerveCommand = new VelocityState(0.0, 0.0, 0.0, false);

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
            isOpen = true;

        } catch (IOException e) {
            e.printStackTrace();
            isOpen = false;
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

    public boolean isCommandActive() {
        return isOpen() && getTime() - last_command_time < ACTIVE_TIME_THRESHOLD;
    }

    public VelocityState getCommand() {
        return swerveCommand;
    }

    private void writePacket(String category, Object... objects) {
        byte[] packet = protocol.makePacket(category, objects);
        // System.out.println("Writing: " + TunnelUtil.packetToString(packet));
        
        try {
            writeBuffer(packet);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Failed while writing packet: " + TunnelUtil.packetToString(packet));
            isOpen = false;
        }
    }

    public boolean isOpen() {
        return isOpen;
    }

    private void writeBuffer(byte[] buffer) throws IOException
    {
        if (!Objects.nonNull(output) || !isOpen) {
            System.out.println("Socket is closed! Skipping write.");
            return;
        }
        try {
            output.write(buffer, 0, buffer.length);
            output.flush();
        }
        catch (SocketException e) {
            e.printStackTrace();
            isOpen = false;
        }
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
            resetCommandTimer();
        }
        else if (category.equals("ping")) {
            writePacket("ping", (double)result.get(0));
        }
    }

    private long getTime() {
        return RobotController.getFPGATime();
    }
    
    private void resetCommandTimer() {
        last_command_time = getTime();
    }

    public void run()
    {
        if (!Objects.nonNull(input)) {
            return;
        }
        
        while (true) {
            try {
                int num_chars_read = input.read(buffer, unparsed_index, buffer_size - unparsed_index);
                if (num_chars_read == 0) {
                    continue;
                }
                // System.out.println("Read " + charsIn + " characters");
                if (num_chars_read == -1) {
                    System.out.println("Closing client");
                    socket.close();
                    return;
                }
                // System.out.println("Received: " + TunnelUtil.packetToString(buffer, charsIn));
                // System.out.println("Buffer: " + TunnelUtil.packetToString(buffer));
                int last_parsed_index = protocol.parseBuffer(Arrays.copyOfRange(buffer, 0, unparsed_index + num_chars_read));
                if (last_parsed_index > 0)
                {
                    for (int index = last_parsed_index, shifted_index = 0; index < buffer.length; index++, shifted_index++) {
                        buffer[shifted_index] = buffer[index];
                    }
                }
                unparsed_index = unparsed_index + num_chars_read - last_parsed_index;
                
                PacketResult result;
                do {
                    result = protocol.popResult();
                    if (result.getErrorCode() == TunnelProtocol.NULL_ERROR) {
                        continue;
                    }
                    if (protocol.isCodeError(result.getErrorCode())) {
                        System.out.println(String.format("Encountered error code %d.",
                            result.getErrorCode()
                        ));
                        continue;
                    }
                    packetCallback(result);
                }
                while (result.getErrorCode() != -1);

            } catch (IOException e) {
                e.printStackTrace();
                isOpen = false;
                return;
            }
        }
    }
}
