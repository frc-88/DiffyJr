package frc.team88.tunnel;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Objects;
import java.util.concurrent.locks.ReentrantLock;


public class TunnelClient extends Thread {
    protected Socket socket;
    private TunnelInterface tunnel_interface;
    private TunnelProtocol protocol;

    private InputStream input = null;
    private OutputStream output = null;
    private boolean isOpen = false;

    private int buffer_size = 1024;
    private byte[] buffer = new byte[buffer_size];
    private int unparsed_index = 0;
    
    private static ReentrantLock write_lock = new ReentrantLock();

    public TunnelClient(TunnelInterface tunnel_interface, Socket clientSocket) {
        this.socket = clientSocket;
        this.tunnel_interface = tunnel_interface;

        protocol = new TunnelProtocol(tunnel_interface.getCategories());

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

    public void writePacket(String category, Object... objects) {
        byte[] packet = protocol.makePacket(category, objects);
        writeBytes(packet);
    }

    private void writeBytes(byte[] data) {
        try {
            TunnelClient.write_lock.lock();
            writeBuffer(data);
        }
        catch (IOException e) {
            e.printStackTrace();
            System.out.println("Failed while writing data: " + TunnelUtil.packetToString(data));
            isOpen = false;
        }
        finally {
            TunnelClient.write_lock.unlock();
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
                if (num_chars_read == -1) {
                    System.out.println("Closing client");
                    socket.close();
                    return;
                }
                
                int last_parsed_index = protocol.parseBuffer(Arrays.copyOfRange(buffer, 0, unparsed_index + num_chars_read));
                if (last_parsed_index > 0)
                {
                    for (int index = last_parsed_index, shifted_index = 0; index < buffer.length; index++, shifted_index++) {
                        buffer[shifted_index] = buffer[index];
                    }
                }
                unparsed_index = unparsed_index + num_chars_read - last_parsed_index;
                if (unparsed_index >= buffer_size) {
                    unparsed_index = 0;
                }
                
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
                    tunnel_interface.packetCallback(this, result);
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
