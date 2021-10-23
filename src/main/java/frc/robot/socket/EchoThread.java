package frc.robot.socket;

import java.io.BufferedReader;
import java.io.CharArrayWriter;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.Socket;
import java.nio.CharBuffer;
import java.nio.charset.Charset;
import java.util.Formatter;

public class EchoThread extends Thread {
    protected Socket socket;

    public EchoThread(Socket clientSocket) {
        this.socket = clientSocket;
    }

    public String formatChar(char c)
    {
        if (c == 92) return "\\\\";
        else if (c == 9) return "\\t";
        else if (c == 10) return "\\n";
        else if (c == 13) return "\\r";
        else if (c == 11 || c == 12 || c <= 9 || (14 <= c && c <= 31) || 127 <= c)
        {
            return String.format("\\x%02x", c).toString();
        }
        else {
            return String.valueOf(c);
        }
    }
    public String packetToString(char[] buffer, int length)
    {
        for (int index = 0; index < length; index++) {
            buffer[index]
        }
    }

    public void run() {
        InputStream inp = null;
        BufferedReader brinp = null;
        DataOutputStream out = null;
        System.out.println("Opening client");
        try {
            inp = socket.getInputStream();
            brinp = new BufferedReader(new InputStreamReader(inp));
            out = new DataOutputStream(socket.getOutputStream());
        } catch (IOException e) {
            return;
        }
        String line;
        int buffer_size = 1024;
        char[] buffer = new char[buffer_size];
        while (true) {
            try {
                int charsIn = brinp.read(buffer, 0, buffer_size);
                System.out.println("Read " + charsIn + " characters");
                line = new String(buffer);
                System.out.println("Received: " + line);
                if (charsIn == -1) {
                    System.out.println("Closing client");
                    socket.close();
                    return;
                } else {
                    byte[] bytes = Charset.forName("UTF-8").encode(CharBuffer.wrap(buffer)).array();
                    out.write(bytes, 0, charsIn);
                    out.flush();
                }
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
        }
    }
}
