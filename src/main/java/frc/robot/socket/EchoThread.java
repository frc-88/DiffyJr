package frc.robot.socket;

import java.io.BufferedReader;
import java.io.CharArrayReader;
import java.io.CharArrayWriter;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.charset.Charset;
import java.util.Formatter;

public class EchoThread extends Thread {
    protected Socket socket;

    public EchoThread(Socket clientSocket) {
        this.socket = clientSocket;
    }

    public String formatChar(byte c)
    {
        // c &= 0xff;
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
    public String packetToString(byte[] buffer, int length)
    {
        String s = "";
        for (int index = 0; index < length; index++) {
            s += formatChar(buffer[index]);
        }
        return s;
    }

    public void run() {
        InputStream inp = null;
        // BufferedReader brinp = null;
        // DataOutputStream out = null;
        // OutputStreamWriter out = null;
        OutputStream out = null;
        System.out.println("Opening client");
        try {
            inp = socket.getInputStream();
            // brinp = new BufferedReader(new InputStreamReader(inp, "UTF-8"));
            // out = new DataOutputStream(socket.getOutputStream());
            // out = new OutputStreamWriter(socket.getOutputStream(), "UTF-8");
            out = socket.getOutputStream();

        } catch (IOException e) {
            return;
        }
        String line;
        int buffer_size = 1024;
        byte[] buffer = new byte[buffer_size];
        // char[] buffer = new char[buffer_size];
        // CharBuffer buffer = CharBuffer.allocate(buffer_size);
        while (true) {
            try {
                int charsIn = inp.read(buffer);
                // int charsIn = brinp.read(buffer);
                System.out.println("Read " + charsIn + " characters");
                // System.out.println("Read " + bytes.length + " characters");
                // line = new String(buffer);
                System.out.println("Received: " + packetToString(buffer, charsIn));
                if (charsIn == -1) {
                    System.out.println("Closing client");
                    socket.close();
                    return;
                } else {
                    // byte[] bytes = Charset.forName("UTF-8").encode(CharBuffer.wrap(buffer)).array();
                    // String str = new String(buffer, 0, charsIn);
                    // byte[] bytes_buffer = str.getBytes();
                    // System.out.println(bytes_buffer.length);
                    // out.write(bytes_buffer, 0, charsIn);
                    // out.writeChars(str);
                    // out.write(buffer.array());
                    // buffer.clear();
                    out.write(buffer, 0, charsIn);
                    out.flush();
                }
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
        }
    }
}
