package frc.robot.tunnel;

import java.io.PrintWriter;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

public class TunnelServer extends Thread {
    private int tunnelPort;
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private PrintWriter out;
    private BufferedReader in;
    private boolean isSocketInitialized = false;
    
    public TunnelServer(int port) {
        tunnelPort = port;
    }

    public void run()
    {
        while (true)
        {
            if (checkSocket())
            {
                String message;
                try {
                    message = in.readLine();
                }
                catch (IOException e) {
                    e.printStackTrace();
                    flagSocketClosed();
                    continue;
                }
                if (message == null) {
                    flagSocketClosed();
                    continue;
                }
            }
        }
    }

    private void write(String message)
    {
        if (!isSocketInitialized) {
            return;
        }
        out.println(message);

    }
    private void flagSocketClosed()
    {
        isSocketInitialized = false;
    }

    private boolean checkSocket()
    {
        if (isSocketInitialized) {
            return true;
        }
        try
        {
            serverSocket = new ServerSocket(tunnelPort);
            clientSocket = serverSocket.accept();
            out = new PrintWriter(clientSocket.getOutputStream(), true);
            in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
            isSocketInitialized = true;
            return true;
        }
        catch (IOException e) {
            e.printStackTrace();
            isSocketInitialized = false;
            return false;
        }
    }
}
