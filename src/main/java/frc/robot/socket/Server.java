package frc.robot.socket;

import java.net.*;
import java.io.*;

public class Server extends Thread {
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private PrintWriter out;
    private BufferedReader in;

    public Server(int port) throws IOException {
        System.out.println("Starting server");
        serverSocket = new ServerSocket(port);
        clientSocket = serverSocket.accept();
        out = new PrintWriter(clientSocket.getOutputStream(), true);
        in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
    }

    public void run()
    {
        while (true) {
            if (!update()) {
                break;
            }
        }
    }

    public boolean update()
    {
        String message;
        try {
            message = in.readLine();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return false;
        }
        if (message == null) {
            return false;
        }
        System.out.println("Message received: " + message);
        out.println(message);

        return true;
    }

    public void close() throws IOException {
        in.close();
        out.close();
        clientSocket.close();
        serverSocket.close();
    }

    public static void main(String[] args) {
        try {
            Server server = new Server(8080);
            server.start();
            while (true)  { Thread.sleep(1000); }
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}