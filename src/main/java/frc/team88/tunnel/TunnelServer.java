package frc.team88.tunnel;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Objects;

public class TunnelServer extends Thread {

    private int port = 0;
    private TunnelInterface tunnel_interface;
    private ArrayList<TunnelClient> clients;
    private TunnelDataRelayThread data_relay_thread;

    public static TunnelServer instance = null;

    public TunnelServer(TunnelInterface tunnel_interface, int port, int data_relay_delay_ms)
    {
        if (!Objects.isNull(instance)) {
            throw new RuntimeException("Only once instance of TunnelServer allowed");
        }
        instance = this;

        clients = new ArrayList<TunnelClient>();
        this.port = port;
        this.tunnel_interface = tunnel_interface;
        data_relay_thread = new TunnelDataRelayThread(tunnel_interface, data_relay_delay_ms);
    }

    public boolean anyClientsAlive()
    {
        for (int index = 0; index < clients.size(); index++)
        {
            if (clients.get(index).isAlive() && clients.get(index).isOpen()) {
                return true;
            }
        }
        return false;
    }

    private void cleanUpThreads()
    {
        for (int index = 0; index < clients.size(); index++)
        {
            if (!clients.get(index).isAlive() || !clients.get(index).isOpen()) {
                clients.remove(index);
                index--;
            }
        }
    }

    // Write a packet to all connected clients
    public void writePacket(String category, Object... objects)
    {
        for (int index = 0; index < clients.size(); index++)
        {
            if (clients.get(index).isAlive() && clients.get(index).isOpen()) {
                clients.get(index).writePacket(category, objects);
            }
        }
    }

    // Send message to all clients
    public void println(String message) {
        writePacket("__msg__", message);
    }

    @Override
    public void run()
    {
        data_relay_thread.start();
        ServerSocket serverSocket = null;
        Socket socket = null;

        try {
            serverSocket = new ServerSocket(port);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        System.out.println("Socket is open");
        try
        {
            while (true) {
                try {
                    socket = serverSocket.accept();
                } catch (IOException e) {
                    System.out.println("I/O error: " + e);
                }
                cleanUpThreads();

                // new threads for a client
                TunnelClient client = new TunnelClient(tunnel_interface, socket);
                clients.add(client);
                client.start();
            }
        }
        finally {
            try {
                System.out.println("Closing socket");
                serverSocket.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
