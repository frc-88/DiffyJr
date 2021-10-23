package frc.robot.tunnel;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

import frc.team88.swerve.SwerveController;

public class TunnelServer extends Thread {

    private int port = 0;
    private SwerveController m_swerve;
    private ArrayList<TunnelThread> tunnels;

    public TunnelServer(SwerveController swerve, int port)
    {
        tunnels = new ArrayList<TunnelThread>();
        this.port = port;
        this.m_swerve = swerve;
    }

    public void update()
    {
        setCommandIfActive();
        sendOdometry();
    }

    private void sendOdometry()
    {
        for (int index = tunnels.size() - 1; index >= 0; index--)
        {
            TunnelThread tunnel = tunnels.get(index);
            if (tunnel.isAlive()) {
                tunnel.sendOdometry();
            }
        }
    }

    private void setCommandIfActive()
    {
        for (int index = tunnels.size() - 1; index >= 0; index--)
        {
            TunnelThread tunnel = tunnels.get(index);
            if (tunnel.isCommandActive()) {
                m_swerve.setVelocity(tunnel.getCommand());
                break;
            }
        }
    }

    private void cleanUpThreads()
    {
        for (int index = 0; index < tunnels.size(); index++)
        {
            if (!tunnels.get(index).isAlive()) {
                tunnels.remove(index);
                index--;
            }
        }
    }

    @Override
    public void run()
    {
        ServerSocket serverSocket = null;
        Socket socket = null;

        try {
            serverSocket = new ServerSocket(port);
        } catch (IOException e) {
            e.printStackTrace();
        }
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
                TunnelThread tunnel = new TunnelThread(m_swerve, socket);
                tunnels.add(tunnel);
                tunnel.start();
            }
        }
        finally {
            try {
                serverSocket.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
