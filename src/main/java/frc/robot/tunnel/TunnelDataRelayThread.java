package frc.robot.tunnel;

public class TunnelDataRelayThread extends Thread {
    TunnelServer tunnel;

    public TunnelDataRelayThread(TunnelServer tunnel) {
        this.tunnel = tunnel;
    }

    @Override
    public void run() {
        while (true) {
            tunnel.update();
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
                return;
            }
        }
    }
}
