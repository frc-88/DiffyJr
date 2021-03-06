package frc.team88.tunnel;

public class TunnelDataRelayThread extends Thread {
    private TunnelInterface tunnel_interface;
    private int delay_ms;

    public TunnelDataRelayThread(TunnelInterface tunnel_interface, int delay_ms) {
        this.tunnel_interface = tunnel_interface;
        this.delay_ms = delay_ms;
    }

    @Override
    public void run() {
        while (true) {
            tunnel_interface.update();
            try {
                Thread.sleep(delay_ms);
            } catch (InterruptedException e) {
                e.printStackTrace();
                return;
            }
        }
    }
}
