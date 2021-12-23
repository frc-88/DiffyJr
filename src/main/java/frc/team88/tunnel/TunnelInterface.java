package frc.team88.tunnel;

public interface TunnelInterface {
    public void packetCallback(TunnelClient tunnel, PacketResult result);
    public void update();
}
