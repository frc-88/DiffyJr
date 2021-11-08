package frc.team88.tunnel;

import java.util.HashMap;

public interface TunnelInterface {
    public HashMap<String, String> getCategories();
    public void packetCallback(TunnelClient tunnel, PacketResult result);
    public void update();
    public void setTunnelServer(TunnelServer server);
}
