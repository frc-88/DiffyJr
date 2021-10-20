package frc.robot.listeners;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;

/** Handles callbacks for when the command subtable is updated */
public class PingListener implements TableEntryListener {
  private NetworkTable m_table;
  private double ping_time = 0.0;
  private NetworkTableEntry pingResponseEntry;

  public PingListener() {

  }

  public double getPingTime()
  {
    return ping_time;
  }

  /**
   * Take a NetworkTable instance from parent and setup keys to listen to
   *
   * @param table A table from DataManager
   */
  public void setTable(NetworkTable table) {
    m_table = table;
    pingResponseEntry = m_table.getEntry("pingResponse");
    pingResponseEntry.setDouble(0.0);
  }

  public void setPingResponse(double ping_response) {
    pingResponseEntry.setDouble(ping_response);
  }

  /** Callback for when data is pushed to the designated subtable */
  @Override
  public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
    ping_time = value.getDouble();
    setPingResponse(ping_time);
  }
}
