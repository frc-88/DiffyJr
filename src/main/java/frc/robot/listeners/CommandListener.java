package frc.robot.listeners;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.RobotController;
import frc.team88.swerve.motion.state.VelocityState;

/** Handles callbacks for when the command subtable is updated */
public class CommandListener implements TableEntryListener {
  private NetworkTable m_table;
  private long lastActiveTime = 0;
  private long activeTimeThreshold = 1_000_000;  // microseconds
  private VelocityState command;

  public CommandListener() {
    command = new VelocityState(0.0, 0.0, 0.0, false);
  }

  public boolean isActive()
  {
    return (getTime() - lastActiveTime) < activeTimeThreshold;
  }
  
  private long getTime()
  {
    return RobotController.getFPGATime();
  }
  /**
   * Gets the last command sent via network tables
   *
   * @return VelocityState
   */
  public VelocityState getCommand() {
    return command;
  }

  /**
   * Take a NetworkTable instance from parent and setup keys to listen to
   *
   * @param table A table from DataManager
   */
  public void setTable(NetworkTable table) {
    m_table = table;
    m_table.getEntry("timestamp").setDouble(0.0);
    m_table.getEntry("translationDirection").setDouble(0.0);
    m_table.getEntry("translationSpeed").setDouble(0.0);
    m_table.getEntry("rotationVelocity").setDouble(0.0);
    m_table.getEntry("isFieldCentric").setBoolean(false);
  }

  /** Callback for when data is pushed to the designated subtable */
  @Override
  public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
    lastActiveTime = getTime();
    double translationDirection = m_table.getEntry("translationDirection").getDouble(0.0);
    double translationSpeed = m_table.getEntry("translationSpeed").getDouble(0.0);
    double rotationVelocity = m_table.getEntry("rotationVelocity").getDouble(0.0);
    boolean isFieldCentric = m_table.getEntry("isFieldCentric").getBoolean(false);

    command =
        command
            .changeTranslationDirection(translationDirection)
            .changeTranslationSpeed(translationSpeed)
            .changeRotationVelocity(rotationVelocity)
            .changeIsFieldCentric(isFieldCentric);

  }
}
