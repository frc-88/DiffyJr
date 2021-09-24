package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team88.swerve.SwerveController;
import frc.team88.swerve.motion.state.VelocityState;
import frc.robot.listeners.PingListener;
import frc.robot.listeners.CommandListener;

public class SwerveNetworkTable extends SubsystemBase {
    /**
     * A class to encapsulate shared data interactions and action notifications with the NVidia Jetson.
     */
    
    private SwerveController m_swerve;

    private NetworkTable table;
    private NetworkTable clientTable;
    private NetworkTable commandsTable;
    private NetworkTableEntry clientTimestamp;
    private NetworkTableEntry hostTimestamp;

    private NetworkTable driverStationTable;

    private final long clientConnectedTimeout = 1_000_000;  // microseconds
    private final String rootTableName = "swerveLibrary";

    private PingListener ntPingListener;
    private CommandListener ntCommandListener;

    public SwerveNetworkTable(SwerveController swerve)
    {
        m_swerve = swerve;

        table = NetworkTableInstance.getDefault().getTable(rootTableName);  // Data and commands from the RoboRIO
        clientTable = table.getSubTable("ROS");  // Data from the Jetson
        commandsTable = table.getSubTable("commands");  // Commands from the Jetson
        clientTimestamp = clientTable.getEntry("timestamp");
        hostTimestamp = table.getEntry("timestamp");

        driverStationTable = table.getSubTable("DriverStation");
        
        hostTimestamp.setNumber(getTime());

        ntPingListener = new PingListener();
        ntPingListener.setTable(table);
        clientTable.addEntryListener("ping", ntPingListener, EntryListenerFlags.kUpdate);

        ntCommandListener = new CommandListener();
        ntCommandListener.setTable(commandsTable);
        commandsTable.addEntryListener("timestamp", ntCommandListener, EntryListenerFlags.kUpdate);
    }

    private long getTime()
    {
        return RobotController.getFPGATime();
    }

    public void setCommand()
    {
        VelocityState state = ntCommandListener.getCommand();
        m_swerve.setVelocity(state.getTranslationDirection(), state.getTranslationSpeed(), state.getRotationVelocity(), state.isFieldCentric());
    }

    public boolean isConnected()
    {
        return clientTimestamp.exists() && hostTimestamp.exists() && (getTime() - clientTimestamp.getLastChange() < clientConnectedTimeout);
    }

    public boolean isCommandActive()
    {
        return isConnected() && ntCommandListener.isActive();
    }

    @Override
    public void periodic()
    {
        hostTimestamp.setNumber(getTime());
        if (!isConnected()) {
            return;
        }
        // Driver Station
        driverStationTable.getEntry("isFMSAttached").setBoolean(DriverStation.getInstance().isFMSAttached());
        driverStationTable.getEntry("getMatchTime").setDouble(DriverStation.getInstance().getMatchTime());
    }
}
