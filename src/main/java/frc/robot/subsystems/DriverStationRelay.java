package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team88.swerve.SwerveController;
import frc.robot.listeners.PingListener;

public class DriverStationRelay extends SubsystemBase {
    /**
     * A class to encapsulate shared data interactions and action notifications with the NVidia Jetson.
     */
    
    private SwerveController m_swerve;

    private NetworkTable table;
    private NetworkTable clientTable;
    private NetworkTableEntry clientTimestamp;
    private NetworkTableEntry hostTimestamp;

    private NetworkTable driverStationTable;

    private final double clientConnectedTimeout = 1.0;  // seconds
    private final String rootTableName = "swerveLibrary";

    private PingListener ntPingListener;

    public DriverStationRelay(SwerveController swerve)
    {
        m_swerve = swerve;

        table = NetworkTableInstance.getDefault().getTable(rootTableName);  // Data and commands from the RoboRIO
        clientTable = table.getSubTable("ROS");  // Data and commands from the Jetson
        clientTimestamp = clientTable.getEntry("timestamp");
        hostTimestamp = table.getEntry("timestamp");

        driverStationTable = table.getSubTable("DriverStation");
        
        hostTimestamp.setNumber(getTime());

        ntPingListener = new PingListener();
        ntPingListener.setTable(clientTable);
        clientTable.addEntryListener("ping", ntPingListener, EntryListenerFlags.kUpdate);
    }

    private long getTime()
    {
        return RobotController.getFPGATime();
    }

    public boolean isConnected()
    {
        return clientTimestamp.exists() && hostTimestamp.exists() && (getTime() - clientTimestamp.getLastChange() < clientConnectedTimeout);
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
