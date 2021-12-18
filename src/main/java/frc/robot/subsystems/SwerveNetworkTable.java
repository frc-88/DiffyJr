package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.listeners.PingListener;
import frc.robot.listeners.SetOdomListener;
import frc.team88.diffswerve.DiffSwerveChassis;


public class SwerveNetworkTable extends SubsystemBase {
    /**
     * A class to encapsulate shared data interactions and action notifications with the NVidia Jetson.
     */
    
    private DiffSwerveChassis m_swerve;

    private NetworkTable table;
    private NetworkTable clientTable;
    private NetworkTable setOdomTable;
    private NetworkTableEntry clientTimestamp;
    private NetworkTableEntry hostTimestamp;

    private NetworkTable driverStationTable;
    private NetworkTableEntry isFMSAttachedEntry;
    private NetworkTableEntry getMatchTimeEntry;

    private final long clientConnectedTimeout = 1_000_000;  // microseconds
    private final String rootTableName = "swerveLibrary";

    private PingListener ntPingListener;
    private SetOdomListener setOdomListener;

    private DriverStation m_driverStationInstance;

    public SwerveNetworkTable(DiffSwerveChassis swerve)
    {
        super();

        m_swerve = swerve;

        table = NetworkTableInstance.getDefault().getTable(rootTableName);  // Data and commands from the RoboRIO
        clientTable = table.getSubTable("ROS");  // Data from the Jetson
        setOdomTable = clientTable.getSubTable("setOdom");
        clientTimestamp = clientTable.getEntry("timestamp");
        hostTimestamp = table.getEntry("timestamp");

        driverStationTable = table.getSubTable("DriverStation");
        
        hostTimestamp.setNumber(getTime());

        ntPingListener = new PingListener();
        ntPingListener.setTable(clientTable);

        setOdomListener = new SetOdomListener(m_swerve);
        setOdomListener.setTable(setOdomTable);
        setOdomTable.addEntryListener("timestamp", setOdomListener, EntryListenerFlags.kUpdate);

        isFMSAttachedEntry = driverStationTable.getEntry("isFMSAttached");
        getMatchTimeEntry = driverStationTable.getEntry("getMatchTime");

        m_driverStationInstance = DriverStation.getInstance();
    }

    private long getTime()
    {
        return RobotController.getFPGATime();
    }

    public boolean isConnected()
    {
        return clientTimestamp.exists() && hostTimestamp.exists() && (getTime() - clientTimestamp.getLastChange() < clientConnectedTimeout);
    }

    public void update()
    {
        if (!isConnected()) {
            return;
        }
        // Driver Station
        isFMSAttachedEntry.setBoolean(m_driverStationInstance.isFMSAttached());
        getMatchTimeEntry.setDouble(m_driverStationInstance.getMatchTime());
    }
}
