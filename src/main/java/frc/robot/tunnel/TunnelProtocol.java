package frc.robot.tunnel;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotController;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class TunnelProtocol {
    private final char PACKET_START_0 = 0x12;
    private final char PACKET_START_1 = 0x13;
    private final char PACKET_STOP = '\n';
    private final char PACKET_SEP = '\t';
    private final int MAX_PACKET_LEN = 128;
    private final int MIN_PACKET_LEN = 13;
    private final int MAX_SEGMENT_LEN = 64;
    private final int CHECKSUM_START_INDEX = 5;
    private final int LENGTH_START_INDEX = 2;
    private final int LENGTH_BYTE_LENGTH = 2;

    private final int NO_ERROR = 0;
    private final int PACKET_0_ERROR = 1;
    private final int PACKET_1_ERROR = 2;
    private final int PACKET_TOO_SHORT_ERROR = 3;
    private final int CHECKSUMS_DONT_MATCH_ERROR = 4;
    private final int PACKET_COUNT_NOT_FOUND_ERROR = 5;
    private final int PACKET_COUNT_NOT_SYNCED_ERROR = 6;
    private final int PACKET_CATEGORY_ERROR = 7;
    private final int INVALID_FORMAT_ERROR = 8;
    private final int PACKET_STOP_ERROR = 9;
    private final int SEGMENT_TOO_LONG_ERROR = 10;
    private final int PACKET_TIMEOUT_ERROR = 11;
    private final int FORMAT_TYPE_ERROR = 12;

    private int read_packet_num = -1;
    private int recv_packet_num = 0;
    private int write_packet_num = 0;
    
    private byte[] write_buffer = new byte[MAX_PACKET_LEN];
    private int write_buffer_index = 0;

    private byte[] current_segment;
    private int parse_error_code = -1;
    
    private Map<String, String> categories;
    private ArrayList<PacketResult> resultQueue = new ArrayList<PacketResult>();

    public TunnelProtocol(HashMap<String, String> categories)
    {
        this.categories = categories;
    }

    public byte[] makePacket(String category, Object... args)
    {
        applyPacketHeader(category);
        for (Object object : args) {
            if (object instanceof Integer || object instanceof Boolean) {
                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, TunnelUtil.toInt32Bytes((int)object));
            }
            if (object instanceof Double) {
                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, TunnelUtil.toFloatBytes((double)object));
            }
            else if (object instanceof String) {
                short length = (short)((String)object).length();

                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, TunnelUtil.toUInt16Bytes(length));
                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, ((String)object).getBytes());
            }
            else {
                System.out.println("Encountered invalid type while making packet for category " + category);
            }
        }
        applyPacketFooter();

        return Arrays.copyOfRange(write_buffer, 0, write_buffer_index);
    }

    private void applyPacketHeader(String category)
    {
        write_buffer_index = 0;
        write_buffer[write_buffer_index++] = PACKET_START_0;
        write_buffer[write_buffer_index++] = PACKET_START_1;
        write_buffer_index += LENGTH_BYTE_LENGTH;  // two bytes for packet length
        byte[] packet_num_bytes = TunnelUtil.toInt32Bytes(write_packet_num);
        write_packet_num++;
        write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, packet_num_bytes);
        byte[] category_bytes = category.getBytes();
        write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, category_bytes);
        write_buffer[write_buffer_index++] = PACKET_SEP;
    }

    private void applyPacketFooter()
    {
        byte calc_checksum = calculateChecksum(Arrays.copyOfRange(write_buffer, CHECKSUM_START_INDEX, write_buffer_index));
        
        write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index,
            String.format("%02x", calc_checksum).getBytes()
        );
        TunnelUtil.copyArray(write_buffer, LENGTH_START_INDEX, 
            TunnelUtil.toUInt16Bytes(
                (short)(write_buffer_index - (LENGTH_START_INDEX + LENGTH_BYTE_LENGTH))
            )
        );
        write_buffer[write_buffer_index++] = PACKET_STOP;
    }

    public byte calculateChecksum(byte[] packet) {
        byte calc_checksum = 0;
        for (byte val : packet) {
            calc_checksum += val;
        }
        return calc_checksum;
    }

    public int parseBuffer(byte[] buffer)
    {
        // TODO enforce max packet length
        int packet_start = 0;
        int index;
        for (index = 0; index < buffer.length; index++) {
            packet_start = index;
            if (buffer[index] != PACKET_START_0) {
                continue;
            }
            if (buffer[index] != PACKET_START_1) {
                continue;
            }
            int length_start = index;
            index += 2;
            if (index >= buffer.length) {
                index = buffer.length;
                continue;
            }

            ByteBuffer wrapped = ByteBuffer.wrap(Arrays.copyOfRange(buffer, length_start, index));
            int length = (int)wrapped.getShort();
            index += length + 1;
            if (index >= buffer.length) {
                index = buffer.length;
                continue;
            }
            if (buffer[index] != PACKET_STOP) {
                continue;
            }
            PacketResult result = parsePacket(Arrays.copyOfRange(buffer, packet_start, index));
            resultQueue.add(result);
        }

        return index;
    }

    private long getTime()
    {
        return RobotController.getFPGATime();
    }
    public PacketResult parsePacket(byte[] buffer)
    {
        long recv_time = getTime();
        if (buffer.length < MIN_PACKET_LEN) {
            System.out.println(String.format("Packet is not the minimum length (%s): %s", MIN_PACKET_LEN, TunnelUtil.packetToString(buffer)));
            return new PacketResult(PACKET_TOO_SHORT_ERROR, recv_time);
        }

        if (buffer[0] != PACKET_START_0) {
            System.out.println(String.format("Packet does not start with PACKET_START_0: %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(PACKET_0_ERROR, recv_time);
        }
        if (buffer[1] != PACKET_START_1) {
            System.out.println(String.format("Packet does not start with PACKET_START_1: %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(PACKET_1_ERROR, recv_time);
        }
        if (buffer[buffer.length - 1] != PACKET_STOP) {
            System.out.println(String.format("Packet does not start with PACKET_STOP: %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(PACKET_STOP_ERROR, recv_time);
        }
        byte calc_checksum = calculateChecksum(Arrays.copyOfRange(buffer, LENGTH_START_INDEX + LENGTH_BYTE_LENGTH, buffer.length - 1));
        byte recv_checksum = TunnelUtil.hexToByte(new String(Arrays.copyOfRange(buffer, buffer.length - 2, buffer.length)));
        if (calc_checksum != recv_checksum) {
            System.out.println(String.format("Checksum failed! recv %02x != calc %02x. %s", recv_checksum, calc_checksum, TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(CHECKSUMS_DONT_MATCH_ERROR, recv_time);
        }
    }

    public static void main(String[] args) {
        TunnelProtocol protocol = new TunnelProtocol(new HashMap<String, String>(){
            private static final long serialVersionUID = 1L; {
            put("ping", "f");
            put("cmd", "fff");
        }});

        
        String result = TunnelUtil.packetToString(protocol.makePacket("ping", 4.0));
        System.out.println(result);

        result = TunnelUtil.packetToString(protocol.makePacket("something", 50.0, 10, "else"));
        System.out.println(result);
    }
}
