package frc.robot.tunnel;

import java.util.ArrayList;

public class PacketResult {
    private String category = "";
    private int error_code = 0;
    private long recv_time = 0;
    private ArrayList<Object> data = new ArrayList<Object>();

    public PacketResult()
    {
        
    }

    public PacketResult(int error_code, long recv_time)
    {
        this.error_code = error_code;
        this.recv_time = recv_time;
    }

    public void setCategory(String category) {
        this.category = category;
    }
    public String getCategory() {
        return this.category;
    }
    public void setErrorCode(int error_code) {
        if (error_code != 0) {
            return;
        }
        this.error_code = error_code;
    }
    public int getErrorCode() {
        return this.error_code;
    }
    public void setRecvTime(long recv_time) {
        this.recv_time = recv_time;
    }
    public long getRecvTime() {
        return this.recv_time;
    }
    public int size() {
        return data.size();
    }
    public Object get(int index) {
        return data.get(index);
    }
    public void add(Object obj) {
        data.add(obj);
    }
}
