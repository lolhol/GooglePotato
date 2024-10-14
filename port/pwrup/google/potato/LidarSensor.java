package port.pwrup.google.potato;

public class LidarSensor {
    public float x, y, z, lidarScanTimeHz;
    public String name;

    /**
     * @param x               the x position of the lidar relative to the middle
     * @param y               the y position of the lidar relative to the middle
     * @param z               the z position of the lidar relative to the middle
     * @param lidarScanTimeHz the lidar scan time EX: 6Hz
     * @param name            the name of the lidar input sensor
     */
    public LidarSensor(float x, float y, float z, float lidarScanTimeHz, String name) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.lidarScanTimeHz = lidarScanTimeHz;
        this.name = name;
    }
}
