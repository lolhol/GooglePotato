package port.pwrup.google.potato;

import java.util.List;

public class GooglePotato {
    static {
        System.loadLibrary("google_potato");
    }

    private long native_ptr;

    public GooglePotato(String dir, String file, List<ImuSensor> imusSensors,
            List<Odom> odoms,
            List<LidarSensor> lidarSensors) {
        this.init(dir, file, imusSensors, odoms, lidarSensors);
    }

    public GooglePotato() {
    }

    /**
     * @param dir          the directory path with config files
     * @param file         the main file of the config
     * @param imusSensors  the imu sensors
     * @param odoms        the odoms
     * @param lidarSensors the lidar sensors that you will use
     * @note this is done by default when you create the class
     */
    public native void init(String dir, String file, List<ImuSensor> imusSensors,
            List<Odom> odoms,
            List<LidarSensor> lidarSensors);

    /**
     * @param timestamp   the current time in ms
     * @param name        the name of the sensor
     * @param pointsX     x positions of the points IN ORDER
     * @param pointsY     y positions of the points IN ORDER
     * @param pointsZ     z positions of the points IN ORDER. If 2D, set this to 0.
     * @param intencities the intensities of the points. MUST BE A LIST OF THE
     *                    LENGTH OF X, Y, Z POINTS.
     */
    public native void addLidarData(long timestamp, String name, float[] pointsX, float[] pointsY, float[] pointsZ,
            float[] intencities);

    /**
     * @param timestamp          the current time. In ms.
     * @param name               the name of the sensor
     * @param linearAcceleration the acceleration captured. [x, y, z]
     * @param angularVelocity    the angualar velocity captured [x, y, z]
     */
    public native void addIMUData(long timestamp, String name, float[] linearAcceleration, float[] angularVelocity);

    /**
     * @param timestamp  the current time. In ms.
     * @param name       the name of sensor
     * @param position   the position in the format of [x, y, z]
     * @param quaternion idk what that is but its in the format of [1, 2, 3, 4]
     */
    public native void addOdomData(long timestamp, String name, float[] position, float[] quaternion);

    /**
     * @return the latest position: [x, y, z, yaw, pitch, roll]
     */
    public native float[] getPosition();

    /**
     * @usage stops cartographer COMPLETELY. No more data will be registered after
     *        this if you do send it.
     */
    public native void stopAndOptimize();

    /**
     * @param getType the map type thgat you want to get
     * @return the map points data in the form of [x, y, z, i, x1, y1, z1, i1, ...]
     * @note this is a wrapper for "getMapPoints" and "getMap"
     */
    public float[] getMapPoints3D(GetType getType) {
        return getMapPoints3D(getType.type);
    }

    /**
     * @param getType the type of map that you want to get.
     * @return the points in format of [x, y, z, i, x1, y1, z1, i1, ...]
     * @warning This is not the function that you would want to call by default use
     *          "getMap" function instead!
     */
    public native float[] getMapPoints3D(int getType);

    public Map2D getMap2D(float minAlpha, float maxIntensity) {
        return (Map2D) get2DMap(minAlpha, maxIntensity);
    }

    /**
     * @param minAlpha     Alpha values denote the
     *                     certainty of that information by indicating whether the
     *                     cell was observed.
     * @param maxIntensity Intensity values often represent the occupancy
     *                     probability (how
     *                     likely a cell is to be an obstacle).
     * @return [x, y, z, i, ...] obstructed points in the map.
     */
    private native Object get2DMap(float minAlpha, float maxIntensity);

    /**
     * @note This is used to switch on / off the logging. By default it will be set
     *       to off.
     */
    private native void switchLogging();
}