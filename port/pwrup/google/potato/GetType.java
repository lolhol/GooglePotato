package port.pwrup.google.potato;

/**
 * HIGH_RESOLUTION_MAP - more points
 * LOW_RESOLUTION_MAP - less points
 * GRAVITY_ALLIGNED_MAP - idk what that is
 */
public enum GetType {
    HIGH_RESOLUTION_MAP(0),
    LOW_RESOLUTION_MAP(1),
    GRAVITY_ALLIGNED_MAP(2);

    public int type;

    GetType(int type) {
        this.type = type;
    }
}
