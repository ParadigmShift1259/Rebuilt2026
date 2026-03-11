package frc.robot;

public class Constants {
    public static Geofencing m_geofenceNeutZoneIfBlue = new Geofencing(18.04, 4.053, 0.0, 16.51);
    public static Geofencing m_geofenceNeutZoneIfRed = new Geofencing(18.04, 0.0, 0.0, 12.417);
    public static Geofencing m_geofenceNeutTop = new Geofencing(18.04, 0.0, 6.9, 16.51);
    public static Geofencing m_geofenceNeutBottom = new Geofencing(1.143, 0.0, 0.0, 16.51);
    public static Geofencing m_geofenceRedBump = new Geofencing(6.4912, 11.3 - 0.4, 1.589, 12.417 + 0.4);
    public static Geofencing m_geofenceBlueBump = new Geofencing(6.4912, 4.053 - 0.4, 1.589, 5.17 + 0.4);
    public static double c_hubY = 4.0;

    public static boolean defaultFlywheel = false; //False means the flywheel is on
    public static double rpmBoost = -50.0;

    public static double m_turretOffsetY = 0.14;;
    public static double m_turretOffsetX = 0.18;

    public static final String LIMELIGHT_NAME = "limelight-fuel";
}
