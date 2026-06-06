package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static Geofencing m_geofenceNeutZoneIfBlue = new Geofencing(8.04, 4.053, 0.0, 16.51);
    public static Geofencing m_geofenceNeutZoneIfRed = new Geofencing(8.04, 0.0, 0.0, 12.417);
    public static Geofencing m_geofenceEnemyZoneIfRed = new Geofencing(8.04, 0.0, 0.0, 4.0);
    public static Geofencing m_geofenceEnemyZoneIfBlue = new Geofencing(8.04, 12.0, 0.0, 16.51);
    public static Geofencing m_geofenceNeutTop = new Geofencing(8.04, 0.0, 6.9, 16.51);
    public static Geofencing m_geofenceNeutBottom = new Geofencing(1.143, 0.0, 0.0, 16.51);
    public static Geofencing m_geofenceRedBump = new Geofencing(6.4912, 11.3 - 0.4, 1.589, 12.417 + 0.4);
    public static Geofencing m_geofenceBlueBump = new Geofencing(6.4912, 4.053 - 0.4, 1.589, 5.17 + 0.4);
    public static double c_hubY = 4.0;

    public static boolean defaultFlywheel = false; //False means the flywheel is on
    public static double rpmBoost = 0.0;
    public static double offsetRpmAdjLimit = 800.0;
    public static double turretTweakAdjLimit = 30.0;
    //public static double bumpLimitFactor = 0.5;     // Use early in Quals to save wear and tear
    public static double bumpLimitFactor = 0.9;   // Use later in Quals when under heavy defense
    public static Pose2d poseDemoBlueHome = new Pose2d(2.612, 4.0, Rotation2d.kZero);

    public static double m_turretOffsetY = 0.14;
    public static double m_turretOffsetX = 0;//0.18;
    public static double m_defaultNegXVelOffsetDegrade = 0.8;
    public static double m_defaultNegYVelOffsetDegrade = 0.8;

    //public static double m_turretLimitAngle = Math.PI / 2.0;
    //public static double m_turretLimitAngle = (Math.PI / 2.0) + (15.0 * Math.PI / 180);
    public static double m_turretLimitAngle = (Math.PI / 2.0) + (30.0 * Math.PI / 180);
    //public static double m_maxTurns = 12.5;  // 90 deg
    //public static double m_maxTurns = 14.2;  // 105 deg (+15)
    public static double m_maxTurns = 16.3;    // 120 deg (+30)

    // public static final String LIMELIGHT_NAME = "limelight-fuel";
    public static final String LIMELIGHT_NAME = "limelight-hub";

}
