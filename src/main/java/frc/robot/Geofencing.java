package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class Geofencing {
    private String m_zoneName;
    private double m_top;
    private double m_left;
    private double m_bottom;
    private double m_right;

    public Geofencing(String zoneName, double top, double left, double bottom, double right) {
        m_zoneName = zoneName;
        m_top = top;
        m_left = left;
        m_bottom = bottom;
        m_right = right;
    }

    public boolean isInZone(Pose2d robotPose) {
        double x = robotPose.getX();
        double y = robotPose.getY();

        // System.out.println("Checking Zone: " + _zoneName);

        return x > m_left && x < m_right && y > m_bottom && y < m_top;
    }


}
