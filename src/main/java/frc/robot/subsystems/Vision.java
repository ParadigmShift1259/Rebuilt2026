package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    
    private QuestNav m_questNav = new QuestNav();
    private double m_timestamp = 0.0;
    private Pose2d m_robotPose = Pose2d.kZero;
    // private Field2d m_field = new Field2d();
    private static final String LIMELIGHT_NAME = "limelight-fuel";

    private final Transform2d ROBOT_TO_QUEST = new Transform2d(0.19, 0.0, Rotation2d.kZero);

    public double getTimestamp() { return m_timestamp; }
    public Pose2d getQuestRobotPose() { return m_robotPose.transformBy(ROBOT_TO_QUEST.inverse()); }
    public Pose3d getLLRobotPose() {return LimelightHelpers.getBotPose3d_wpiBlue(LIMELIGHT_NAME); }
    public boolean isTracking()  { return m_questNav.isTracking(); }
    public boolean isLLTracking() { return LimelightHelpers.getTA(LIMELIGHT_NAME) != 0; }
    
    public Vision() {
    // SmartDashboard.putData("RobotPose", m_field);
    }

    @Override 
    public void periodic() {
        
        m_questNav.commandPeriodic();

        SmartDashboard.putBoolean("QuestTracking", m_questNav.isTracking());

        if (m_questNav.isTracking()) {
            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = m_questNav.getAllUnreadPoseFrames();

            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                m_timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose 
                m_robotPose = questPose.toPose2d(); // TODO: offset from quest to center of robot

                // TODO: You can put some sort of filtering here if you would like! 

                // Add the measurement to our estimator
                
                SmartDashboard.putNumber("QuestPoseX", questPose.getX());
                SmartDashboard.putNumber("QuestPoseY", questPose.getY());
                SmartDashboard.putNumber("QuestPoseZ", questPose.getZ());

                SmartDashboard.putNumber("2DQuestPoseX", questPose.toPose2d().getX());
                SmartDashboard.putNumber("2DQuestPoseY", questPose.toPose2d().getY());

                // m_field.setRobotPose(questPose.toPose2d());

            }
        }
    }

    public void setQuestPose(Pose3d pose) {
        m_questNav.setPose(pose);
    }

    public double getFuelAngle() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }

    public boolean isTrackingFuel() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }

    public double[] getTargetPose() {
        return LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_NAME);
    }

    public void updateQuestPose(){
        m_questNav.setPose(getLLRobotPose());
    }

}