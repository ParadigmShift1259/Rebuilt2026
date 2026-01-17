package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

public class Vision implements Subsystem {
    
    private QuestNav m_questNav = new QuestNav();
    private double m_timestamp = 0.0;
    private Pose2d m_robotPose = Pose2d.kZero;
    private Field2d m_field = new Field2d();

    public double getTimestamp() { return m_timestamp; }
    public Pose2d getRobotPose() { return m_robotPose; }
    
    public Vision() {
        SmartDashboard.putData(m_field);
    }

    @Override 
    public void periodic() {
        
        m_questNav.commandPeriodic();

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
                m_robotPose = questPose.toPose2d(); // TO DO: offset from quest to center of robot

                // TO DO: You can put some sort of filtering here if you would like! 

                // Add the measurement to our estimator
                
                SmartDashboard.putNumber("QuestPoseX", questPose.getX());
                SmartDashboard.putNumber("QuestPoseY", questPose.getY());
                SmartDashboard.putNumber("QuestPoseZ", questPose.getZ());

                SmartDashboard.putNumber("2DQuestPoseX", questPose.toPose2d().getX());
                SmartDashboard.putNumber("2DQuestPoseY", questPose.toPose2d().getY());

                m_field.setRobotPose(questPose.toPose2d());

            }
        }
    }

    public void setQuestPose(Pose3d pose) {
        m_questNav.setPose(pose);
    }
}