package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.List;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import frc.robot.LimelightHelpers;

import frc.robot.Constants;

@Logged
public class Vision extends SubsystemBase {
    
    private QuestNav m_questNav = new QuestNav();
    private double m_timestamp = 0.0;
    private Pose2d m_robotPose = Pose2d.kZero;

    private final Transform2d ROBOT_TO_QUEST = new Transform2d(-0.22, 0.225, Rotation2d.k180deg);
    private final Transform3d ROBOT_TO_QUEST3d = new Transform3d(ROBOT_TO_QUEST);


    public double getTimestamp() { return m_timestamp; }
    //public Pose2d getQuestRobotPose() { return m_robotPose.transformBy(ROBOT_TO_QUEST); }
    public Pose2d getQuestRobotPose() { return m_robotPose.transformBy(ROBOT_TO_QUEST.inverse()); }
    // Always return blue since that is the origin and we want the field coordinates
    public Pose3d getLLRobotPose() {return LimelightHelpers.getBotPose3d_wpiBlue(Constants.LIMELIGHT_NAME); }
    public LimelightHelpers.PoseEstimate getBotPoseEstimate() { return LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LIMELIGHT_NAME); }
    public Pose2d getBotPoseMegaTag2() { return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_NAME).pose; }
    public LimelightHelpers.PoseEstimate getBotPoseEstimateMegaTag2() { return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_NAME); }
    public LimelightHelpers.PoseEstimate getRedPoseEstimate() { return LimelightHelpers.getBotPoseEstimate_wpiRed(Constants.LIMELIGHT_NAME); }
    public boolean isTracking()  { return m_questNav.isTracking(); }
    public boolean isLLTracking() { return LimelightHelpers.getTA(Constants.LIMELIGHT_NAME) != 0; }
    
    public Vision() {

    }

    @Override 
    public void periodic() {
        m_questNav.commandPeriodic();

        SmartDashboard.putBoolean("QuestTracking", m_questNav.isTracking());
        SmartDashboard.putBoolean("LLTracking", isLLTracking());

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
                SmartDashboard.putNumber("QuestPoseRot", questPose.getRotation().getAngle() * 180 / Math.PI);

                SmartDashboard.putNumber("2DQuestPoseX", questPose.toPose2d().getX());
                SmartDashboard.putNumber("2DQuestPoseY", questPose.toPose2d().getY());
            }
        }
        SmartDashboard.putNumber("LLtag", LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME));
    }

    public void setQuestPose(Pose3d pose) {
        m_questNav.setPose(pose.transformBy(ROBOT_TO_QUEST3d));
    }

    public double getFuelAngle() {
        return LimelightHelpers.getTX(Constants.LIMELIGHT_NAME);
    }

    public boolean isTrackingFuel() {
        return LimelightHelpers.getTV(Constants.LIMELIGHT_NAME);
    }

    public double[] getTargetPose() {
        return LimelightHelpers.getTargetPose_RobotSpace(Constants.LIMELIGHT_NAME);
    }

    public void updateQuestPose(){
        if (isLLTracking()) {
            m_questNav.setPose(getLLRobotPose()/* .rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) */);
        }
        else {
            System.out.println("LimeLight not tracking, skipping Quest reset");
        }
    }
}