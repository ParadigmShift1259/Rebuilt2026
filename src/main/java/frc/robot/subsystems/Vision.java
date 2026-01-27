package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
    private PhotonCamera m_photonCamera = new PhotonCamera("Camera1");
    // private List<PhotonPipelineResult> m_photonResults;
    private PhotonPipelineResult m_photonResult;
    private double m_timestamp = 0.0;
    private Pose2d m_robotPose = Pose2d.kZero;
    // private Field2d m_field = new Field2d();
    private static final String LIMELIGHT_NAME = "limelight-fuel";

    public double getTimestamp() { return m_timestamp; }
    public Pose2d getRobotPose() { return m_robotPose; }
    public boolean isTracking()  { return m_questNav.isTracking(); }
    
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

        //m_photonResults = m_photonCamera.getAllUnreadResults();
        m_photonResult = m_photonCamera.getLatestResult();
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

        public boolean photonIsTrackingFuel() {
        if (m_photonResult == null){
            return false;
        }
        return m_photonResult.hasTargets();
    }

    public double photonGetFuelAngle(){
        if (m_photonResult == null){
            return 0.0;
        }
        if (m_photonResult.hasTargets()){
            return m_photonResult.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public Transform2d photonGetTargetPose() {
        if (m_photonResult == null){
            return Transform2d.kZero;
        }
        if (m_photonResult.hasTargets()){
            //double dist = PhotonUtils.getDistanceToPose(m_robotPose, m_robotPose)
            Transform3d targetXform3d = m_photonResult.getBestTarget().getBestCameraToTarget();
            return new Transform2d(targetXform3d.getX(), targetXform3d.getY(), targetXform3d.getRotation().toRotation2d());
        }
        return Transform2d.kZero;
    }

    public double photonGetFuelPitch() {
        if (m_photonResult == null){
            return 0.0;
        }
        if (m_photonResult.hasTargets()){
            return m_photonResult.getBestTarget().pitch;
        }
        return 0.0;
    }

    // public boolean photonIsTrackingFuel() {
    //     if (m_photonResults == null){
    //         return false;
    //     }
    //     return m_photonResults.get(0).hasTargets();
    // }

    // public double photonGetFuelAngle(){
    //     if (m_photonResults == null){
    //         return 0.0;
    //     }
    //     if (m_photonResults.get(0).hasTargets()){
    //         return m_photonResults.get(0).getBestTarget().getYaw();
    //     }
    //     return 0.0;
    // }

    // public Transform3d photonGetTargetPose() {
    //     if (m_photonResults == null){
    //         return Transform3d.kZero;
    //     }
    //     if (m_photonResults.get(0).hasTargets()){
    //         return m_photonResults.get(0).getBestTarget().getBestCameraToTarget();
    //     }
    //     return Transform3d.kZero;
    // }
}