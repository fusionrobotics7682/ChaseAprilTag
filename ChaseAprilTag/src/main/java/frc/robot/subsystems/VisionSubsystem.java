// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera(Constants.DriveConstants.USB_CAMERA_NAME); // Declare the name of the camera used in the pipeline
  
  private static final Pose3d ROBOT_POSE  = new Pose3d(new Translation3d(), new Rotation3d());

  public double getBestTargetYaw() {
    try{
        return camera.getLatestResult().getBestTarget().getYaw();
    }catch(Exception exception){
        exception.printStackTrace();
    }
    return 0;
  }

  public Pose2d getTargetPose2d(){
    try {
      return ROBOT_POSE.transformBy(camera.getLatestResult().getBestTarget().getBestCameraToTarget()).toPose2d();
    } catch (Exception e) {
      e.printStackTrace();
    }
      return null;
  }

  @Override
  public void periodic() {
    try {
        // SmartDashboard.putNumber("X :", getTargetPose2d().getX());
        // SmartDashboard.putNumber("Y :", getTargetPose2d().getY());
        // SmartDashboard.putNumber("Degrees :", getTargetPose2d().getRotation().getDegrees());
    } catch (Exception e) {}
  }
}