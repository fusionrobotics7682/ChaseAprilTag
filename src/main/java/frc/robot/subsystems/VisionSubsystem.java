// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera(Constants.DriveConstants.USB_CAMERA_NAME); // Declare the name of the camera used in the pipeline
  
  public double getBestTargetYaw() {
    try{
        return camera.getLatestResult().getBestTarget().getYaw();
    }catch(Exception exception){
        exception.printStackTrace();
    }
    return 0;
  }

  public double getBestXTarget(){
    try{
      return camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
    }catch(Exception exception){
      exception.printStackTrace();
    }
    return 0;
  }

  @Override
  public void periodic() {
    // try {
    //     photonTrackedTarget = camera.getLatestResult().getBestTarget();
    //     SmartDashboard.putNumber("April Tag Yaw :", photonTrackedTarget.getYaw());
    // } catch (Exception e) {
    //     e.printStackTrace();
    // }
    try {
        SmartDashboard.putNumber("APril Tag Yaw :", camera.getLatestResult().getBestTarget().getYaw());
        //SmartDashboard.putNumber("Get best target id :", camera.getLatestResult().getBestTarget().getFiducialId());
        SmartDashboard.putNumber("X :", camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX());
        SmartDashboard.putNumber("Y :", camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY());
        SmartDashboard.putNumber("Z :", camera.getLatestResult().getBestTarget().getBestCameraToTarget().getZ());
    } catch (Exception e) {
        // TODO: handle exception
    }
  }
}