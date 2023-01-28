// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final AHRS navx = new AHRS(Port.kMXP);

    // Motors
    // left
    WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_PIN_1);
    WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_PIN_2);
    WPI_VictorSPX leftMotor3 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_PIN_3);
    // right
    WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_PIN_1);
    WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_PIN_2);
    WPI_VictorSPX rightMotor3 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_PIN_3);

    // Groups
    MotorControllerGroup leftGroup = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    MotorControllerGroup rightGroup = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

    // Differential
    private DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    private DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(Constants.DriveConstants.DIFFERENTIAL_DRIVE_KINEMATICS, new Rotation2d(), 0, 0, new Pose2d());
    private Field2d field2d = new Field2d();

        // Filters
        private final SlewRateLimiter linearLimiter = new SlewRateLimiter(4.5);
        private final SlewRateLimiter angularLimiter = new SlewRateLimiter(3);

    Joystick joystick;

    private final VisionSubsystem visionSubsystem;

/** Creates a new DriveSubsystem. */
 public DriveSubsystem(Joystick joystick, VisionSubsystem visionSubsystem) {
  this.visionSubsystem = visionSubsystem;
  this.joystick = joystick;
  rightGroup.setInverted(true);
 }

@Override
public void periodic() {  
  differentialDrivePoseEstimator.addVisionMeasurement(visionSubsystem.getTargetPose2d(), Timer.getFPGATimestamp());

  SmartDashboard.putNumber("estimation x:", differentialDrivePoseEstimator.getEstimatedPosition().getX());
  SmartDashboard.putNumber("estimation y:", differentialDrivePoseEstimator.getEstimatedPosition().getY());
  SmartDashboard.putNumber("estimation degrees:", differentialDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
  //SmartDashboard.putData(field2d);
  SmartDashboard.putNumber("Navx Yaw :", navx.getYaw());
}

public void arcadeDrive(){
  // Will be add filters
 differentialDrive.arcadeDrive(linearLimiter.calculate(joystick.getY()), -angularLimiter.calculate(joystick.getZ()*0.7)); // lineer, rotation
}

public void resetNavX(){
 navx.reset();
}

public void stopDrive(){
 differentialDrive.stopMotor();
}

/** Zeroes the heading of the robot. */
public void zeroHeading() {
 navx.reset();
}

public double getYaw(){
 return navx.getYaw();
}

/**
* Returns the heading of the robot.
*
* @return the robot's heading in degrees, from -180 to 180
*/
public double getHeading() {
return navx.getRotation2d().getDegrees();
}

/**
* Returns the turn rate of the robot.
*
* @return The turn rate of the robot, in degrees per second
*/
public double getTurnRate() {
return -navx.getRate();
}
}
