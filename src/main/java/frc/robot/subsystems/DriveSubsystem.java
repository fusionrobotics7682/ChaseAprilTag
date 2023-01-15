// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

    public ProfiledPIDController yawPidController = new ProfiledPIDController(1.5 , 0, 0.05, new TrapezoidProfile.Constraints(3, 3));

        // Filters
        private final SlewRateLimiter linearLimiter = new SlewRateLimiter(2.5);
        private final SlewRateLimiter angularLimiter = new SlewRateLimiter(3);

    Joystick joystick;

/** Creates a new DriveSubsystem. */
 public DriveSubsystem(Joystick joystick) {
  this.joystick = joystick;
    yawPidController.setTolerance(3);
 }

@Override
public void periodic() {
    SmartDashboard.putNumber("Navx Yaw :", navx.getYaw());
}

public void arcadeDrive(){
  // Will be add filters
 differentialDrive.arcadeDrive(linearLimiter.calculate(-joystick.getRawAxis(1)), angularLimiter.calculate(joystick.getRawAxis(2)*0.7)); // lineer, rotation
}

public void aprilDrive(double currentYaw, double targetYaw){
  differentialDrive.arcadeDrive(0, yawPidController.calculate(currentYaw, targetYaw));
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
