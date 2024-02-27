// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;


public class Drivetrain extends SubsystemBase {
  private final CANSparkBase leftRear = new CANSparkMax(DriverConstants.leftRearId, MotorType.kBrushed);
  private final CANSparkBase leftFront = new CANSparkMax(DriverConstants.leftFrontId, MotorType.kBrushed);
  private final CANSparkBase rightRear = new CANSparkMax(DriverConstants.rightRearId, MotorType.kBrushed);
  private final CANSparkBase rightFront = new CANSparkMax(DriverConstants.rightFrontId, MotorType.kBrushed);
  private final DifferentialDrive m_drivetrain = new DifferentialDrive(leftFront, rightFront); 

  private final RelativeEncoder encoderLeft = leftFront.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
  private final RelativeEncoder encoderRight = rightFront.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private Pose2d pose = new Pose2d(0, 0, gyro.getRotation2d());

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition(), pose);


  public Drivetrain() { 
    gyro.reset();
  
    leftRear.follow(leftFront);
    rightRear.follow(rightFront); 
  }

  public void drive(double left, double right){
    m_drivetrain.tankDrive(left, -right);
  }

  @Override
  public void periodic(){
    odometry.update(gyro.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());
    pose = odometry.getPoseMeters();
  }

  public double getPitch(){
    return -(gyro.getPitch());
  }

  public double getYaw(){
      return (gyro.getYaw());
  }

  public double getAngle(){
      return pose.getRotation().getDegrees();
  }

  public void stop() {
    leftFront.stopMotor();
    rightRear.stopMotor();
    rightFront.stopMotor();
    leftRear.stopMotor();
  }
}
