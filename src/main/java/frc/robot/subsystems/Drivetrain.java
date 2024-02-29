// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;

import org.photonvision.PhotonCamera;

public class Drivetrain extends SubsystemBase {
  private final CANSparkBase leftRear = new CANSparkMax(DriverConstants.leftRearId, MotorType.kBrushed);
  private final CANSparkBase leftFront = new CANSparkMax(DriverConstants.leftFrontId, MotorType.kBrushed);
  private final CANSparkBase rightRear = new CANSparkMax(DriverConstants.rightRearId, MotorType.kBrushed);
  private final CANSparkBase rightFront = new CANSparkMax(DriverConstants.rightFrontId, MotorType.kBrushed);
  private final DifferentialDrive drivetrain = new DifferentialDrive(leftFront, rightFront);

  private final RelativeEncoder encoderLeft = leftFront.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
  private final RelativeEncoder encoderRight = rightFront.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private Pose2d pose = new Pose2d(0, 0, gyro.getRotation2d());

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition(), pose);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriverConstants.trackWidth);


  public Drivetrain() { 
    gyro.reset();
  
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    AutoBuilder.configureRamsete(
            this::getPose, 
            this::resetPose, 
            this::getChassiSpeeds,
            this::drive,
            new ReplanningConfig(),
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
  }

  public void drive(double left, double right){
    SmartDashboard.putNumber("Encoder Left", encoderLeft.getPosition());
    SmartDashboard.putNumber("Encoder Right", encoderRight.getPosition());
    drivetrain.tankDrive(left, right);
  }

  public void drive(ChassisSpeeds speed){
    drivetrain.tankDrive(kinematics.toWheelSpeeds(speed).leftMetersPerSecond, kinematics.toWheelSpeeds(speed).rightMetersPerSecond);
  }

  @Override
  public void periodic(){
    odometry.update(gyro.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());
    pose = odometry.getPoseMeters();
  }

  public ChassisSpeeds getChassiSpeeds() {
    return new ChassisSpeeds(encoderLeft.getVelocity(), encoderRight.getVelocity(), gyro.getAngle());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition(), pose);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    drivetrain.feed();
  }

  public void resetEncoders() {
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (encoderLeft.getPosition() + encoderRight.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return encoderLeft;
  }


  public RelativeEncoder getRightEncoder() {
    return encoderRight;
  }


  public void setMaxOutput(double maxOutput) {
    drivetrain.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void stop() {
    leftFront.stopMotor();
    rightRear.stopMotor();
    rightFront.stopMotor();
    leftRear.stopMotor();
  }
}
