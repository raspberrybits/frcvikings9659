package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkBase;  
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {
  private DifferentialDrive m_drivetrain;
  private XboxController joystick = new XboxController(0);
  private CANSparkBase leftRear = new CANSparkMax(1, MotorType.kBrushed);;
  private CANSparkBase leftFront = new CANSparkMax(2, MotorType.kBrushed);
  private CANSparkBase rightRear = new CANSparkMax(4, MotorType.kBrushed);
  private CANSparkBase rightFront = new CANSparkMax(3, MotorType.kBrushed);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);


  @Override
  public void robotInit() {
    leftRear.follow(leftFront);
    rightRear.follow(rightFront); 
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);  
  }

  @Override
  public void teleopPeriodic() {
    //negative on right
    m_drivetrain.tankDrive(joystick.getLeftY(), -joystick.getRightY());
  }
}