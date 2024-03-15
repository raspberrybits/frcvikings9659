package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DriverConstants;

public class driveDistance extends Command {
    private final double m_distance;
    private final Drivetrain m_drivetrain;
    private final double m_speed = DriverConstants.autoSpeed;

    private double m_distanceCounter;

    public driveDistance(double distance, Drivetrain drivetrain) {
        m_distance = distance;
        m_drivetrain = drivetrain;
        
        setName("driveDistance");
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.drive(0, 0);
        m_drivetrain.resetEncoders();
        m_distanceCounter = 0;
    }

    @Override
    public void execute() {
        m_distanceCounter = (- m_drivetrain.getLeftEncoder().getPosition()
                             + m_drivetrain.getRightEncoder().getPosition())
                            / 2;

        if (m_distance >= 0 && m_distanceCounter < m_distance){
            m_drivetrain.drive(m_speed, m_speed);
        } else if (m_distance < 0 && m_distanceCounter > m_distance){
            m_drivetrain.drive(-m_speed, -m_speed);
        }
    }

    @Override
    public boolean isFinished() {
        m_distanceCounter = (- m_drivetrain.getLeftEncoder().getPosition() 
                             + m_drivetrain.getRightEncoder().getPosition())
                            / 2;

        if (m_distance >= 0) {
            return m_distanceCounter >= m_distance;
        } else {
            return m_distanceCounter <= m_distance;
        }
    }
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

}