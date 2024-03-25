package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.OperatorConstants;;

public class SysIdRoutineBot {
  // The robot's subsystems
  private final Drivetrain m_drive = new Drivetrain();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.driverPort);


  public void configureBindings() {
    m_drive.setDefaultCommand(
                new RunCommand(
                        () -> m_drive.drive(m_driverController.getLeftY(), m_driverController.getRightY()),
                        m_drive));

    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    // Using bumpers as a modifier and combining it with the buttons so that we can have both sets
    // of bindings at once
    m_driverController
        .a()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .x()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_driverController
        .a()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .x()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return m_drive.run(() -> {});
  }
}