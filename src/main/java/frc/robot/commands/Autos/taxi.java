package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.iDrive;
import frc.robot.subsystems.Drivetrain;

public class taxi extends SequentialCommandGroup {
    public taxi(Drivetrain mDrivetrain) {
        addCommands(
            new iDrive(mDrivetrain, () -> 0.5, () -> 0.5).
            withTimeout(3)
        );
    }
}
