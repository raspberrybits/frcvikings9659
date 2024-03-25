package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pointAndShoot;
import frc.robot.commands.prime;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class shootPreload extends SequentialCommandGroup {
    public shootPreload(Shooter mShooter, Intake mIntake) {
        addCommands(
            new prime(mShooter)
            .withTimeout(2),
            new pointAndShoot(mShooter, mIntake)
            .withTimeout(2)
        );
    }
}
