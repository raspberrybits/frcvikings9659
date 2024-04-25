package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ampShoot;
import frc.robot.commands.floorIntake;
import frc.robot.commands.floorReverse;
import frc.robot.commands.hangRetract;
import frc.robot.commands.pointAndShoot;
import frc.robot.commands.prime;
import frc.robot.commands.topIntake;
import frc.robot.commands.Autos.shootPreload;
import frc.robot.commands.Autos.taxi;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final Drivetrain mDrivetrain = new Drivetrain();
    private final Intake mIntake = new Intake();
    private final Shooter mShooter = new Shooter();
    private final Hang mHang = new Hang();

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController mDriver = new CommandXboxController(OperatorConstants.driverPort);
    private final CommandXboxController mControls = new CommandXboxController(OperatorConstants.controlsPort);

    public RobotContainer() {
        NamedCommands.registerCommand("prime", new prime(mShooter));
        NamedCommands.registerCommand("pointAndShoot", new pointAndShoot(mShooter, mIntake));
        NamedCommands.registerCommand("floorIntake", new floorIntake(mIntake));
        NamedCommands.registerCommand("ampShoot", new ampShoot(mShooter, mIntake));
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.setDefaultOption("Shoot Preload", new shootPreload(mShooter, mIntake));
        autoChooser.addOption("Taxi", new taxi(mDrivetrain));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Tank Drive (single controller)
        //FOR DUAL CONTROLLER SETUP COMMENT THIS BLOCK AND UNCOMMENT THE NEXT BLOCK
        /*mDrivetrain.setDefaultCommand(
                new RunCommand(
                        () -> mDrivetrain.drive(mControls.getLeftY(), mControls.getRightY()),
                        mDrivetrain));*/

        /* FOR DUAL CONTROLLER SETUP UNCOMMENT THIS BLOCK*/
        //Tank Drive (2 controllers)            
         mDrivetrain.setDefaultCommand(new RunCommand(
                () -> mDrivetrain.drive(mDriver.getLeftY(), mDriver.getRightY()),
                mDrivetrain));

        // Amp Shoot
        mControls

                .b()
                .whileTrue(
                new ampShoot(mShooter, mIntake).withTimeout(0.5));

        // Shooter Prime
        mControls
                .leftTrigger()
                .whileTrue(
                        new prime(mShooter));

        // Shooter Launch
        mControls
                .leftBumper()
                .whileTrue(
                        new pointAndShoot(mShooter, mIntake));

        // Floor Intake
        mControls
                .rightTrigger()
                .whileTrue(
                        new floorIntake(mIntake));

        // Floor Intake Reverse
        mControls
                .rightBumper()
                .whileTrue(
                        new floorReverse(mIntake));

        // Top Intake
        mControls
                .x()
                .whileTrue(
                        new topIntake(mShooter));

        // Hang Winch
        mControls
                .a()
                .whileTrue(
                        new hangRetract(mHang, HangConstants.speed));

        // Hang Unwinch
        mControls
                .y()
                .whileTrue(
                        new hangRetract(mHang, -HangConstants.speed));
        
        mControls
        .a()
        .and(mControls.rightBumper())
        .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        mControls
                .b()
                .and(mControls.rightBumper())
                .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        mControls
                .x()
                .and(mControls.rightBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        mControls
                .y()
                .and(mControls.rightBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        mControls
                .a()
                .and(mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        mControls
                .b()
                .and(mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        mControls
                .x()
                .and(mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        mControls
                .y()
                .and(mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        mDrivetrain.resetEncoders();
        mDrivetrain.resetGyro();
        return autoChooser.getSelected();
    }
}

