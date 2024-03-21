package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangConstants;;

public class Hang extends SubsystemBase {
    private final CANSparkBase hookMotor = new CANSparkMax(HangConstants.hangId, MotorType.kBrushless);

    public Hang() {
    }

    public void drive(double speed) {
        
        hookMotor.set(speed);
    }


}
