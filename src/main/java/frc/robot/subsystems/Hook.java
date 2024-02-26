package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;

public class Hook extends SubsystemBase {
    private final CANSparkBase hookMotor = new CANSparkMax(HookConstants.hookId, MotorType.kBrushed);

    public Hook() {}

    public void extend() {}

    public void retract() {}


}
