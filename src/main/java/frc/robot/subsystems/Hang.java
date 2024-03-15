package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangConstants;;

public class Hang extends SubsystemBase {
    private final VictorSPX hookMotor = new VictorSPX(HangConstants.hangId);

    public Hang() {
    }

    public void drive(double speed) {
        hookMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }


}
