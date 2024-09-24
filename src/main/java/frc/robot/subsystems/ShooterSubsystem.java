package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem {
    private CANSparkMax conveyorSparkMax = new CANSparkMax(ShooterConstants.kConveyorMotorCanId, MotorType.kBrushless);
    public ShooterSubsystem() {

    }

    public void convey(double effort) {
        conveyorSparkMax.set(effort);
    }
}
