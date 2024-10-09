package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armSparkMax = new CANSparkMax(ArmConstants.kArmMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder armEncoder = armSparkMax.getEncoder();
    private final PIDController armPIDController = new PIDController(
        ArmConstants.kArmMotorP, ArmConstants.kArmMotorI, ArmConstants.kArmMotorD
    );
    private final double startTime = WPIUtilJNI.now();
    private double setpoint = 0.0;
    public ArmSubsystem() {
        armEncoder.setPosition(0.0);
        armEncoder.setPositionConversionFactor(1);
    }
    public void reset() {
        armEncoder.setPosition(0.0);
    }
    // Go to angle in radians
    public void goToAngle(double angleRadians) {
        goToAngle(Rotation2d.fromRadians(angleRadians));
    }
    public void goToAngle(Rotation2d angle) {
        setpoint = angle.getRotations();
    }
    public boolean atAngle() {
        return armPIDController.atSetpoint();
    }
    public void periodic() {
        double output = armEncoder.getPosition();
        double input = armPIDController.calculate(output, setpoint);
        // armSparkMax.set(armPIDController.calculate(armEncoder.getPosition(), setpoint));
        SmartDashboard.putNumber("armOutput", output);
        SmartDashboard.putNumber("armInput", input);
    }
    public void sendData() {

    }
}
