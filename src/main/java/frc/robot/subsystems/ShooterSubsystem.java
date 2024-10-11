package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public CANSparkMax conveyorMotor = new CANSparkMax(ShooterConstants.kConveyorMotorCanId, MotorType.kBrushless);
    public CANSparkMax intakeMotor = new CANSparkMax(ShooterConstants.kIntakeMotorCanId, MotorType.kBrushless);
    public CANSparkMax lowerShooterMotor = new CANSparkMax(ShooterConstants.kLowerMotorCanId, MotorType.kBrushless);
    public CANSparkMax upperShooterMotor = new CANSparkMax(ShooterConstants.kUpperMotorCanId, MotorType.kBrushless);
    public AnalogInput distanceSensor = new AnalogInput(Constants.ShooterConstants.kDistanceSensorId);
    
    private PIDController lowerPIDController = new PIDController(
        ShooterConstants.kLowerMotorP, ShooterConstants.kLowerMotorI, ShooterConstants.kLowerMotorD
    );
    private PIDController upperPIDController = new PIDController(
        ShooterConstants.kUpperMotorP, ShooterConstants.kUpperMotorI, ShooterConstants.kUpperMotorD
    );

    private PIDController intakePID = new PIDController(
        0.01,0, 0
    );

    private double targetRPM = Constants.ShooterConstants.SteadySpeedRPM;

    public ShooterSubsystem() {
        resetMotors();

        conveyorMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        lowerShooterMotor.setIdleMode(IdleMode.kCoast);
        upperShooterMotor.setIdleMode(IdleMode.kCoast);

        conveyorMotor.setSmartCurrentLimit(ShooterConstants.kConveyorMotorCurrentLimit);
        intakeMotor.setSmartCurrentLimit(ShooterConstants.kIntakeMotorCurrentLimit);
        lowerShooterMotor.setSmartCurrentLimit(ShooterConstants.kLowerMotorCurrentLimit);
        upperShooterMotor.setSmartCurrentLimit(ShooterConstants.kUpperMotorCurrentLimit);

        // // NOTE I think this makes the motors accelerate as fast as possible. How safe is
        // // this for the motors?
        lowerShooterMotor.setOpenLoopRampRate(0.0);
        upperShooterMotor.setOpenLoopRampRate(0.0);

        lowerPIDController.setTolerance(ShooterConstants.kTolerance);
        upperPIDController.setTolerance(ShooterConstants.kTolerance);
        intakePID.setTolerance(ShooterConstants.kTolerance);

        // distanceSensor.setAverageBits(ShooterConstants.kDistanceSensorAverageBits);
    }

    @Override
    public void periodic() 
    {

    }

    public void shoot()
    {
        conveyorMotor.set(Constants.ShooterConstants.conveyorEffort);
        intakeMotor.set(Constants.ShooterConstants.intakeEffort);
    }

    public void autonShoot()
    {
        conveyorMotor.set(Constants.ShooterConstants.conveyorEffort);
        intakeMotor.set(Constants.ShooterConstants.intakeEffort);
        fullSend();
    }

    public void resetMotors() {
        conveyorMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();
        lowerShooterMotor.restoreFactoryDefaults();
        upperShooterMotor.restoreFactoryDefaults();
    }

    public double intakeMotorRPM() {
        return intakeMotor.getEncoder().getVelocity();
    }
    public double lowerMotorRPM() {
        return lowerShooterMotor.getEncoder().getVelocity();
    }
    public double upperMotorRPM() {
        return upperShooterMotor.getEncoder().getVelocity();
    }

    public double conveyorMotorRPM() {
        return conveyorMotor.getEncoder().getVelocity();
    }


    public void intake() {
        if (getDistanceValue() <= 1.2)
        {
            // if (intakeMotorRPM() < 1500)
            // {
            //     intakeMotor.set(0.75);
            // }
            // else
            // {
            //     intakeMotor.set(0);
            // }

            intakeMotor.set(Constants.ShooterConstants.intakeEffort);
            conveyorMotor.set(Constants.ShooterConstants.conveyorEffort);
        
        } else
        {
            intakeMotor.set(0);
            conveyorMotor.set(0);
        }
    }

    public void defaultShooter()
    {
        intakeMotor.set(0);
        // upperShooterMotor.set(-0.5);
        // lowerShooterMotor.set(0.5);
        conveyorMotor.set(0);
        // if(isFinished(100))
        RPMShoot(targetRPM, targetRPM);  
        // else
        //     fullSend();
    }
    public void outake()
    {
        intakeMotor.set(-Constants.ShooterConstants.intakeEffort/2.0);
        conveyorMotor.set(Constants.ShooterConstants.conveyorEffort/2.0);
    }

    public void convey() {
        conveyorMotor.set(Constants.ShooterConstants.conveyorEffort);
    }

    public double getDistanceValue()
    {
        return distanceSensor.getAverageVoltage();
    }

    public boolean isFinishedAuton(double target)
    {
      return (Math.abs(lowerMotorRPM()) >= Math.abs((target - Constants.ShooterConstants.kTolerance)));
    }


    


    public void zeroShooter() {
        upperShooterMotor.set(0);
        lowerShooterMotor.set(0);
    }

    public void RPMShoot(double lower, double upper) {
        double lowerSetpoint = lowerPIDController.calculate(lowerMotorRPM(), -lower);
        double upperSetpoint = upperPIDController.calculate(upperMotorRPM(), -upper);

        lowerShooterMotor.setVoltage(lowerSetpoint);
        upperShooterMotor.setVoltage(upperSetpoint);
    }

    public void fullSend() {
        upperShooterMotor.setVoltage(-upperShooterMotor.getBusVoltage()*ShooterConstants.kFullSendVoltageScale);
        lowerShooterMotor.setVoltage(-lowerShooterMotor.getBusVoltage()*ShooterConstants.kFullSendVoltageScale);
    }

    // TODO We should replace `Math.abs(lowerMotorRPM())` with either `lowerMotorRPM()`
    // or `-lowerMotorRPM()`, depending on which one works well
    public boolean isFinished(double target, double offset) {
        // return Math.abs(Math.abs(lowerMotorRPM()) - (target + offset)) <= ShooterConstants.kTolerance;
        return Math.abs(lowerMotorRPM()) >= (target + offset) - ShooterConstants.kTolerance;
        // The 1st (commented) line is only true if the lower motor RPM is within tolerance of target+offset
        // The 2nd line is also true if the lower motor RPM is any value above target+offset
    }
    public boolean isFinished(double offset)
    {
      return (Math.abs(lowerMotorRPM())-offset >= Math.abs((targetRPM - Constants.ShooterConstants.kTolerance)));
    }

    public boolean isFinished() {
        return isFinished(0);
    }
    // isFinishedAuton(double target) is now isFinished(double offset) (they do the same exact thing)
    public boolean isFinishedAuton() {
        return isFinished(AutoConstants.kSubwooferTopRPM);
    }

    //TODO: delete
    public Command exampleMethodCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'exampleMethodCommand'");
    }
}
