// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonIntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  public AutonIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        //Subsystems.m_shooterSubsystem.updateHasPassed();
        RobotContainer.m_robotShooter.intake();
        // RobotContainer.Subsystems.m_intakeSubsystem.runIntake(.5);
        // if(RobotContainer.Subsystems.m_intakeSubsystem.shouldIntakeStop())
        //   RobotContainer.Subsystems.m_intakeSubsystem.runIntake(0);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_robotShooter.intakeMotor.set(0);
    RobotContainer.m_robotShooter.conveyorMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.m_robotShooter.getDistanceValue() >= 1.2){
      RobotContainer.m_robotShooter.intakeMotor.set(0);
      RobotContainer.m_robotShooter.conveyorMotor.set(0);
    }
    return RobotContainer.m_robotShooter.getDistanceValue() >= 1.2;
    //return Subsystems.m_intakeSubsystem.shouldIntakeStop();
  }
}
