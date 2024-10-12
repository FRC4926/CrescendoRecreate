// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class CenterTwoNote {
    public static int pipelineNum;

    public CenterTwoNote() {
        RobotContainer.driveSubsystem.resetOdometry();
    }
    public static Command getCommand(){
        return new AutonShooterCommand()
        .andThen(new AutonIntakeCommand()).alongWith(new AutonDriveCommand(0.75, 0.5))
        .andThen(new AutonDriveCommand(0.5, -0.5))
        .andThen(new AutonShooterCommand());

    }
}
