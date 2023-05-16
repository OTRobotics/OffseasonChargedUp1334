// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    // Driver and operator controllers
    private final OperatorInput  operatorInput  = new OperatorInput(
        OperatorInputConstants.DRIVER_CONTROLLER,
        OperatorInputConstants.OPERATOR_CONTROLLER);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // FIXME: Configure the default commands for all subsystems.

        // Configure the trigger bindings
        operatorInput.configureBindings(driveSubsystem);


    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        return null;
    }
}
