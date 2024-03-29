package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class IntakeIn extends CommandBase {

    private Intake intake;
    private final XboxController gamepad;

    /**
     * 1. Changes the intake direction to take in balls from the ground. <br>
     * 2. Hold the right trigger to activate the command. <br>
     * 3. Balls from the intake are then stored in the feeder. <br>
     * 4. This changes the intake direction, not whether it is stowed in or not.
     */
    public IntakeIn(Intake itk, XboxController gpd) {
        this.intake = itk;
        this.gamepad = gpd;
        addRequirements(intake);
    }

    /**
     * Spins the motor so balls are taken in
     */
    @Override
    public void execute() {
        //if right bumper pressed and NOT right trigger pressed
        if (gamepad.getRawButton(Constants.Right_Bumper_Button) 
        && !gamepad.getRawButton(Constants.Right_Trigger_Button)) {
            intake.runIntakeIn(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.stopIntake();
        }
    }
}