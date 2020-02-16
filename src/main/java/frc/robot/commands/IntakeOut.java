package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeOut extends CommandBase {

    private Intake intake;

    /**
     * Hold down the right button on the gamepad to push a ball out onto the ground.
     * This changes the intake direction, not whether it is stowed out or not.
     */
    public IntakeOut(Intake itk) {
        this.intake = itk;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.runOut();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.stopMotor();
        }
    }
}