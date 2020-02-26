package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class PullClimber extends CommandBase {

    private Climber climber;

    public PullClimber(Climber cl) {

        this.climber = cl;
        addRequirements(climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
        climber.climb();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            climber.stop();
        }
    }
}