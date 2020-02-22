package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeederRun extends CommandBase {
    
    // TODO: This can be replaced with some sort of RunCommand type and an inline definition probably
    private final Feeder feeder;

    /**
     * 1. This feeds the balls to the shooter mechanism. <br>
     * 2. This starts the motors on the feeder. <br>
     * 3. The feeder is the system that takes the balls up into the shooter.
     */
    public FeederRun(Feeder fd) {
        this.feeder = fd;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.run();
    }
}