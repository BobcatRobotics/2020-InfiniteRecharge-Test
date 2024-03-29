package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class MoveTurret extends CommandBase {
    
    // If the speed of the motor is less than this threshold, we'll just set it to zero
    private double power = 0.0;
    public static final double percentToTurnMotorWhenCommandedByButton = .05;

    private final Turret turret;
    private final XboxController gamepad;

    /**
     * 1. Comply with limit switchs <br>
     * 3. Takes power value from the left joystick on the game pad <br>
     * 4. Comply with minimum power threshold <br>
     * 5. Set the turret's speed
     * 
     * @param trrt Turret subsystem
     * @param gmpd XboxController instance
     */
    public MoveTurret(Turret trrt, XboxController gmpd) {
        this.turret = trrt;
        this.gamepad = gmpd;
        addRequirements(turret);
    }
    
    @Override
    public void execute() {
        double rightJoyStickXAxis = gamepad.getRawAxis(Joystick.AxisType.kY.value);
        boolean rightJoyStickDown = gamepad.getRawButton(Joystick.AxisType.kX.value);

        //If right is NOT pressed down AND joystick x axis is moved (disregarding nominal movement)
        //Is .02 a good threshold?
        if (!rightJoyStickDown && (rightJoyStickXAxis < -.02 || rightJoyStickXAxis > .02)) {
            //Is this too lower, high?
            turret.setSpinPower(rightJoyStickXAxis);
        }

        
    }

    /**
     * Stops the turret when the command ends
     */
    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();
    }
}