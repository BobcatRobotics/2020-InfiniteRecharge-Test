package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private Solenoid intakeSolenoid;
    private WPI_TalonSRX intakeMotor;

    private double in = -1.0;
    private double out = 1.0;

    private boolean state = false;

    /**
     * The intake pulls the balls from the ground into the robot.
     * The balls are then stored in the feeder to wait to be shot.
     */
    public Intake() {
        intakeMotor = new WPI_TalonSRX(Constants.intakeMotor);
        intakeSolenoid = new Solenoid(Constants.intakeSolenoid);

        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeSolenoid.set(true);
    }

    /**
     * Takes a ball in from the ground.
     */
    public void runIn() {
        intakeMotor.set(in);
    }

    /**
     * Pushes out a ball out from the feeder.
     */
    public void runOut() {
        intakeMotor.set(out);
    }

    public void stopMotor() {
        intakeMotor.set(0.0);
    }

    /**
     * Brings the intake system in and out.
     * It can be stowed away or out.
     */
    public void toggle() {
        intakeSolenoid.set(!state);
    }
}