package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final Solenoid intakeSolenoid;
    private final WPI_TalonSRX intakeMotor;
    private final WPI_TalonSRX intakeMid;
    private final WPI_TalonFX intakeLowerTower;

    private final double inSpeed = -1.0;
    private final double outSpeed = 1.0;

    /**
     * The intake pulls the balls from the ground into the robot.
     * The balls are then stored in the feeder to wait to be shot.
     * The intake can be stowed in or stowed out.
     * The direction of the intake can also be switched.
     */
    public Intake() {
        intakeMotor = new WPI_TalonSRX(intakeFrontMotorPort);
        intakeMid = new WPI_TalonSRX(intakeMidMotorPort);
        intakeLowerTower = new WPI_TalonFX(intakeLowerTowerPort);

        intakeSolenoid = new Solenoid(intakeSolenoidPort);

        // When the motor is in neutral mode the motor will keep moving easily (coast)
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMid.setNeutralMode(NeutralMode.Brake);
        intakeLowerTower.setNeutralMode(NeutralMode.Brake);

        intakeSolenoid.set(true);
    }

    /**
    * 1. Changes the intake direction to take in balls from the ground. <br>
    * 2. Hold the right trigger to activate the command. <br>
    * 3. Balls from the intake are then stored in the feeder. <br>
    * 4. This changes the intake direction, not whether it is stowed in or not.
    */
    public void runIn() {
        intakeMotor.set(inSpeed);
        intakeMid.set(inSpeed);
        intakeLowerTower.set(inSpeed);
    }

    /**
    * 1. Changes the intake direction to take in balls from the ground. <br>
    * 2. Hold the right button to activate the command. <br>
    * 3. Balls can be pushed out onto the ground. <br>
    * 4. This changes the intake direction, not whether it is stowed out or not.
    */
    public void runOut() {
        intakeMotor.set(outSpeed);
        intakeMid.set(outSpeed);
        intakeLowerTower.set(outSpeed);
    }

    /**
    * Stops the intake motor.
    */
    public void stopMotor() {
        intakeMotor.set(0.0);
        intakeMid.set(0.0);
        intakeLowerTower.set(0.0);
    }

    /**
    * 1. This command stows in the intake system and puts it out.
    * 2. Press the A button to toggle this command.
    * 3. This works by turning the intake solenoid on and off.
    * 4. It will toggle it from stowed in to out or vice versa.
    * 5. This is different from turning the intake direction from in and out.
    */
    public void toggle() {
        intakeSolenoid.set(!intakeSolenoid.get());
    }
}