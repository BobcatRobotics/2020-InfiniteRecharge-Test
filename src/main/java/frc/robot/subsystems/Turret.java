package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    private WPI_TalonFX shootMotor2;
    private WPI_TalonFX shootMotor1;
    private WPI_TalonFX turretMotor;

    private double spinPower;
    private double power;

    public Turret() {
        
        shootMotor1 = new WPI_TalonFX(Constants.shootMotor1);
        shootMotor2 = new WPI_TalonFX(Constants.shootMotor2);
        turretMotor = new WPI_TalonFX(Constants.turretMotor);

        shootMotor1.setNeutralMode(NeutralMode.Brake);
        shootMotor2.setNeutralMode(NeutralMode.Brake);
        turretMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void setSpinPower(double spinPwr) {

        if (spinPwr > 1.0) {
            
            spinPwr = 1.0;
        } else if (spinPwr < -1.0) {
            
            spinPwr = -1.0;
        }

        this.spinPower = spinPwr;
    }

    public void moveTurret() {
        turretMotor.set(spinPower);
    }

    public void setPower(double pwr) {
        
        this.power = pwr;
    }

    public void shoot() {
        shootMotor1.set(power);
        shootMotor2.set(power);
    }

    public void stopTurret() {
        
        power = 0.0;
        
        turretMotor.stopMotor();
    }

    public void stopShooter() {
        
        spinPower = 0.0;
        
        shootMotor1.stopMotor();
        shootMotor2.stopMotor();
    }
    
}