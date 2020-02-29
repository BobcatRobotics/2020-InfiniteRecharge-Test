package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavxGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.*;

// If a collision is detected in auto mode, then stop the robot for 1 second
public class StopAtCollision extends CommandBase {
    private NavxGyro gyro = new NavxGyro(SPI.Port.kMXP);
    private Drivetrain dt = new Drivetrain();
    double last_world_linear_accel_x = 0;
    double last_world_linear_accel_y = 0;
    final static double kCollisionThreshold_DeltaG = 0.5f; // Jerk (m/s^3) threshold
    public static boolean collisionDetected = false;

    public StopAtCollision(NavxGyro gyro, Drivetrain dt) {
        this.dt = dt;
        this.gyro = gyro;
        gyro.calibrate();
    }

    public void DetectCollision() {
        // Calculating jerk
        double curr_world_linear_accel_x = gyro.getWorldLinearAccelX();
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        double curr_world_linear_accel_y = gyro.getWorldLinearAccelY();
        last_world_linear_accel_y = curr_world_linear_accel_y;
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;

        // Testing the actual jerk against the threshold
        if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
                || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
            collisionDetected = true;
        }
    }

    public void StopIfCollision(boolean collisionDetected) throws InterruptedException {
        if (collisionDetected) {
            // Stop the drivetrain
            dt.stop();
        }
    }

    public void execute() {
        DetectCollision();
        try {
            StopIfCollision(collisionDetected);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}