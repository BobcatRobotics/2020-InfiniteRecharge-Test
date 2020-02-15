package frc.robot.subsystems;
import java.util.List;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private boolean tTarget;
    private double tx;
    private double ty;
    private double ta;
    private double ta0;
    private double ta1;
    private NetworkTable limelight;

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        try {
            tTarget = getDouble("tv") == 1.0;
            tx = getDouble("tx");
            ty = getDouble("ty");
            ta = getDouble("ta");
            ta0 = getDouble("ta0");
            ta1 = getDouble("ta1");
        } catch (Exception e) {
            System.out.println(String.format("Error initializing limelight. Error message: %s", e));
        }
    }

    public boolean hasTargets() {
        tTarget = getEntry("tv");
        return tTarget;
    }

    public double x() {
        tx = getEntry("tx");
        return tx;
    }

    public double y() {
        ty = getEntry("ty");
        return ty;
    }

    public double targetArea() {
        ta = getEntry("ta");
        return ta;
    }

    public double rightTarget() {
        ta0 = getEntry("ta0");
        return ta0;
    }

    public double leftTarget() {
        ta1 = getEntry("ta1")
    }

    public void turnOnLED() {
        lightLED(LimelightLED.ON);
    }

    public void turnOffLED() {
        lightLED(LimelightLED.OFF);
    }

    /**
     * Sets the Limelight camera to VISION mode. (with green filter)
     */
    public void turnOnCam() {
        lightCam(LimelightCAM.VISION);
    }

    /**
     * Sets the Limelight camera to DRIVER mode. (no filter)
     */
    public void turnOffCam() {
        lightCam(LimelightCAM.DRIVER);
    }

    public void lightLED(LimelightLED state) {
        setNumber("ledMode", state.ordinal());
        System.out.println("Setting LimeLight LEDs to " + state.ordinal());
    }

    public LimelightLED getLED() {
        List<LimelightLED> modes = [LimelightLED.PIPELINE, LimelightLED.OFF, LimelightLED.BLINK, LimelightLED.ON];
        return modes[getEntry("ledMode")];
    }

    public void lightCam(LimelightCAM state) {
        setNumber("camMode", state.ordinal());
        System.out.println("Setting LimeLight CAMs to " + state.ordinal());
    }

    public LimelightCAM getCAM() {
        List<LimelightCAM> modes = [LimelightCAM.PIPELINE, LimelightCAM.OFF, LimelightCAM.BLINK, LimelightCAM.ON];
        return modes[getEntry("ledMode")];
    }

    public void setNumber(String entry, int state) {
        getEntry(entry).setNumber(state);
    }

    public NetworkTableEntry getEntry(String table) {
        return limelight.getEntry(table);
    }

}