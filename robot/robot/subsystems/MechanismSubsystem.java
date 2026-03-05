package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismSubsystem extends SubsystemBase {

    // Motor ID'leri
    private static final int ELEVATOR1 = 100;
    private static final int ELEVATOR2 = 101;

    private static final int INTAKE = 102;
    private static final int INDEXER = 103;

    private static final int SHOOTER1 = 104;
    private static final int SHOOTER2 = 105;

    // Motorlar
    private final TalonFX elevator1 = new TalonFX(ELEVATOR1);
    private final TalonFX elevator2 = new TalonFX(ELEVATOR2);

    private final TalonFX intake = new TalonFX(INTAKE);
    private final TalonFX indexer = new TalonFX(INDEXER);

    private final TalonFX shooter1 = new TalonFX(SHOOTER1);
    private final TalonFX shooter2 = new TalonFX(SHOOTER2);

    public MechanismSubsystem() {}

    // ---------------- ELEVATOR ----------------

    public void elevatorUp() {
        elevator1.set(1.0);
        elevator2.set(1.0);
    }

    public void elevatorDown() {
        elevator1.set(-1.0);
        elevator2.set(-1.0);
    }

    public void elevatorStop() {
        elevator1.set(0);
        elevator2.set(0);
    }

    // ---------------- INTAKE ----------------

    public void intakeStart() {
        intake.set(1.0);
    }

    public void intakeStop() {
        intake.set(0);
    }

    // ---------------- INDEXER ----------------

    public void indexerForward() {
        indexer.set(1.0);
    }

    public void indexerStop() {
        indexer.set(0);
    }

    // ---------------- SHOOTER ----------------

    public void shooterShoot() {
        shooter1.set(1.0);
        shooter2.set(-1.0);
    }

    public void shooterStop() {
        shooter1.set(0);
        shooter2.set(0);
    }
}