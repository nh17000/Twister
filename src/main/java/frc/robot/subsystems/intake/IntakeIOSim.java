package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim extends IntakeIOTalonFX {
    private final DCMotorSim rollerPhysicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    IntakeConstants.ROLLER_MOTOR, IntakeConstants.ROLLER_SHAFT_MOI, IntakeConstants.ROLLER_GEAR_RATIO),
            IntakeConstants.ROLLER_MOTOR);

    private final TalonFXSimState rollerSimState;

    public IntakeIOSim() {
        rollerSimState = rollerMotor.getSimState();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        super.updateInputs(inputs);

        rollerSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        rollerPhysicsSim.setInputVoltage(rollerSimState.getMotorVoltage());
        rollerPhysicsSim.update(Constants.LOOP_PERIOD);

        rollerSimState.setRawRotorPosition(
                rollerPhysicsSim.getAngularPositionRad() / IntakeConstants.ROLLER_P_COEFFICIENT);
        rollerSimState.setRotorVelocity(
                rollerPhysicsSim.getAngularVelocityRadPerSec() / IntakeConstants.ROLLER_P_COEFFICIENT);
    }
}
