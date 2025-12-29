package frc.robot.subsystems.diverter;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.Constants.DiverterConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class DiverterIOTalonFX implements DiverterIO {
    protected final PearadoxTalonFX pivotMotor;
    protected final PearadoxTalonFX rollerMotor;

    protected final MotionMagicVoltage pivotMMRequest = new MotionMagicVoltage(0);

    protected DiverterIOTalonFX() {
        pivotMotor = new PearadoxTalonFX(DiverterConstants.PIVOT_ID, DiverterConstants.getPivotConfig());
        rollerMotor = new PearadoxTalonFX(DiverterConstants.ROLLER_ID, DiverterConstants.getRollerConfig());
    }

    @Override
    public void updateInputs(DiverterIOInputs inputs) {
        inputs.pivotData = pivotMotor.getData();
        inputs.rollerData = rollerMotor.getData();
    }

    @Override
    public void runPivotPosition(double setpointRots) {
        pivotMotor.setControl(pivotMMRequest.withPosition(setpointRots));
    }

    @Override
    public void runRollerVolts(double volts) {
        rollerMotor.setVoltage(volts);
    }
}
