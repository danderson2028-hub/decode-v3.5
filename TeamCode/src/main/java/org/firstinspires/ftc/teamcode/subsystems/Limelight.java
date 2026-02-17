package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight(){}
    public Limelight3A limelight;

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(30);

        limelight.start();
    }
    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }
}
