package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.Calculations;
import org.firstinspires.ftc.teamcode.opmodes.Teleop;
import org.firstinspires.ftc.teamcode.utils.Data;


import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret(){}

    public MotorEx turret = new MotorEx("turret");
    public static boolean alignment = false;
    public static double sotmAngularOffset = 0.0;
    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005,0.0,0.000175)
            .basicFF(0,0,0.05)
            .build();

    public Command runTurretToPosition(double position){
        return new RunToPosition(controlSystem,position,30).requires(this);
    }

    
    private enum AlignmentMode{
        Limelight,
        Odometry,
        OFF
    }
    public static AlignmentMode state = AlignmentMode.Odometry;
    @Override
    public void initialize(){
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        state=AlignmentMode.OFF;

    }

    @Override
    public void periodic(){

        LLResult result = Limelight.INSTANCE.getLatestResult();
        if(!alignment) state = AlignmentMode.OFF;
        else if(result.isValid()&&Math.abs(result.getFiducialResults().get(0).getTargetXDegrees())<10) state = AlignmentMode.Limelight;
        else state = AlignmentMode.Odometry;
        switch(state){
            case Limelight:
                if(result.isValid()){
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        double adjustedTx = fr.getTargetXDegrees(); // <-- apply offset
                        double target = turret.getCurrentPosition() + adjustedTx * 2.872;
                        if(Math.abs(adjustedTx)>2){
                            controlSystem.setGoal(new KineticState(target, Math.signum(target - turret.getCurrentPosition())));
                        }
                    }
                }
                else controlSystem.setGoal(new KineticState(turret.getCurrentPosition(),Math.signum(turret.getCurrentPosition() - turret.getCurrentPosition())));
                break;
            case Odometry:
                Pose currentPose = Teleop.getFollower().getPose();
                double targetTicks = Calculations.getTurretAngle(currentPose);
                controlSystem.setGoal(new KineticState(targetTicks, Math.signum(targetTicks - turret.getCurrentPosition())));
                break;
            case OFF:
                break;
        }





        ActiveOpMode.telemetry().addData("Align mode", state);
        ActiveOpMode.telemetry().addData("Auto Align", alignment);
        ActiveOpMode.telemetry().addData("Turret Pos", turret.getCurrentPosition());
        turret.setPower(controlSystem.calculate(turret.getState()));
    }
}
