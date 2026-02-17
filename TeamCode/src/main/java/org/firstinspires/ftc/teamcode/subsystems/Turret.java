package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.Calculations;
import org.firstinspires.ftc.teamcode.opmodes.Teleop;


import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret(){}

    public MotorEx turret = new MotorEx("turret");
    public static boolean alignment = false;

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.0065,0.0,0.000175)
            .basicFF(0,0,0.08)
            .build();

    public Command runTurretToPosition(double position){
        return new RunToPosition(controlSystem,position,20).requires(this);
    }
    public Command resetTurretEncoder(){
        turret.zero();
        return new NullCommand();
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
        //else if(result.isValid()) state = AlignmentMode.Limelight;
        else state = AlignmentMode.Odometry;

        switch(state){
            case Limelight:
                if(result.isValid()){
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        double target = turret.getCurrentPosition() + fr.getTargetXDegrees()*2.872;
                        controlSystem.setGoal(new KineticState(target,Math.signum(target - turret.getCurrentPosition())));

                        //ActiveOpMode.telemetry().addData("tx", tx);
                        //ActiveOpMode.telemetry().addData("pos", current);
                        //ActiveOpMode.telemetry().addData("goal", target);
                        //ActiveOpMode.telemetry().addData("err(goal-pos)", target - current);

                    }
                }
                else controlSystem.setGoal(new KineticState(turret.getCurrentPosition(),Math.signum(turret.getCurrentPosition() - turret.getCurrentPosition())));
                break;
            case Odometry:
                double targetTicks = Calculations.getTurretAngle(Teleop.getFollower().getPose());
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
