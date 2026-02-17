package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Calculations;
import org.firstinspires.ftc.teamcode.commands.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPositions;

@TeleOp
public class Teleop extends NextFTCOpMode {
    public Teleop(){
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Turret.INSTANCE, Hood.INSTANCE, Limelight.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }
    //public static Pose endPose;
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;



    public static Follower getFollower(){
        return PedroComponent.follower();
    }
    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().rightStickY().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                Gamepads.gamepad1().leftStickX().map(value -> 0.5*value).negate()
        );
        driverControlled.schedule();
        Gamepads.gamepad1().circle().whenBecomesTrue(Intake.INSTANCE.runIntake);
        Gamepads.gamepad1().square().whenBecomesTrue(Intake.INSTANCE.runIntakeReverse);
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(Intake.INSTANCE.stopIntake);
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(0.4)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(0.7)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(Shooter.INSTANCE.runFlywheel.and(Hood.INSTANCE.close));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(Shooter.INSTANCE.runFlywheelFar.and(Hood.INSTANCE.up));
    }
    @Override
    public void onInit(){
        CommandManager.INSTANCE.cancelAll();

        frontLeftMotor = new MotorEx("FL").reversed();
        frontRightMotor = new MotorEx("FR");
        backLeftMotor = new MotorEx("BL").reversed();
        backRightMotor = new MotorEx("BR");
        if(Poses.endPose!=null) PedroComponent.follower().setStartingPose(Poses.endPose);
        else PedroComponent.follower().setStartingPose(new Pose(51,24, Math.toRadians(135)));
        Turret.alignment=true;
        Calculations.goalPose=Calculations.blueGoalPose;
        //Limelight.INSTANCE.limelight.pipelineSwitch(1);
    }
    @Override
    public void onUpdate(){
        LLResult result = Limelight.INSTANCE.getLatestResult();
        //telemetry.addData("distance", Math.hypot(Calculations.redGoalPose.getY()-PedroComponent.follower().getPose().getY(),Calculations.redGoalPose.getX()-PedroComponent.follower().getPose().getX()));
        //telemetry.addData("x y heading", PedroComponent.follower().getPose().getX() + ", "+ PedroComponent.follower().getPose().getY()+ ", "+ PedroComponent.follower().getPose().getHeading());
        telemetry.addData("Commands", CommandManager.INSTANCE.snapshot());
        telemetry.update();
    }
    @Override
    public void onStop(){

        CommandManager.INSTANCE.cancelAll();
        Shooter.INSTANCE.stopFlywheel.schedule();
    }
}
