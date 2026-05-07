package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.Calculations;
import org.firstinspires.ftc.teamcode.utils.Data;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp (name = "Teleop Red")
public class TeleopRed extends NextFTCOpMode {
    public TeleopRed(){
        addComponents(
                new SubsystemComponent(
                        Shooter.INSTANCE,
                        Intake.INSTANCE,
                        Transfer.INSTANCE,
                        Turret.INSTANCE,
                        Hood.INSTANCE,
                        Limelight.INSTANCE,
                        Lift.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }

    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private boolean runShooter = false;
    public double transferPower = 0.5;
    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().rightStickY().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            Gamepads.gamepad1().leftStickX().map(value -> 0.5*value).negate()
    );
    public static Follower getFollower(){
        return PedroComponent.follower();
    }

    private Command park(Pose botPose){
        PathChain move = PedroComponent.follower().pathBuilder().addPath(new BezierLine(botPose, new Pose(botPose.getX()-2,botPose.getY()+29)))
                .setLinearHeadingInterpolation(botPose.getHeading(), botPose.getHeading()+Math.toRadians(45))
                .build();
        return new FollowPath(move);
    }
    private Command moveUp(){
        PathChain move = PedroComponent.follower().pathBuilder().addPath(new BezierLine(PedroComponent.follower().getPose(), new Pose(PedroComponent.follower().getPose().getX(),PedroComponent.follower().getPose().getY()+.75)))
                .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), PedroComponent.follower().getPose().getHeading())
                .build();
        return new FollowPath(move);
    }
    private Command moveDown(){
        PathChain move = PedroComponent.follower().pathBuilder().addPath(new BezierLine(PedroComponent.follower().getPose(), new Pose(PedroComponent.follower().getPose().getX(),PedroComponent.follower().getPose().getY()-.75)))
                .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), PedroComponent.follower().getPose().getHeading())
                .build();
        return new FollowPath(move);
    }
    private Command moveLeft(){
        PathChain move = PedroComponent.follower().pathBuilder().addPath(new BezierLine(PedroComponent.follower().getPose(), new Pose(PedroComponent.follower().getPose().getX()-.75,PedroComponent.follower().getPose().getY())))
                .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), PedroComponent.follower().getPose().getHeading())
                .build();
        return new FollowPath(move);
    }
    private Command moveRight(){
        PathChain move = PedroComponent.follower().pathBuilder().addPath(new BezierLine(PedroComponent.follower().getPose(), new Pose(PedroComponent.follower().getPose().getX()+.75,PedroComponent.follower().getPose().getY())))
                .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), PedroComponent.follower().getPose().getHeading())
                .build();
        return new FollowPath(move);
    }

    @Override
    public void onStartButtonPressed() {
        Limelight.INSTANCE.limelight.pipelineSwitch(1);


        Turret.alignment=true;
        driverControlled.schedule();
        Lift.INSTANCE.holdPlate().schedule();
        Gamepads.gamepad1().circle().whenBecomesTrue(Intake.INSTANCE.runIntake);
        Gamepads.gamepad1().square().whenBecomesTrue(Intake.INSTANCE.runIntakeReverse);
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(Intake.INSTANCE.stopIntake);
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Transfer.INSTANCE.tapFire());
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(1)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));
        Gamepads.gamepad2().circle().whenBecomesTrue(Lift.INSTANCE.lift()).whenBecomesFalse(Lift.INSTANCE.holdLift());
        Gamepads.gamepad2().triangle().whenBecomesTrue(Lift.INSTANCE.retract()).whenBecomesFalse(Lift.INSTANCE.holdLift());
    }
    @Override
    public void onInit(){
        CommandManager.INSTANCE.cancelAll();
        Limelight.INSTANCE.limelight.pipelineSwitch(1);
        frontLeftMotor = new MotorEx("FL").reversed();
        frontRightMotor = new MotorEx("FR");
        backLeftMotor = new MotorEx("BL").reversed();
        backRightMotor = new MotorEx("BR");

        Calculations.goalPose=Calculations.redGoalPose;
        if (Data.endPose != null) {
            PedroComponent.follower().setStartingPose(Data.endPose);
            telemetry.addData("X", Data.endPose.getX());
            telemetry.addData("Y", Data.endPose.getY());
            telemetry.addData("heading", Data.endPose.getHeading());
        } else {
            PedroComponent.follower().setStartingPose(new Pose(72,72,Math.toRadians(90)));
        }

        telemetry.update();

    }
    @Override
    public void onUpdate(){
        LLResult result = Limelight.INSTANCE.getLatestResult();

        if(gamepad1.dpad_up) {
            runShooter = true;
            Turret.alignment = true;
        }
        if(gamepad1.dpad_down) {
            runShooter = false;
            Turret.alignment = false;
        }
        if(gamepad1.right_trigger_pressed) driverControlled.setScalar(0.25);
        else driverControlled.setScalar(1.0);
        if(gamepad2.touchpadWasPressed()){
            CommandManager.INSTANCE.cancelAll();
            driverControlled.schedule();
        }
        if(gamepad2.dpadUpWasPressed()) moveUp().schedule();
        if(gamepad2.dpadDownWasPressed()) moveDown().schedule();
        if(gamepad2.dpadLeftWasPressed()) moveLeft().schedule();
        if(gamepad2.dpadRightWasPressed()) moveRight().schedule();
        if(gamepad2.squareWasPressed()) park(PedroComponent.follower().getPose()).schedule();
        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                double ty = fr.getTargetYDegrees();
                double distance = 16 / (Math.tan(Math.toRadians(7.6026 + ty)));

                ActiveOpMode.telemetry().addData("ty", ty);
                ActiveOpMode.telemetry().addData("dist", distance);
                ActiveOpMode.telemetry().addData("target rpm", Calculations.getShooterRPM(distance));
                ActiveOpMode.telemetry().addData("hood offset", .63-Calculations.getHoodAngle(distance));
                if(runShooter){
                    Shooter.INSTANCE.calculateFlywheel(distance).schedule();
                    Hood.INSTANCE.calculateAngle(distance).schedule();
                }


                //transferPower = Calculations.getTransferSpeed(distance);

                //ActiveOpMode.telemetry().addData("transfer power", transferPower);


            }
        }
        /*
        else{
            if(runShooter){
                Shooter.INSTANCE.calculateFlywheel(Calculations.getDist(PedroComponent.follower().getPose())).schedule();
                Hood.INSTANCE.calculateAngle(Calculations.getDist(PedroComponent.follower().getPose())).schedule();
            }
        }

         */
        if(!runShooter){
            Shooter.INSTANCE.stopFlywheel.schedule();
        }
        //telemetry.addData("distance", Math.hypot(Calculations.redGoalPose.getY()-PedroComponent.follower().getPose().getY(),Calculations.redGoalPose.getX()-PedroComponent.follower().getPose().getX()));
        telemetry.addData("x y heading", PedroComponent.follower().getPose().getX() + ", "+ PedroComponent.follower().getPose().getY()+ ", "+ PedroComponent.follower().getPose().getHeading());
        telemetry.addData("Commands", CommandManager.INSTANCE.snapshot());
        telemetry.update();
    }
    @Override
    public void onStop(){
        Turret.alignment=false;
        CommandManager.INSTANCE.cancelAll();
        Shooter.INSTANCE.stopFlywheel.schedule();
        //Poses.endPose=PedroComponent.follower().getPose();
    }
}
