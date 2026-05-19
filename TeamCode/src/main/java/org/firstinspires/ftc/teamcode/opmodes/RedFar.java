package org.firstinspires.ftc.teamcode.opmodes;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Data;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous
public class RedFar extends NextFTCOpMode {
    public RedFar(){
        addComponents(
                new SubsystemComponent(Hood.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Transfer.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Pose startPose = new Pose(57,8.5  , Math.toRadians(90)).mirror();
    private final Pose clearPose = new Pose(57,11, Math.toRadians(90)).mirror();
    private final Pose preIntakePose = new Pose(36,36,Math.toRadians(180)).mirror();
    private final Pose intakePose = new Pose(9,36, Math.toRadians(180)).mirror();
    private final Pose shootPose = new Pose(57.5,15, Math.toRadians(180)).mirror();
    private final Pose intakeHumanPose = new Pose(9,8, Math.toRadians(180)).mirror();
    private final Pose intake2Pose = new Pose(9,12, Math.toRadians(180)).mirror();
    private final Pose leavePose = new Pose(51,24, Math.toRadians(180)).mirror();



    private PathChain clear,preIntake, intake, shoot1, intake2, shoot2, intake3, shoot3, intake4, shoot4, leave;

    public void buildPaths() {
        clear = PedroComponent.follower().pathBuilder().addPath(new BezierLine(startPose,clearPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), clearPose.getHeading())
                .build();
        preIntake = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(clearPose, new Pose(50,36).mirror(),preIntakePose))
                .setLinearHeadingInterpolation(clearPose.getHeading(), preIntakePose.getHeading())
                .build();
        intake = PedroComponent.follower().pathBuilder().addPath(new BezierLine(preIntakePose,intakePose))
                .setLinearHeadingInterpolation(preIntakePose.getHeading(), intakePose.getHeading())
                .build();
        shoot1 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(intakePose,shootPose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shootPose.getHeading())
                .build();
        intake2 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(shootPose,intakeHumanPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeHumanPose.getHeading())
                .build();
        shoot2 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(intakeHumanPose,shootPose))
                .setLinearHeadingInterpolation(intakeHumanPose.getHeading(), shootPose.getHeading())
                .build();
        intake3 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(shootPose,intake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Pose.getHeading())
                .build();
        shoot3 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(intake2Pose,shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
                .build();
        intake4 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(shootPose,intakeHumanPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeHumanPose.getHeading())
                .build();
        shoot4 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(intakeHumanPose,shootPose))
                .setLinearHeadingInterpolation(intakeHumanPose.getHeading(), shootPose.getHeading())
                .build();
        leave = PedroComponent.follower().pathBuilder().addPath(new BezierLine(shootPose,leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();

    }
    private Command auto(){
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(clear,true),
                        Intake.INSTANCE.runIntake,
                        Shooter.INSTANCE.runFlywheelFar,
                        Turret.INSTANCE.runTurretToPosition(24),
                        Hood.INSTANCE.up
                ),
                Transfer.INSTANCE.fireFar(),
                new FollowPath(preIntake,true),
                new FollowPath(intake,true),
                new ParallelGroup(
                        new FollowPath(shoot1,true),
                        Turret.INSTANCE.runTurretToPosition(-68)
                ),
                new Delay(.25),
                Transfer.INSTANCE.fireFar(),
                new FollowPath(intake2,true),
                new FollowPath(shoot2,true),
                new Delay(.25),
                Transfer.INSTANCE.fireFar(),
                new FollowPath(intake3,true),
                new FollowPath(shoot3,true),
                new Delay(.25),
                Transfer.INSTANCE.fireFar(),
                new FollowPath(intake4,true),
                new FollowPath(shoot4,true),
                new Delay(.25),
                Transfer.INSTANCE.fireFar(),
                new ParallelGroup(
                        new FollowPath(leave,true),
                        Turret.INSTANCE.runTurretToPosition(0)
                )


        );

    }
    @Override
    public void onInit(){
        Turret.INSTANCE.resetEncoder();
        Turret.alignment=false;
        panelsTelemetry.addLine("Initialized");
        panelsTelemetry.update(telemetry);

        PedroComponent.follower().setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void onStartButtonPressed(){
        panelsTelemetry.addLine("Started");
        panelsTelemetry.update(telemetry);
        Turret.INSTANCE.runTurretToPosition(0);
        auto().schedule();
    }
    @Override
    public void onUpdate(){
        panelsTelemetry.addLine(CommandManager.INSTANCE.snapshot().toString());
        if(!PedroComponent.follower().getPose().roughlyEquals(new Pose(0,0,0),5)){
            Data.endPose = PedroComponent.follower().getPose();
        }
        Data.turretPos=Turret.INSTANCE.getTurretAngleDeg();

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop(){
        Intake.INSTANCE.stopIntake.schedule();
        Shooter.INSTANCE.stopFlywheel.schedule();
    }
}
