package org.firstinspires.ftc.teamcode.opmodes;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
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
public class Compatible18Blue extends NextFTCOpMode {
    public Compatible18Blue(){
        addComponents(
                new SubsystemComponent(
                        Hood.INSTANCE,
                        Intake.INSTANCE,
                        Shooter.INSTANCE,
                        Transfer.INSTANCE,
                        Turret.INSTANCE,
                        Limelight.INSTANCE,
                        Lift.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Pose startPose = new Pose(28.5,128.5  , Math.toRadians(180));
    private final Pose shootPose = new Pose(51,93, Math.toRadians(180));
    private final Pose intakeMidPose = new Pose(12,58,Math.toRadians(180));
    private final Pose gatePose = new Pose(12,64.5, Math.toRadians(140));
    private final Pose clearGatePose = new Pose(12,60, Math.toRadians(120));
    private final Pose shoot2Pose = new Pose(51,93, Math.toRadians(135));
    private final Pose intakeClosePose = new Pose(15,84, Math.toRadians(180));
    private final Pose leavePose = new Pose(57,108, Math.toRadians(150));



    private PathChain shoot1,intakeMid, shoot2, gate1, clear1, shoot3, gate2, clear2, shoot4, intakeClose,leave;

    public void buildPaths() {
        shoot1 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        intakeMid = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(shootPose, new Pose(51,48),intakeMidPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeMidPose.getHeading())
                .build();
        shoot2 = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(intakeMidPose, new Pose(48,63),shootPose))
                .setLinearHeadingInterpolation(intakeMidPose.getHeading(), shootPose.getHeading())
                .build();
        gate1 = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(shootPose, new Pose(48,63),gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                .build();
        clear1 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(gatePose,clearGatePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), clearGatePose.getHeading())
                .build();
        shoot3 = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(clearGatePose, new Pose(48,63),shoot2Pose))
                .setLinearHeadingInterpolation(clearGatePose.getHeading(), shoot2Pose.getHeading())
                .build();
        gate2 = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(shootPose, new Pose(48,63),gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                .build();
        clear2 = PedroComponent.follower().pathBuilder().addPath(new BezierLine(gatePose,clearGatePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), clearGatePose.getHeading())
                .build();
        shoot4 = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(clearGatePose, new Pose(48,63),shoot2Pose))
                .setLinearHeadingInterpolation(clearGatePose.getHeading(), shoot2Pose.getHeading())
                .build();
        intakeClose = PedroComponent.follower().pathBuilder().addPath(new BezierCurve(shootPose,new Pose(45,78),intakeClosePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeClosePose.getHeading())
                .build();
        leave = PedroComponent.follower().pathBuilder().addPath(new BezierLine(intakeClosePose,leavePose))
                .setLinearHeadingInterpolation(intakeClosePose.getHeading(), leavePose.getHeading())
                .build();

    }
    private Command auto(){
        return new SequentialGroup(
                new ParallelGroup(
                        Intake.INSTANCE.runIntake,
                        Shooter.INSTANCE.runFlywheelClose,
                        Turret.INSTANCE.runTurretToPosition(46),
                        new FollowPath(shoot1,true),
                        Hood.INSTANCE.close

                ),

                Transfer.INSTANCE.runTransfer(1.0),
                new Delay(.5),
                Transfer.INSTANCE.runTransfer(0.0),
                new FollowPath(intakeMid,true),
                new FollowPath(shoot2,true),
                Transfer.INSTANCE.runTransfer(1.0),
                new Delay(.5),
                Transfer.INSTANCE.runTransfer(0.0),
                new FollowPath(gate1,true),
                new Delay(.3),
                new FollowPath(clear1,true),
                new Delay(.9),
                new ParallelGroup(
                        new FollowPath(shoot3,true),
                        Turret.INSTANCE.runTurretToPosition(0)
                ),
                Transfer.INSTANCE.runTransfer(1.0),
                new Delay(.5),
                Transfer.INSTANCE.runTransfer(0.0),
                new FollowPath(gate2,true),
                new Delay(.3),
                new FollowPath(clear2,true),
                new Delay(.9),
                new FollowPath(shoot4,true),
                Transfer.INSTANCE.runTransfer(1.0),
                new Delay(.5),
                Transfer.INSTANCE.runTransfer(0.0),
                new FollowPath(gate2,true),
                new Delay(.3),
                new FollowPath(clear2,true),
                new Delay(.9),
                new FollowPath(shoot4,true),
                Transfer.INSTANCE.runTransfer(1.0),
                new Delay(.5),
                Transfer.INSTANCE.runTransfer(0.0),
                new FollowPath(intakeClose,true),
                new FollowPath(leave,true),
                Transfer.INSTANCE.runTransfer(1.0),
                new Delay(.5),
                Transfer.INSTANCE.runTransfer(0.0)

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

        auto().schedule();
    }
    @Override
    public void onUpdate(){
        panelsTelemetry.addLine(CommandManager.INSTANCE.snapshot().toString());
        if(!PedroComponent.follower().getPose().roughlyEquals(new Pose(0,0,0),5)){
            Data.endPose = PedroComponent.follower().getPose();
        }
        telemetry.addData("endpose", Data.endPose.getX() + ", "+ Data.endPose.getY()+ ", "+ Data.endPose.getHeading());
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void onStop(){
        Intake.INSTANCE.stopIntake.schedule();
        Shooter.INSTANCE.stopFlywheel.schedule();
    }
}
