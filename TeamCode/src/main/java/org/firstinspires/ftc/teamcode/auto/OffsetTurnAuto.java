package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "OffsetTurnAuto", group = "Examples")
public class OffsetTurnAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose pose1 = new Pose(8, 111, Math.toRadians(0));
    private final Pose pose2 = new Pose(8, 129, Math.toRadians(-90));
    private final Pose pose3 = new Pose(72, 129, Math.toRadians(-180));
    private final Pose pose4 = new Pose(72, 106, Math.toRadians(-180));

    private Path toMid, toCross, toEnd;

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(pose1);
        buildPaths();
    }

    private void buildPaths() {
        toMid = new Path(new BezierLine(new Point(pose1), new Point(pose2)));
        toMid.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());

        toCross = new Path(new BezierLine(new Point(pose2), new Point(pose3)));
        toCross.setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading());

        toEnd = new Path(new BezierLine(new Point(pose3), new Point(pose4)));
        toEnd.setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading());
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(toMid);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(toCross);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(toEnd);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        Pose rawPose = follower.getPose();
        Pose usedPose = rawPose;

        switch (pathState) {
            case 0:
                if (OffsetUtils.isTurning(pose1.getHeading(), pose2.getHeading()))
                    usedPose = OffsetUtils.applyOffset(rawPose, 4, -2);
                break;
            case 1:
                if (OffsetUtils.isTurning(pose2.getHeading(), pose3.getHeading()))
                    usedPose = OffsetUtils.applyOffset(rawPose, 4, -2);
                break;
            case 2:
                if (OffsetUtils.isTurning(pose3.getHeading(), pose4.getHeading()))
                    usedPose = OffsetUtils.applyOffset(rawPose, 4, -2);
                break;
        }

        follower.setPose(usedPose);
        follower.update();  // Note: update takes no arguments

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", usedPose.getX());
        telemetry.addData("Y", usedPose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(usedPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}
}
