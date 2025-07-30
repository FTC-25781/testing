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
import org.firstinspires.ftc.teamcode.auto.OffsetUtils;

@Autonomous(name = "OffsetTurnAuto", group = "Examples")
public class OffsetTurnAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose pose1 = new Pose(8, 111, Math.toRadians(90));
    private final Pose pose2 = new Pose(8, 129, Math.toRadians(90));
    private final Pose pose3 = new Pose(8, 129, Math.toRadians(0));
    private final Pose pose4 = new Pose(72, 129, Math.toRadians(0));
    private final Pose pose5 = new Pose(72, 129, Math.toRadians(270));
    private final Pose pose6 = new Pose(72, 106, Math.toRadians(270));

    private Path path1, path2, path3;

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(pose1);
        buildPaths();
    }

    private void buildPaths() {
        path1 = new Path(new BezierLine(new Point(pose1), new Point(pose2)));
        path1.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());

        path2 = new Path(new BezierLine(new Point(pose3), new Point(pose4)));
        path2.setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading());

        path3 = new Path(new BezierLine(new Point(pose5), new Point(pose6)));
        path3.setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading());
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.turnTo(pose3.getHeading());
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(path2);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.turnTo(pose5.getHeading());
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(path3);
                setPathState(5);
                break;
            case 5:
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
        boolean usingOffset = false;

        if (pathState == 1 || pathState == 3) {
            usedPose = OffsetUtils.applyOffset(rawPose, 4, -2);
            usingOffset = true;
        }

        follower.setPose(usedPose);
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Pathing Mode", usingOffset ? "OFFSET" : "CENTER");
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
