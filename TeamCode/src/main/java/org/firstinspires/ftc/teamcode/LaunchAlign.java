package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class LaunchAlign extends OpMode {

    private Follower follower;

    /*
    Create two poses
    The current pose and the pose with bearing==0

    Make a path between the two poses
    Make a command to follow the path
     */
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose = poseFactory.of(0, 0, 0);

    private Path rotateTo(double bearing) {
        return line(startPose, startPose)
                .linear(startPose, poseFactory.of(startPose.x(), startPose.y(), startPose.heading()+bearing));
    }

    private Command autoRoutine(double bearing) {
        return follow(follower, rotateTo(bearing));

    }


    @Override
    public void init() {
        Scheduler.reset();
        Camera.init("Webcam 1", hardwareMap, telemetry, true);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        follower.update();

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        AprilTagClusterDetection blue_scoring = Camera.get("BLUE SCORING");

        if (blue_scoring!=null){
            schedule(autoRoutine(blue_scoring.ftcPose.bearing));
        }


        telemetry.addData("Range", blue_scoring);

        telemetry.update();
    }
}
