package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import javafx.scene.transform.Rotate;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.ftc16072.Util.Polar;
import org.firstinspires.ftc.teamcode.ftc16072.Util.RobotPosition;

@TeleOp(name = "Mech Test Code 2", group = "FRC 4557")
public class MECHTEST2 extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
    }


    BNO055IMU.Parameters params = new BNO055IMU.Parameters();

    double startHeading = 0;
    boolean previouspress = false;
    boolean previouspressy = false;

    // Always return the right side of the angle (i.e. always positive)
    private double GetTurnedAngle(double currentHeading, double startHeading)
    {
        double LeftAngle = 0;
        if (currentHeading < startHeading) {
            LeftAngle = 360 - (startHeading - currentHeading);
        } else {
            LeftAngle = currentHeading - startHeading;
        }
        return LeftAngle;
    }



    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = 0;
        double strafe = 0;
        double rotate = 0;
        if (gamepad1.left_stick_y * -1 > 0.2) {
            forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        }
        if (gamepad1.left_stick_y * -1 < -0.2) {
            forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        }
        if (gamepad1.left_stick_x > 0.2) {
            strafe = gamepad1.left_stick_x;
        }
        if (gamepad1.left_stick_x < -0.2) {
            strafe = gamepad1.left_stick_x;
        }
        if (gamepad1.right_stick_x > 0.2) {
            rotate = gamepad1.right_stick_x;
        }
        if (gamepad1.right_stick_x < -0.2) {
            rotate = gamepad1.right_stick_x;
        }

        double currentHeading = mecanumDrive.getHeading(AngleUnit.DEGREES);

        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("rotatedegree", currentHeading);


        boolean leftrotate = gamepad1.x;
        boolean rightrotate = gamepad1.b;
        double rotatedegree = mecanumDrive.getHeading(AngleUnit.DEGREES);
        double turndegree = GetTurnedAngle(currentHeading, startHeading);

        if (previouspress == false && leftrotate) {
            startHeading = currentHeading;
        }

        if (previouspressy == false && rightrotate) {
            startHeading = currentHeading;
        }

        if (leftrotate && rightrotate) {
            rotate = 0;
        } else {
            if (leftrotate && turndegree <= 90) {
                rotate = -0.5;
            } else {

                // if we have not start the rotation, turn degree will  be 0. Need to start the turn
                double rightrotatedegree = (turndegree==0 ? 0 :  360 - turndegree );

                if (rightrotate &&  rightrotatedegree <= 90 ) {
                    rotate = 0.5;
                }
            }
        }

        mecanumDrive.driveMecanum(forward, strafe, rotate);
        previouspress = gamepad1.x;
        previouspressy = gamepad1.b;


        telemetry.addData("leftrotate", leftrotate);
        telemetry.addData("rightrotate", rightrotate);
        telemetry.addData("turndegree", GetTurnedAngle(currentHeading, startHeading));
        telemetry.addData("startheading", startHeading);
        telemetry.addData("currentheading", currentHeading);

    }


}
