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

        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("rotatedegree", mecanumDrive.getHeading(AngleUnit.DEGREES));

    boolean leftrotate = gamepad1.x;
    boolean rightrotate = gamepad1.b;
    double rotatedegree = mecanumDrive.getHeading(AngleUnit.DEGREES);

    if (leftrotate && rightrotate) {
            rotate = 0;
        } else {
            if (leftrotate) {

            } else {
                if (rightrotate) {

                }
            }
    }

    mecanumDrive.driveMecanum(forward, strafe, rotate);

    }
}
