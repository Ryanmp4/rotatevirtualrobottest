package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mech Test Code", group = "FRC 4557")
public class MECHTEST extends OpMode {
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
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        mecanumDrive.driveMecanum(forward, strafe, rotate);
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);










        boolean leftrotate = gamepad1.x;
        boolean rightrotate = gamepad1.b;

        if (leftrotate && rightrotate) {
            mecanumDrive.driveMecanum(0, 0, 0);
        } else {
            if (leftrotate) {
                mecanumDrive.driveMecanum(0, 0, -1);
            } else {
                if (rightrotate) {
                    mecanumDrive.driveMecanum(0, 0, 1);
                }
            }
        }



        boolean strafeleft = gamepad1.dpad_left;
        boolean straferight = gamepad1.dpad_right;

        if (strafeleft && straferight) {
            mecanumDrive.driveMecanum(0, 0, 0);
        } else {
            if (strafeleft)
            {
                mecanumDrive.driveMecanum(0,-1,0);
            } else {
                if (straferight) {
                    mecanumDrive.driveMecanum(0, 1, 0);
                }
            }
        }

        boolean dpadforward = gamepad1.dpad_up;
        boolean dpadback = gamepad1.dpad_down;

        if (dpadback && dpadforward) {
            mecanumDrive.driveMecanum(0, 0, 0);
        } else {
            if (dpadforward ) {
                mecanumDrive.driveMecanum(1,0,0);
            } else {
                if (dpadforward) {
                    mecanumDrive.driveMecanum(-1,0,0);
                }
            }
        }








        

    }
}
