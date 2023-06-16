package com.ibug;

import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import simbad.sim.*;
import java.util.LinkedList;

/**
 *
 * @author The one and only me
 */
public class MyRobot extends Agent {
    LightSensor left, right, center;
    RangeSensorBelt bumpers;
    static double ORI_EPS = 1e-3;
    static double FORWARD = 0.5;
    static double GOAL_LUM = 0.794; // min: 00788212 max: 0.799569

    static double SAFETY = 0.45;
    static double K1 = 5.4;
    static double K2 = 0.15;
    static double K3 = 5;

    double prev_center_lum = 0;
    double following_initial_lum = 0;
    LinkedList<Double> lum_list = new LinkedList<Double>();
    static int LL_MAX_SIZE = 60;
    int steps_without_hit = 0;

    private enum RobotStatus {
        ORIENTATION,
        FORWARD,
        FOLLOWING,
        END;
    }
    RobotStatus status = RobotStatus.ORIENTATION;

    public MyRobot (Vector3d position, String name) {
        super(position,name);

        // light sensors
        left = RobotFactory.addLightSensor(this, new Vector3d(0.6,0.47,-0.6), 0, "left");
        right = RobotFactory.addLightSensor(this, new Vector3d(0.6,0.47,0.6), 0, "right");
        center = RobotFactory.addLightSensor(this, new Vector3d(0,0.47,0), 0, "center");

        // bumper sensors
        bumpers = RobotFactory.addBumperBeltSensor(this, 24);
    }

    public void initBehavior() {
        setTranslationalVelocity(0);
        setRotationalVelocity(0);

        // reset values
        prev_center_lum = 0;
        status = RobotStatus.ORIENTATION;
    }

    public void performBehavior() {
        double[] light_data = readLightSensors();
        double c = light_data[2];

        // System.out.println(status);
        switch(status) {
            case ORIENTATION:
                orientation();
                break;
            case FORWARD:
                forward();
                following_initial_lum = c;
                break;
            case FOLLOWING:
                following();
                break;
            case END:
                setRotationalVelocity(0);
                setTranslationalVelocity(0);

                // print the results
                Point3d cp = new Point3d();
                getCoords(cp);
                System.out.println("Reached GOAL!");
                System.out.printf("\tFinal Position: %f, %f\n", cp.x , cp.z);
                System.out.printf("\tDistance from target: %f\n", cp.distance(new Point3d(6, cp.y, 2)));
                break;
            default:
                System.out.println("SOULD NOT BE HERE!!!");
        }

        // update the previous center luminocity value
        prev_center_lum = c;
    }

    // move around the obstacle
    private void following() {
        // update the number of steps without hits
        if (bumpers.oneHasHit()) {
            steps_without_hit = 0;
        }
        else {
            steps_without_hit++;
        }

        int min_idx = 0;
        for (int i = 1; i < bumpers.getNumSensors(); i++) {
            if (bumpers.getMeasurement(i) < bumpers.getMeasurement(min_idx)) {
                min_idx = i;
            }
        }

        Point3d hit_point = getHitPoint(min_idx);
        double hit_distance = hit_point.distance(new Point3d(0, 0, 0));
        Vector3d hit_vec = new Vector3d(hit_point.z, 0, -hit_point.x);

        double ph_lin = Math.atan2(hit_vec.z, hit_vec.x);
        double ph_rot = Math.atan(K3 * (hit_distance - SAFETY));
        double ph_ref = ph_lin + ph_rot;

        if (ph_ref > Math.PI) {
            ph_ref = Math.PI;
        }
        if (ph_ref < -Math.PI) {
            ph_ref = -Math.PI;
        }

        setRotationalVelocity(K1 * ph_ref);
        setTranslationalVelocity(K2 * Math.cos(ph_ref));

        // add last luminocity to the list
        lum_list.add(readLightSensors()[2]);
        if (lum_list.size() > LL_MAX_SIZE) {
            lum_list.removeFirst();
        }
        if (lum_list.size() == LL_MAX_SIZE) {
            double sum1 = 0;
            double sum2 = 0;
            double sum3 = 0;
            for (int i = 0; i < lum_list.size(); i++) {
                if (i < LL_MAX_SIZE/3) sum1 += lum_list.get(i);
                else if (i < 2*LL_MAX_SIZE/3) sum2 += lum_list.get(i);
                else sum3 += lum_list.get(i);
            }
            sum1 /= LL_MAX_SIZE/3;
            sum2 /= LL_MAX_SIZE/3;
            sum3 /= LL_MAX_SIZE/3;

            if (sum3 < sum2 && sum2 > sum1 && sum3 > following_initial_lum) {
                lum_list.clear();
                status = RobotStatus.ORIENTATION;
            }
        }

        if (steps_without_hit > 200) {
            steps_without_hit = 0;
            lum_list.clear();
            status = RobotStatus.ORIENTATION;
        }
    }

    // calculate the hit point of the given bumper
    private Point3d getHitPoint(int bumper_idx) {
        double dist = getRadius() + bumpers.getMeasurement(bumper_idx);
        double x = dist * Math.cos(bumpers.getSensorAngle(bumper_idx));
        double z = dist * Math.sin(bumpers.getSensorAngle(bumper_idx));
        return new Point3d(x, 0, z);
    }

    // move forward
    private void forward() {
        setRotationalVelocity(0);

        double[] light_data = readLightSensors();
        double c = light_data[2];

        setTranslationalVelocity(FORWARD / c);

        if (bump()) {
            // bumped to an obstacle
            status = RobotStatus.FOLLOWING;
        }
        if (c >= GOAL_LUM) {
            // we reached the goal
            status = RobotStatus.END;
        }
        if (c < prev_center_lum) {
            // local maximum
            status = RobotStatus.ORIENTATION;
        }
    }

    // check if there is an obstacle in front (not back) of the robot
    private boolean bump() {
        int none_back_hits = bumpers.getFrontQuadrantHits() +
                             bumpers.getLeftQuadrantHits() +
                             bumpers.getRightQuadrantHits();
        if (none_back_hits > 0) return true;
        return false;
    }

    // Rotates the robot in order to align with the light source
    private void orientation() {
        // reset movements
        setTranslationalVelocity(0);

        double[] light_data = readLightSensors();
        boolean light_is_in_front = lightInFront(light_data);
        double l = light_data[0];
        double r = light_data[1];

        // System.out.printf("Diff: %f\n", Math.abs(l - r));
        if (!light_is_in_front || Math.abs(l - r) > ORI_EPS) {
            setRotationalVelocity(Math.signum(l - r) * 0.5);
        } else {
            setRotationalVelocity(0);
            status = RobotStatus.FORWARD;
        }
    }

    // true is the light source is in front of the robot
    private boolean lightInFront(double l, double r, double c) {
        if (l > c || r > c) return true;
        if (((l + r) / 2) > c) return true;
        return false;
    }

    // true is the light source is in front of the robot
    private boolean lightInFront(double[] data) {
        return lightInFront(data[0], data[1], data[2]);
    }

    // returns the values of the light sensors
    private double[] readLightSensors() {
        double[] out = new double[3];
        out[0] = Math.pow(left.getLux(), 0.1);
        out[1] = Math.pow(right.getLux(), 0.1);
        out[2] = Math.pow(center.getLux(), 0.1);
        return out;
    }
}
