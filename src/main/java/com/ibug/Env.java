package com.ibug;
import javax.vecmath.Color3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import simbad.sim.Box;
import simbad.sim.CherryAgent;
import simbad.sim.EnvironmentDescription;

/**
 *
 * @author The one and only me
 */
public class Env extends EnvironmentDescription {
    // hold y to 0 for the cherry agent
    Vector3d light_pos = new Vector3d(6, 0, 2);

    Env() {
        // set the lights
        light1SetPosition(light_pos.x, 2, light_pos.z);
        light2SetPosition(light_pos.x, 2, light_pos.z);

        // create the obstacles
        // add(new Box(new Vector3d(3,0,2), new Vector3f(1,1,4), this));
        // add(new Box(new Vector3d(1,0,0.5), new Vector3f(4,1,1), this));

        add(new Box(new Vector3d(-2,0,-3), new Vector3f(1,1,4), this));
        add(new Box(new Vector3d(-4,0,-4.5), new Vector3f(4,1,1), this));
        add(new Box(new Vector3d(-4,0,-1.5), new Vector3f(4,1,1), this));

        add(new Box(new Vector3d(0,0,3), new Vector3f(5,1,5), this));
        add(new Box(new Vector3d(0,0,3), new Vector3f(5,1,5), this));
        add(new Box(new Vector3d(2.75,0,3), new Vector3f(0.5f,1,4),this));
        add(new Box(new Vector3d(3.25,0,3), new Vector3f(0.5f,1,3),this));
        add(new Box(new Vector3d(-2.75,0,3), new Vector3f(0.5f,1,4),this));
        add(new Box(new Vector3d(-3.25,0,3), new Vector3f(0.5f,1,3),this));


        add(new CherryAgent(light_pos, "goal", 0.1f));
        add(new MyRobot(new Vector3d(-8, 0, -3), "i-robot"));
    }

}
