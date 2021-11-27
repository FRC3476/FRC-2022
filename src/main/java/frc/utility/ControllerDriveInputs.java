package frc.utility;

public class ControllerDriveInputs {
    
    private double x, y, rotation;

    public ControllerDriveInputs(double x, double y, double rotation){
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getRotation(){
        return rotation;
    }

    /**
     * Apply a circular deadzone on the controller and then scale the joystick values
     * @param snappingDeadZoneX An X value smaller than this value will get set to 0 
     * (This help you with driving straight forwards and backwards) 
     * Larger x values will be scaled so that the edge of the deadzone will be 0
     * @param snappingDeadZoneY An Y value smaller than this value will get set to 0 
     * (This help you with driving straight sideways)
     * Larger y values will be scaled so that the edge of the deadzone will be 0
     * @param rotationDeadZone A rotation value smaller than this value will get set to 0. 
     * Larger rotation values will be scaled so that the edge of the deadzone will be 0
     * @param circularDeadZone If the controller input is inside the circle with a radius of 
     * this value we'll set the X and Y values to 0. If its higher we'll scale the joystick values to make 
     * the edge of the deadzone have a value of 0
     * @return {@link ControllerDriveInputs} 
     */
    public ControllerDriveInputs applyDeadZone(double snappingDeadZoneX, double snappingDeadZoneY, double rotationDeadZone, double circularDeadZone){
		double amplitudeSquared = x*x + y*y;
		if(amplitudeSquared<circularDeadZone*circularDeadZone){
			x = 0;
			y = 0;
		} else{
			double angle = Math.atan2(x, y);
			double minx = Math.sin(angle)*circularDeadZone;
			double miny = Math.cos(angle)*circularDeadZone;

			//System.out.println("min controller: x: " + minx1 + " y: " +  minx2);

			x = Math.copySign(OrangeUtility.coercedNormalize(Math.abs(x), Math.abs(minx), 1, 0, 1), x);
			y = Math.copySign(OrangeUtility.coercedNormalize(Math.abs(y), Math.abs(miny), 1, 0, 1), y);
        }

        //coercedNormalize should do this for us
        // if(Math.abs(x)<snappingDeadZoneX) x = 0;
        // if(Math.abs(y)<snappingDeadZoneY) y = 0;
        // if(Math.abs(rotation)<rotationDeadZone) rotation = 0;

        rotation = Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rotation), Math.abs(rotationDeadZone), 1, 0, 1), rotation);
        x = Math.copySign(OrangeUtility.coercedNormalize(Math.abs(x), Math.abs(snappingDeadZoneX), 1, 0, 1), x);
        y = Math.copySign(OrangeUtility.coercedNormalize(Math.abs(y), Math.abs(snappingDeadZoneY), 1, 0, 1), y);

        return this;
    }

    /**
     * Squares Inputs to apply Acceleration
     * @return {@link ControllerDriveInputs} 
     */
    public ControllerDriveInputs squareInputs(){
        x = Math.copySign(x*x, x);
        y = Math.copySign(y*y, y);
        rotation = Math.copySign(rotation*rotation, rotation);
        return this;
    }

    /**
     * Cubes Inputs to apply Acceleration
     * @return {@link ControllerDriveInputs} 
     */
    public ControllerDriveInputs cubeInputs(){
        x = x*x*x;
        y = y*y*y;
        rotation = rotation*rotation*rotation;
        return this;
    }
}
