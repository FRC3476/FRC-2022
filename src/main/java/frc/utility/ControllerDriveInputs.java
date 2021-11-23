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
     * @param snapingDeadZoneX An X value smaller than this value will get set to 0 
     * (This help you with driving stright forwards and backwards) 
     * @param snapingDeadZoneY An Y value smaller than this value will get set to 0 
     * (This help you with driving stright sideways)
     * @param rotationDeadZone A rotation value smaller than this value will get set to 0. 
     * Larger rotation values will be scaled so that the edge of the deadzone will be 0
     * @param circularDeadZone If the controller input is inside the circle with a radius of 
     * this value we'll set the X and Y values to 0. If its higher we'll scale the joystick values to make 
     * the edge of the deadzone have a value of 0
     * @return {@link ControllerDriveInputs} 
     */
    public ControllerDriveInputs applyDeadZone(double snapingDeadZoneX, double snapingDeadZoneY, double rotationDeadZone, double circularDeadZone){
        if(Math.abs(x)<snapingDeadZoneX) x = 0;
        if(Math.abs(y)<snapingDeadZoneY) y = 0;
        if(Math.abs(rotation)<rotationDeadZone) rotation = 0;

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
		rotation = Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rotation), Math.abs(rotationDeadZone), 1, 0, 1), rotation);

        return this;
    }

    /**
     * Squares Inputs to apply Acceleration
     * @return {@link ControllerDriveInputs} 
     */
    public ControllerDriveInputs squareInputs(){
        x = Math.copySign(x*x, y);
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
