// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class stores the int sent back from the Driver Station and uses it to
 * check for rising or falling edges
 */
public class Controller extends Joystick {

	public static class Xbox {
		public static int A = 1;
		public static int B = 2;
		public static int X = 3;
		public static int Y = 4;
		public static int LeftBumper = 5;
		public static int RightBumper = 6;
		public static int Back = 7;
		public static int Start = 8;
		public static int LeftClick = 9;
		public static int RightClick = 10;

		public static int LeftX = 0;
		public static int LeftY = 1;
		public static int LeftTrigger = 2;
		public static int RightTrigger = 3;
		public static int RightX = 4;
		public static int RightY = 5;
	}

	/*
	 * The Driver Station sends back an int(32 bits) for buttons Shifting 1 left
	 * (button - 1) times and ANDing it with the int sent from the Driver
	 * Station will either give you 0 or a number not zero if it is true
	 */
	private int oldButtons;
	private int currentButtons;
	private int axisCount, povCount;
	private double[] oldAxis;
	private double[] currentAxis;
	private int[] currentPOV;

	public Controller(int port) {
		super(port);
		axisCount = DriverStation.getInstance().getStickAxisCount(port);
		povCount = DriverStation.getInstance().getStickPOVCount(port);
		oldAxis = new double[axisCount];
		currentAxis = new double[axisCount];
		currentPOV = new int[povCount];
	}

	/**
	 * Only works if update() is called in each iteration
	 *
	 * @param button
	 *            Joystick button ID
	 * @return Falling edge state of the button
	 */
	public boolean getFallingEdge(int button) {
		boolean oldVal = getButtonState(button, oldButtons);
		boolean currentVal = getButtonState(button, currentButtons);
		if (oldVal == true && currentVal == false) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Only works if update() is called in each iteration
	 *
	 * @param button
	 *            Joystick button ID
	 * @return Rising edge state of the button
	 */
	public boolean getRisingEdge(int button) {
		boolean oldVal = getButtonState(button, oldButtons);
		boolean currentVal = getButtonState(button, currentButtons);
		if (oldVal == false && currentVal == true) {
			return true;
		} else {
			return false;
		}
	}

	public boolean getRisingEdge(int axis, double threshold) {
		if (axis <= axisCount) {
			boolean oldVal = oldAxis[axis] > threshold;
			boolean currentVal = currentAxis[axis] > threshold;
			if (oldVal == false && currentVal == true) {
				return true;
			} else {
				return false;
			}
		}
		return false;
	}

	public boolean getFallingEdge(int axis, double threshold) {
		if (axis <= axisCount) {
			boolean oldVal = oldAxis[axis] > threshold;
			boolean currentVal = currentAxis[axis] > threshold;
			if (oldVal == true && currentVal == false) {
				return true;
			} else {
				return false;
			}
		}
		return false;
	}

	/**
	 * This method needs to be called for each iteration of the teleop loop
	 */
	public void update() {
		oldButtons = currentButtons;
		currentButtons = DriverStation.getInstance().getStickButtons(getPort());
		if (axisCount != DriverStation.getInstance().getStickAxisCount(getPort())) {
			axisCount = DriverStation.getInstance().getStickAxisCount(getPort());
			oldAxis = new double[axisCount];
			currentAxis = new double[axisCount];
		}
		if (povCount != DriverStation.getInstance().getStickPOVCount(getPort())) {
			povCount = DriverStation.getInstance().getStickPOVCount(getPort());
			currentPOV = new int[povCount];
		}
		oldAxis = currentAxis;
		for (int i = 0; i < axisCount; i++) {
			currentAxis[i] = DriverStation.getInstance().getStickAxis(getPort(), i);
		}

		for (int i = 0; i < povCount; i++) {
			currentPOV[i] = DriverStation.getInstance().getStickPOV(getPort(), i);
		}
	}

	@Override
	public boolean getRawButton(int button) {
		return getButtonState(button, currentButtons);
	}

	@Override
	public double getRawAxis(int axis) {
		try {
			if (axis <= axisCount && axis >= 0) {
				return currentAxis[axis];
			}
		}
		catch(ArrayIndexOutOfBoundsException e)
		{
		//	System.out.println("Axis out of bounds " + axis + "/" + axisCount);
		}
		return 0;
	}

	@Override
	public int getPOV(int pov) {
		if (pov < povCount && pov >= 0) {
			return currentPOV[pov];
		}
		return -1;
	}

	public int getAxesAsPOV(int x, int y, boolean xinv, boolean yinv) {
		try {
			if (x <= axisCount && x >= 0 && y <= axisCount && y >= 0) {
				int xval = (int)Math.round(currentAxis[x]) * (xinv ? -1 : 1),
					yval = (int)Math.round(currentAxis[y]) * (yinv ? -1 : 1);
				if(xval == 0 && yval == 0) return -1;
				int val = (int)(Math.atan2(yval, xval) * 180.0 / Math.PI);
				if(val < 0) val += 360;
				return val;
			}
		}
		catch(ArrayIndexOutOfBoundsException e)
		{
			System.out.println("Axes out of bounds " + x + " " + y + "/" + axisCount);
		}
		return -1;
	}

	public boolean getButtonState(int button, int state) {
		return ((0x1 << (button - 1)) & state) != 0;
	}
}
