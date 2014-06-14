package toad4;

public class Commands {
	final public static int MAX_LINE_LENGTH = 32;

	//final public static int MOVE_REVERSE = 0;
	final public static int MOVE_DISTANCE = 1;
	final public static int JOG_REVERSE = 2;
	final public static int JOG_FORWARD = 3;
	final public static int RESET_QUEUE = 8;
	final public static int ENABLE_QUEUE = 9;
	final public static int QUEUE_STATE = 10;
	final public static int MOTOR_STATE = 11;
	final public static int SET_MODE = 12;
	final public static int SEEK_HOME = 13;
	final public static int CTRL_JOG = 14;
	final public static int SET_POS = 15;
	final public static int GET_POS = 16;
	final public static int GET_VERSION = 17;
	final public static int ARM_PROBE = 18;
	final public static int GET_PROBE_POS = 19;
	final public static int CONFIG_PROBE = 20;
	final public static int SET_OUTPUT = 21;
	final public static int GET_INPUT = 22;

	final public static int STATE_STOP = 0;
	final public static int STATE_TIMEOUT = 1;
	final public static int STATE_PROCESS_QUEUE = 2;
	final public static int STATE_RUN = 3;
	final public static int STATE_PROBE_TRIGGERED = 17;

	// these are 'absolute' values, used mainly in asserting parameters to TOAD 
	final public static int MAX8B = 0xFF;
	final public static int UMAX16B = 65535;
	final public static int SMAX16B = +32787;
	final public static int SMIN16B = -32768;

	// these may in the future depend on the motor controller, and if so should be moved somewhere else
	final public static int MAX_TRAVEL = SMAX16B;
	final public static int MAX_VELO = UMAX16B;

	final public static int[] m_CommandResp;
	static {
		m_CommandResp = new int[32];
		m_CommandResp[MOVE_DISTANCE] = 1;
		m_CommandResp[JOG_REVERSE] = 1;
		m_CommandResp[JOG_FORWARD] = 1;
		m_CommandResp[RESET_QUEUE] = 1;
		m_CommandResp[ENABLE_QUEUE] = 1;
		m_CommandResp[QUEUE_STATE] = 4;
		m_CommandResp[MOTOR_STATE] = 2;
		m_CommandResp[SET_MODE] = 1;
		m_CommandResp[SEEK_HOME] = 1; //OK
		m_CommandResp[CTRL_JOG] = 1;
		m_CommandResp[SET_POS] = 1;
		m_CommandResp[GET_POS] = 5;
		m_CommandResp[GET_VERSION] = 17;
		m_CommandResp[ARM_PROBE] = 1;
		m_CommandResp[GET_PROBE_POS] = 5;
		m_CommandResp[CONFIG_PROBE] = 1;
		m_CommandResp[SET_OUTPUT] = 1;
		m_CommandResp[GET_INPUT] = 2;

	}

}
