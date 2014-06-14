package toad4;

/**
 * *
 * 
 * @author nyholku
 * 
 */
final public class CommandFormatter extends Commands {
	private byte[] m_Buffer;
	private int m_Length;
	private int m_RespLength;

	private void ASSERT(boolean x) {
		if (!x)
			throw new IllegalArgumentException();
	}

	public CommandFormatter() {
		m_Buffer = new byte[1024];
		reset();
	}

	public void reset() {
		m_Length = 1; // leave room for message length byte
		m_RespLength = 2; // the message overhead is two bytes, length + checksum
	}

	public byte[] getBuffer() {

		return m_Buffer;
	}

	public void pack() {
		ASSERT(m_Length >= 2);
		byte sum = 0x55;
		int len = m_Length;
		m_Buffer[0] = (byte) (0x80 + len);

		// m_Buffer[1] = (byte) ((command << 3) | motor);
		for (int i = 1; i < len; ++i)
			sum += m_Buffer[i];
		m_Buffer[len] = sum;
		m_Length++;
	}

	public byte[] getBytes() {
		byte[] copy = new byte[m_Length];
		System.arraycopy(m_Buffer, 0, copy, 0, m_Length);
		return copy;
	}

	public int getLength() {
		return m_Length;
	}

	public int getRespLength() {
		return m_RespLength;
	}

	public String toString() {
		StringBuffer t = new StringBuffer();
		for (int i = 0; i < m_Length; ++i)
			t.append(String.format("%02X ", m_Buffer[i]));
		return t.toString();
	}

	private void appendQuadByte(int x) {
		ASSERT(m_Length + 4 <= m_Buffer.length);
		m_Buffer[m_Length++] = (byte) (x >> 24);
		m_Buffer[m_Length++] = (byte) (x >> 16);
		m_Buffer[m_Length++] = (byte) (x >> 8);
		m_Buffer[m_Length++] = (byte) x;
		ASSERT(m_Length <= MAX_LINE_LENGTH);
	}

	private void appendDoubleByte(int x) {
		ASSERT(m_Length + 2 <= m_Buffer.length);
		m_Buffer[m_Length++] = (byte) (x >> 8);
		m_Buffer[m_Length++] = (byte) x;
		ASSERT(m_Length <= MAX_LINE_LENGTH);
	}

	private void appendByte(int x) {
		ASSERT(m_Length + 1 <= m_Buffer.length);
		m_Buffer[m_Length++] = (byte) x;
		ASSERT(m_Length <= MAX_LINE_LENGTH);
	}

	private void appendCommand(int motor, int cmd) {
		ASSERT(m_Length + 1 <= m_Buffer.length);
		ASSERT((motor & ~0x07) == 0);
		ASSERT((cmd & ~0x1F) == 0);
		m_Buffer[m_Length++] = (byte) ((cmd << 3) | motor);
		ASSERT(m_Length <= MAX_LINE_LENGTH);
	}

	// 86 2B 03 E8 07 D0 42
	public void appendMode(int motor, int sync, boolean enable, boolean fulltorq, int dir, int home) {
		ASSERT(sync >= 0 && sync <= 0xF);
		int cmd = SET_MODE;
		// u8 motorNo, u8 enable, u8 torque, u8 syncFlags, u8 dir, u8 home
		appendCommand(motor, cmd);
		appendByte(sync);
		appendByte(enable ? 1 : 0);
		appendByte(fulltorq ? 1 : 0);
		appendByte(dir);
		appendByte(home);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendGetVersion() {
		int cmd = GET_VERSION;
		appendCommand(0, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendGo(int motor) {
		int cmd = ENABLE_QUEUE;
		appendCommand(motor, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendGetState(int motor) {
		int cmd = MOTOR_STATE;
		appendCommand(motor, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendGetQueueState(int motor) {
		int cmd = QUEUE_STATE;
		appendCommand(motor, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendResetQueue(int motor) {
		int cmd = RESET_QUEUE;
		appendCommand(motor, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendGetInput(int input) {
		int cmd = GET_INPUT;
		appendCommand(0, cmd);
		appendByte(input);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendSetOutput(int output, int value) {
		int cmd = SET_OUTPUT;
		appendCommand(0, cmd);
		appendByte(output);
		appendByte(value);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendAbort(int motor) {
		int cmd = RESET_QUEUE;
		appendCommand(motor, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendGetPos(int motor) {
		int cmd = GET_POS;
		appendCommand(motor, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendGetProbePos(int motor) {
		int cmd = GET_PROBE_POS;
		appendCommand(motor, cmd);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendSetPos(int motor, int pos) {
		int cmd = SET_POS;
		appendCommand(motor, cmd);
		appendDoubleByte((pos >> 16) & UMAX16B);
		appendDoubleByte(pos & UMAX16B);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendHome(int motor, int loVelo, int hiVelo, int acceleration, int timeout) {
		ASSERT(loVelo >= 0 && loVelo <= UMAX16B);
		ASSERT(hiVelo >= 0 && hiVelo <= UMAX16B);
		ASSERT(acceleration >= 0 && acceleration <= UMAX16B);
		ASSERT(timeout >= 0 && timeout <= UMAX16B);
		// u8 motorNo,uint16 loSpeed,uint16 hiSpeed,uint16 acceleration,uint16 timeout
		int cmd = SEEK_HOME;
		appendCommand(motor, cmd);
		//appendByte(syncMask);
		appendDoubleByte(loVelo);
		appendDoubleByte(hiVelo);
		appendDoubleByte(acceleration);
		appendQuadByte(timeout);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendMove(int motor, int travel, int velo) {
		ASSERT(velo >= 0 && velo <= UMAX16B);
		ASSERT(travel >= SMIN16B && travel <= SMAX16B); 

		int cmd = MOVE_DISTANCE;

		appendCommand(motor, cmd);
		//appendByte(syncMask);
		appendDoubleByte(travel);
		appendDoubleByte(velo);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendConfigProbe(int input, int value) {
		ASSERT(input >= 0 && input <= 0);
		ASSERT(value >= 0 && value <= 1);

		int cmd = CONFIG_PROBE;

		appendCommand(0, cmd);
		appendByte(input);
		appendByte(value);
		m_RespLength += m_CommandResp[cmd];

	}

	public void appendArmProbe(int motor, int stopVelo, int decel) {
		ASSERT(stopVelo >= 0 && stopVelo <= UMAX16B);
		ASSERT(decel >= 0 && decel <= UMAX16B);

		int cmd = ARM_PROBE;

		appendCommand(motor, cmd);
		appendDoubleByte(stopVelo);
		appendDoubleByte(decel);
		m_RespLength += m_CommandResp[cmd];
	}

	

	public void appendJogControl(int motor, int distance) {
		ASSERT(distance >= 0 && distance <= UMAX16B);
		int cmd = CTRL_JOG;
		appendCommand(motor, cmd);
		appendDoubleByte(distance);
		m_RespLength += m_CommandResp[cmd];
	}

	public void appendJog(int motor, boolean forward, int loVelo, int hiVelo, int acceleration, int porch, int crawl, int timeout) {
		// u8 stepperJog(u8 motorNo, uint16 loSpeed,uint16 hiSpeed,uint16 acceleration,u8 forward,uint16 porch, uint16 crawl);
		ASSERT(porch >= 0 && porch <= UMAX16B);
		ASSERT(crawl >= 0 && crawl <= UMAX16B);
		ASSERT(loVelo >= 0 && loVelo <= UMAX16B);
		ASSERT(hiVelo >= 0 && hiVelo <= UMAX16B);
		ASSERT(acceleration >= 0 && acceleration <= UMAX16B);
		int cmd = forward ? JOG_FORWARD : JOG_REVERSE;
		appendCommand(motor, cmd);
		appendDoubleByte(loVelo);
		appendDoubleByte(hiVelo);
		appendDoubleByte(acceleration);
		appendDoubleByte(porch);
		appendDoubleByte(crawl);
		appendQuadByte(timeout);
		m_RespLength += m_CommandResp[cmd];

	}

}
