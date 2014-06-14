package toad4;

import java.io.IOException;

public class ResponseParser {
	public static class ChecksumErrorException extends Exception {
	};

	public static class BadDataException extends Exception {
		public BadDataException(String message) {
			super(message);
		}
	};

	private byte[] m_Buffer = new byte[128];
	private int m_Length;
	private int m_Cursor;

	public String toString() {
		StringBuffer t = new StringBuffer();
		for (int i = 0; i <= m_Length; ++i)
			t.append(String.format("%02X ", m_Buffer[i]));
		return t.toString();
	}

	private void ASSERT(boolean x) {
		if (!x)
			throw new IllegalArgumentException();
	}

	public byte[] getBuffer() {
		return m_Buffer;
	}

	public void reset(int len) throws ChecksumErrorException, BadDataException {

		if (len < 1)
			throw new BadDataException("No data");
		byte sum = 0x55;
		for (int i = 1; i < len - 1; ++i)
			sum += m_Buffer[i];

		//System.out.printf("%02X=%02X:  ",sum,bytes[len - 1] );
		m_Length = len - 1;
		m_Cursor = 1;
		if (m_Buffer[len - 1] != sum)
			throw new ChecksumErrorException();

	}

	public String parseString() throws Exception {
		StringBuffer s = new StringBuffer();
		char t;
		while ((t = (char) parseByte()) != 0)
			s.append(t);

		return s.toString();
	}

	public int parseByte() throws BadDataException {
		if (m_Cursor >= m_Length)
			throw new BadDataException("Expected more data from TOAD");
		return m_Buffer[m_Cursor++] & 0xFF;
	}

	public void parseErrCode() throws BadDataException {
		int err = parseByte();
		if (err != 0)
			throw new BadDataException("TOAD returned error code " + err);
	}

	public int parseDoubleByte() throws BadDataException {
		return (parseByte() << 8) + parseByte();
	}

	public int parseQuadByte() throws BadDataException {
		return (parseDoubleByte() << 16) + parseDoubleByte();
	}

	public int getLength() {
		return m_Length;
	}
}
