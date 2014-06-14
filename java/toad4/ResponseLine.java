package toad4;

/**
 * A wrapper for the controller response
 * 
 * @author nyholku
 * 
 */
public class ResponseLine {
	private byte[] m_Buffer;
	private int m_Length;
	private int m_Cursor;
	private boolean m_OK;

	public String toString() {
		StringBuffer t = new StringBuffer();
		for (int i = 0; i < m_Length; ++i)
			t.append(String.format("%02X ", m_Buffer[i]));
		return t.toString();
	}

	public boolean isOK() {
		return m_OK;
	}

	private void ASSERT(boolean x) {
		if (!x)
			throw new IllegalArgumentException();
	}

	public ResponseLine(byte[] bytes) {
		if (bytes == null) {
			m_OK = false;
			return;
		}
		byte sum = 0x55;
		int len = bytes.length;
		for (int i = 0; i < len - 1; ++i)
			sum += bytes[i];

		m_OK = bytes[len - 1] == sum;
		m_Buffer = bytes;
		m_Length = m_Buffer.length - 1;
		m_Cursor = 0;
	}

	public String parseString() {
		StringBuffer s = new StringBuffer();
		char t;
		while ((t = (char) parseByte()) != 0)
			s.append(t);

		return s.toString();
	}

	public int parseByte() {
		ASSERT(m_Cursor < m_Length);
		return m_Buffer[m_Cursor++] & 0xFF;
	}

	public int parseDoubleByte() {
		ASSERT(m_Cursor < m_Length);
		return (parseByte() << 8) + parseByte();
	}

	public int getLength() {
		return m_Length;
	}
}
