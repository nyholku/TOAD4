package toad4;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;

import purejavacomm.*;

public class ToadCommSerial implements ToadComm {
	private volatile InputStream m_InputStream;
	private volatile OutputStream m_OutputStream;
	private volatile SerialPort m_CommPort;

	public void closeConnection() {
		if (m_CommPort != null) {
			try {
				m_InputStream.close();
			} catch (Exception e) {
				e.printStackTrace();

			}
			try {
				m_OutputStream.close();
			} catch (Exception e) {
				e.printStackTrace();

			}
			try {
				m_CommPort.close();
			} catch (Exception e) {
				e.printStackTrace();
			}
			m_CommPort = null;
		}

	}

	public void openConnection(String portName) throws DeviceBusyException, DeviceNotFound, IOException {
		if (m_CommPort == null) {
			CommPortIdentifier portId = null;
			try {
				System.out.print("attempt to open port: '" + portName + "'... ");
				portId = CommPortIdentifier.getPortIdentifier(portName);
				m_CommPort = (SerialPort) portId.open("EazyCNC", 10);
				m_CommPort.enableReceiveTimeout(100);
				m_CommPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
				m_InputStream = m_CommPort.getInputStream();
				// Just flush the input empty
				while (m_InputStream.available() > 0)
					m_InputStream.read();
				m_OutputStream = m_CommPort.getOutputStream();
				System.out.print("ok");
			} catch (NoSuchPortException ex) {
				System.out.print("no such port");
				throw new DeviceNotFound();
			} catch (PortInUseException e) {
				System.out.print("port busy");
				throw new DeviceBusyException();
			} catch (UnsupportedCommOperationException e) {
				System.out.println("did not support our settings");
				throw new IOException("UnsupportedCommOperationException on port " + portId.getName());
			} finally {
				System.out.println();
			}

			if (false) {

				Enumeration portList = CommPortIdentifier.getPortIdentifiers();

				while (portList.hasMoreElements()) {
					portId = (CommPortIdentifier) portList.nextElement();
					System.out.println("found port: " + portId.getName() + " " + portName);
					if (portName.endsWith(portId.getName())) {
						System.out.println("MATCH");
						if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
							// Should probably use application name property here,
							// not static string
							try {
								m_CommPort = (SerialPort) portId.open("EazyCNC", 10);
								m_CommPort.enableReceiveTimeout(100);
								m_CommPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
								m_InputStream = m_CommPort.getInputStream();
								// Just flush the input empty
								while (m_InputStream.available() > 0)
									m_InputStream.read();
								m_OutputStream = m_CommPort.getOutputStream();
								System.out.println("PORT OPEN SUCCEFULLY " + portId.getName());
							} catch (PortInUseException e) {
								throw new DeviceBusyException();
							} catch (UnsupportedCommOperationException e) {
								throw new IOException("UnsupportedCommOperationException on port " + portId.getName());
							}
							return;
						}
						throw new IOException("Not serial port " + portId.getName());
					}
				}
				throw new DeviceNotFound();
			}
		}
	}

	public boolean isConnected() {
		return m_CommPort != null;
	}

	public void write(byte[] buffer, int len) throws Exception {
		m_OutputStream.write(buffer, 0, len);
	}

	public int read(byte[] buffer, int len) throws Exception {
		int left;
		if (len == 0) {
			len = m_InputStream.read();
			//System.out.printf("READ#1: %02X\n", len);
			if (len == 0)
				throw new TimeoutException();
			if (len < 0)
				return 0;
			if (len < 0x80)
				return 0;
			buffer[0] = (byte) len;
			len = len & 0x7F;
			left = len - 1;
		} else
			left = len;
		while (left > 0) {
			int n = m_InputStream.read(buffer, len - left, left); // len? left?
			if (n == 0)
				throw new TimeoutException();
			if (n < 0)
				break;
			left -= n;
		}
		return len - left;
	}

	@Override
	public void flush() throws InterruptedException, IOException {
		int n = 0;
		do { // read until input empty
			if (n > 0) {
				for (int i = 0; i < n; ++i)
					m_InputStream.read();
			}
			Thread.sleep(10); // give data chance to arrive
		} while ((n = m_InputStream.available()) > 0);

	}
}
