#pragma once

#include <string>

namespace output {

// Minimal fire-and-forget UDP datagram sender (POSIX sockets, no addon).
// Used to publish one motion-path.v1 JSON object per analysed frame to the
// renderer / control software. Datagrams larger than ~60 KB are dropped
// with a warning (the path arrays are already capped by MotionPathWriter).
class UdpSender {
public:
	UdpSender() = default;
	~UdpSender();
	UdpSender(const UdpSender &) = delete;
	UdpSender & operator=(const UdpSender &) = delete;

	bool open(const std::string & host, int port);
	void close();
	bool isOpen() const { return fd >= 0; }

	// Returns false if the datagram could not be sent (not open, too large,
	// socket error). Never blocks.
	bool send(const std::string & payload);

	const std::string & getHost() const { return host; }
	int getPort() const { return port; }
	unsigned long getSentCount() const { return sent; }
	unsigned long getErrorCount() const { return errors; }

private:
	int fd = -1;
	std::string host;
	int port = 0;
	unsigned long sent = 0, errors = 0;
	unsigned char addrStorage[128] = {};
	int addrLen = 0;
};

} // namespace output
