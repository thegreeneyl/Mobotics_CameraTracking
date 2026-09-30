#include "UdpSender.h"

#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "ofLog.h"

namespace output {

namespace {
constexpr size_t kMaxDatagram = 60 * 1024;
}

UdpSender::~UdpSender(){
	close();
}

bool UdpSender::open(const std::string & h, int p){
	close();
	addrinfo hints{};
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	addrinfo * res = nullptr;
	const int rc = getaddrinfo(h.c_str(), std::to_string(p).c_str(), &hints, &res);
	if(rc != 0 || !res){
		ofLogError("UdpSender") << "cannot resolve " << h << ":" << p << " (" << gai_strerror(rc) << ")";
		return false;
	}
	fd = ::socket(res->ai_family, SOCK_DGRAM, 0);
	if(fd < 0){
		ofLogError("UdpSender") << "socket() failed: " << std::strerror(errno);
		freeaddrinfo(res);
		return false;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
	addrLen = static_cast<int>(res->ai_addrlen);
	std::memcpy(addrStorage, res->ai_addr, std::min<size_t>(sizeof(addrStorage), res->ai_addrlen));
	freeaddrinfo(res);
	host = h;
	port = p;
	sent = errors = 0;
	ofLogNotice("UdpSender") << "publishing to udp://" << host << ":" << port;
	return true;
}

void UdpSender::close(){
	if(fd >= 0) ::close(fd);
	fd = -1;
}

bool UdpSender::send(const std::string & payload){
	if(fd < 0) return false;
	if(payload.size() > kMaxDatagram){
		if(errors++ % 100 == 0){
			ofLogWarning("UdpSender") << "datagram too large (" << payload.size() << " bytes), dropped";
		}
		return false;
	}
	const ssize_t n = ::sendto(fd, payload.data(), payload.size(), 0,
	                           reinterpret_cast<const sockaddr *>(addrStorage), addrLen);
	if(n < 0){
		if(errors++ % 100 == 0){
			ofLogWarning("UdpSender") << "sendto failed: " << std::strerror(errno);
		}
		return false;
	}
	sent++;
	return true;
}

} // namespace output
