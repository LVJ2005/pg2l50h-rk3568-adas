#include "recognition_control.h"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace adas {
ControlServer::ControlServer(Runtime &r,std::string address,int port):runtime_(r),address_(std::move(address)),port_(port) {}
ControlServer::~ControlServer() {stop();}
bool ControlServer::start(std::string &error) {
  socket_=socket(AF_INET,SOCK_DGRAM,0);
  if(socket_<0) {error=std::strerror(errno);return false;}
  sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port_);
  const int reuse=1;setsockopt(socket_,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
  timeval timeout{0,200000};setsockopt(socket_,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  if(inet_pton(AF_INET,address_.c_str(),&address.sin_addr)!=1 ||
      bind(socket_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0) {
    error="UDP bind failed: "+std::string(std::strerror(errno));close(socket_);socket_=-1;return false;
  }
  worker_=std::thread(&ControlServer::run,this);return true;
}
void ControlServer::stop() {
  stopping_=true;
  if(worker_.joinable()) worker_.join();
  if(socket_>=0) {close(socket_);socket_=-1;}
}
void ControlServer::run() {
  while(!stopping_) {
    char buffer[514];sockaddr_in peer{};socklen_t length=sizeof(peer);
    const auto n=recvfrom(socket_,buffer,sizeof(buffer),0,reinterpret_cast<sockaddr*>(&peer),&length);
    if(n<0) {if(errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue;break;}
    std::string response;
    runtime_.command(std::string(buffer,static_cast<size_t>(n)),response);
    sendto(socket_,response.data(),response.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),length);
  }
}
} // namespace adas
