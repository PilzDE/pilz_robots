/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstring>
#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <prbt_hardware_support/modbus_socket_connection_check.h>

namespace prbt_hardware_support
{
struct timeval init_timeval(const unsigned int& secs, const unsigned int& usecs)
{
  struct timeval ret;
  ret.tv_sec = secs;
  ret.tv_usec = usecs;
  return ret;
}

bool checkIPConnection(const char* ip, const unsigned int& port)
{
  // Create socket for connection testing purpose
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);

  // set connection to non blocking, no timeout
  long file_descr_flags = fcntl(sockfd, F_GETFL, NULL);
  file_descr_flags |= O_NONBLOCK;
  fcntl(sockfd, F_SETFL, file_descr_flags);
  struct sockaddr_in serv_addr = init_sockaddr_in(port);
  serv_addr.sin_addr.s_addr = inet_addr(ip);
  connect(sockfd, (const sockaddr*)&serv_addr, sizeof(serv_addr));

  // set the sockfd filedescriptor to be affected in following select function
  fd_set writeset;
  FD_ZERO(&writeset);
  FD_SET(sockfd, &writeset);
  struct timeval tv = init_timeval(1, 0);  // timout is 1s

  int socket_ready_for_writing = select(sockfd + 1, nullptr, &writeset, nullptr, &tv);
  if (socket_ready_for_writing <= 0)
  {
    // Timeout or fail
    return false;
  }

  int optval;
  socklen_t optlen = sizeof(optval);
  int read_pending_errors_failed = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (void*)&optval, &optlen);
  close(sockfd);
  if ((read_pending_errors_failed == 0) && (optval == 0))
  {
    // wait one second to grant a free port
    std::this_thread::sleep_for(std::chrono::duration<double>(1));
    return true;
  }
  return false;
}
struct sockaddr_in init_sockaddr_in(const unsigned int& port)
{
  struct sockaddr_in ret;
  std::memset((char*)&ret, 0, sizeof(ret));
  ret.sin_family = AF_INET;
  ret.sin_port = htons((short unsigned int)port);
  return ret;
}

}  // namespace prbt_hardware_support