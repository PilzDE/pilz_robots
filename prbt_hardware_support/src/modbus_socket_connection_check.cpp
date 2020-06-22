/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <fcntl.h>
#include <prbt_hardware_support/modbus_socket_connection_check.h>


namespace prbt_hardware_support
{
bool checkIPConnection(const char* ip, const unsigned int port)
{
  int conresult;
  int optval;
  int sockfd;
  socklen_t optlen = sizeof(optval);
  long result;
  struct sockaddr_in serv_addr;
  fd_set writeset;
  struct timeval tv;

  tv.tv_sec = 1;
  tv.tv_usec = 0;  // timout is 1s

  sockfd = socket(AF_INET, SOCK_STREAM, 0);  // Create socket for connection testing purpose
  bzero((char*)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons((short unsigned int)port);

  result = fcntl(sockfd, F_GETFL, NULL);
  result |= O_NONBLOCK;
  fcntl(sockfd, F_SETFL, result);  // set connection to non blocking, no timeout
  serv_addr.sin_addr.s_addr = inet_addr(ip);
  conresult = connect(sockfd, (const sockaddr*)&serv_addr, sizeof(serv_addr));

  FD_ZERO(&writeset);         // clear writeset all to zero
  FD_SET(sockfd, &writeset);  // set the sockfd filedescriptor to be affected in following select function
  conresult =
      select(sockfd + 1, nullptr, &writeset, nullptr, &tv);  // wait if sockfd is ready for writing with tv timeout
  if (conresult <= 0)
  {
    /* Timeout or fail */
    return false;
  }
  conresult = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (void*)&optval,
                         &optlen);        // get from socket api if any error is pending
  if ((conresult == 0) && (optval == 0))  // if getsockopt was executed with success annd no error is returned from
                                          // socket api
  {
    ::close(sockfd);
    std::this_thread::sleep_for(std::chrono::duration<double>(1));  // wait one second to grant a free port
    return true;
  }
  return false;
}

}  // namespace prbt_hardware_support