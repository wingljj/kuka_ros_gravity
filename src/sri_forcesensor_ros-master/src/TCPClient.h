#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <array>
#include <vector>

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class TCPClient
{
private:
  int sock;
  std::string address;
  int port;
  struct sockaddr_in server;
  std::vector<unsigned char> receive_buffer_;

public:
  TCPClient();
  bool setup(string address, int port);
  bool Send(string data);
  string receive(int size = 4096);
  bool GetADCounts(MatrixXd &pdBuffer);
  bool GetChParameter(MatrixXd &pdBuffer);
  bool readrecieveBuffer(MatrixXd &pdBuffer);
  bool readrecieveBuffer_IEEEfloat32(MatrixXd &pdBuffer);
  static bool extractLatestEngineeringFrame(std::vector<unsigned char>& stream,
                                            std::array<double, 6>& values);
  string read();
  void exit();
};

#endif
