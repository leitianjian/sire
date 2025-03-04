#include "sire/server/sire_socket_server.hpp"

#include <stdint.h>

#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <new>
#include <thread>

#ifdef WIN32
#include <ws2tcpip.h>
#ifdef max
#undef max
#endif
#endif

#ifdef UNIX
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <errno.h>  

#include <algorithm>
#include <map>
#include <sstream>

#include "sha1.h"

#include <aris/core/log.hpp>
#include <aris/core/reflection.hpp>

#include "sire/core/constants.hpp"

#define SOCKET_FAILED_ACCEPT           \
  aris::core::LogLvl::kError, -3001, { \
    "socket failed to accept : %d"     \  
    "Socket 接受链接失败：%d"   \
  }
#define WEBSOCKET_SHAKE_HAND_FAILED     \
  aris::core::LogLvl::kError, -3002, {  \
    "websocket shake hand failed : %d"  \
    "Websocket 协议握手失败：%d" \
  }
#define WEBSOCKET_SHAKE_HAND_FAILED_INVALID_KEY      \
  aris::core::LogLvl::kError, -3003, {               \
    "websocket shake hand failed : invalid key"      \
    "Websocket 协议握手失败：非法的key值" \
  }
#define WEBSOCKET_SHAKE_HAND_FAILED_LOOSE_CONNECTION                 \
  aris::core::LogLvl::kError, -3004, {                               \
    "websocket shake hand failed : lose connection before succesful" \
    "Websocket 协议握手失败：提前失去连接"              \
  }
#define WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT                                \
  aris::core::LogLvl::kError, -3005, {                                    \
    "websocket receive too large or negative object, size:%ji"            \
    "Websocket 数据接收失败，过大的数据包，字节数：%ji" \
  }
#define WEBSOCKET_RECEIVE_RAW                                                \
  aris::core::LogLvl::kError, -3006, {                                       \
    "websocket espect msg, but receive raw data"                             \
    "Websocket 数据接受失败，数据不是消息类型（Msg类型）" \
  }
#define WEBSOCKET_RECEIVE_WRONG_MSG_SIZE                                                            \
  aris::core::LogLvl::kError, -3007, {                                                              \
    "websocket receive wrong msg size, msg size:%i payload size:%ji"                                \
    "Websocket 数据接受失败，错误的消息大小，消息大小：%i，负载大小：%ji" \
  }
#define SOCKET_UDP_WRONG_MSG_SIZE      \
  aris::core::LogLvl::kError, -3008, { \
    "UDP msg size not correct"         \
    "UDP 消息大小不对"           \
  }
#define SOCKET_SHUT_DOWN_ERROR          \
  aris::core::LogLvl::kError, -3009, {  \
    "socket shut down error %d"         \
    "Socket Shutdown 关闭错误：%d" \
  }
#define SOCKET_SHUT_CLOSE_ERROR        \
  aris::core::LogLvl::kError, -3010, { \
    "socket close error %d"            \
    "Socket Close 关闭错误：%d"   \
  }
#define SOCKET_SERVER_START_ERROR          \
  aris::core::LogLvl::kError, -3011, {     \
    "SocketServer startServer error: %s"   \
    "SocketServer startServer 错误：%s" \
  }
#define SOCKET_SERVER_STOP_ERROR       \
  aris::core::LogLvl::kError, -3012, { \
    "SocketServer stop error: %d"      \
    "SocketServer stop 错误：%d"    \
  }
#define SOCKET_SERVER_SEND_MSG_ERROR   \
  aris::core::LogLvl::kError, -3013, { \
    "SocketServer sendMsg error: %d"   \
    "SocketServer sendMsg 错误：%d" \
  }
#define SOCKET_SERVER_SEND_RAW_DATA_ERROR  \
  aris::core::LogLvl::kError, -3014, {     \
    "SocketServer sendRawData error: %d"   \
    "SocketServer sendRawData 错误：%d" \
  }

namespace sire::server {
auto sire_send(decltype(socket(AF_INET, SOCK_STREAM, 0)) sock, const char* buf,
               int len, int flag) -> int {
#ifdef WIN32
  return ::send(sock, buf, len, flag);
#endif
#ifdef UNIX
  return ::send(sock, buf, len, flag & MSG_NOSIGNAL);
#endif
}
auto sire_close(decltype(socket(AF_INET, SOCK_STREAM, 0)) sock) -> int {
#ifdef WIN32
  auto ret = ::closesocket(sock);
#endif
#ifdef UNIX
  auto ret = ::close(sock);
#endif
  return ret;
}
auto safe_recv333(SOCKET_T s, char* data, int size) -> int {
  int result{0};
  for (; result < size;) {
    int ret = recv(s, data + result, size - result, 0);
    if (ret <= 0) {
      sire_close(s);
      result = ret;
      break;
    }

    result += ret;
  }

  return result;
};
auto make_header_map2(const std::string& hand_shake_text)
    -> std::map<std::string, std::string> {
  std::istringstream istream(hand_shake_text);

  // 找到Sec-WebSocket-Key //
  std::map<std::string, std::string> header_map;
  std::string header;
  while (std::getline(istream, header) && header != "\r") {
    if (header[header.size() - 1] != '\r') {
      continue;  // end
    } else {
      header.erase(header.end() - 1);  // remove last char
    }

    auto end = header.find(": ", 0);
    if (end != std::string::npos) {
      std::string key = header.substr(0, end);
      std::string value = header.substr(end + 2);
      header_map[key] = value;
    }
  }

  return header_map;
}
auto pack_data_server(const char* data, int size) -> std::string {
  std::string s;
  if (size < 126) {
    s.resize(std::size_t(size) + 2);
    s[0] = char(0x82);  // binary data, 0x81 is text data
    s[1] = char(size);
    std::copy_n(data, size, &s[2]);
  } else if (size < 0xFFFF) {
    s.resize(std::size_t(size) + 4);
    s[0] = char(0x82);
    s[1] = char(126);
    s[2] = size >> 8;
    s[3] = size & 0xFF;
    std::copy_n(data, size, &s[4]);
  } else {
    s.resize(std::size_t(size) + 10);
    s[0] = char(0x82);
    s[1] = char(127);
    s[2] = 0;
    s[3] = 0;
    s[4] = 0;
    s[5] = 0;
    s[6] = size >> 24;
    s[7] = size >> 16;
    s[8] = size >> 8;
    s[9] = size & 0xFF;
    std::copy_n(data, size, &s[10]);
  }

  return s;
};
// please refer to
// https://www.cnblogs.com/chyingp/p/websocket-deep-in.html
// sha1 hash 生成出来的是纯数字，可以把它改成2进制来保存
//
std::string base64_encode2_2(unsigned char const* bytes_to_encode,
                             unsigned int in_len) {
  static const std::string base64_chars =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz"
      "0123456789+/";

  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] =
          ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] =
          ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for (i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 3; j++) char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] =
        ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] =
        ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

    for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];

    while ((i++ < 3)) ret += '=';
  }

  return ret;
}
auto pack_data_client(const char* data, int size) -> std::string {
  std::string s;
  if (size < 126) {
    s.resize(std::size_t(size) + 6);
    s[0] = char(0x82);  // binary data, 0x81 is text data
    s[1] = char(size) | char(0x80);
    std::copy_n(data, size, &s[6]);
  } else if (size < 0xFFFF) {
    s.resize(std::size_t(size) + 8);
    s[0] = char(0x82);
    s[1] = char(126) | char(0x80);
    s[2] = size >> 8;
    s[3] = size & 0xFF;
    std::copy_n(data, size, &s[8]);
  } else {
    s.resize(std::size_t(size) + 14);
    s[0] = char(0x82);
    s[1] = char(127) | char(0x80);
    s[2] = 0;
    s[3] = 0;
    s[4] = 0;
    s[5] = 0;
    s[6] = size >> 24;
    s[7] = size >> 16;
    s[8] = size >> 8;
    s[9] = size & 0xFF;
    std::copy_n(data, size, &s[14]);
  }

  std::fill_n(&s[s.size() - size - 4], 4, 0xf0);
  for (int i = 0; i < size; ++i) {
    s[i + s.size() - size] =
        s[i + s.size() - size] ^ s[s.size() - size - 4 + i % 4];
  }

  return s;
};
auto make_header_map(const std::string& hand_shake_text)
    -> std::map<std::string, std::string> {
  std::istringstream istream(hand_shake_text);

  // 找到Sec-WebSocket-Key //
  std::map<std::string, std::string> header_map;
  std::string header;
  while (std::getline(istream, header) && header != "\r") {
    if (header[header.size() - 1] != '\r') {
      continue;  // end
    } else {
      header.erase(header.end() - 1);  // remove last char
    }

    auto end = header.find(": ", 0);
    if (end != std::string::npos) {
      std::string key = header.substr(0, end);
      std::string value = header.substr(end + 2);
      header_map[key] = value;
    }
  }

  return header_map;
}
auto pack_data_server2(const char* data, int size) -> std::string {
  std::string s;
  if (size < 126) {
    s.resize(std::size_t(size) + 2);
    s[0] = char(0x82);  // binary data, 0x81 is text data
    s[1] = char(size);
    std::copy_n(data, size, &s[2]);
  } else if (size < 0xFFFF) {
    s.resize(std::size_t(size) + 4);
    s[0] = char(0x82);
    s[1] = char(126);
    s[2] = size >> 8;
    s[3] = size & 0xFF;
    std::copy_n(data, size, &s[4]);
  } else {
    s.resize(std::size_t(size) + 10);
    s[0] = char(0x82);
    s[1] = char(127);
    s[2] = 0;
    s[3] = 0;
    s[4] = 0;
    s[5] = 0;
    s[6] = size >> 24;
    s[7] = size >> 16;
    s[8] = size >> 8;
    s[9] = size & 0xFF;
    std::copy_n(data, size, &s[10]);
  }

  return s;
};
auto pack_data_client2(const char* data, int size) -> std::string {
  std::string s;
  if (size < 126) {
    s.resize(std::size_t(size) + 6);
    s[0] = char(0x82);  // binary data, 0x81 is text data
    s[1] = char(size) | char(0x80);
    std::copy_n(data, size, &s[6]);
  } else if (size < 0xFFFF) {
    s.resize(std::size_t(size) + 8);
    s[0] = char(0x82);
    s[1] = char(126) | char(0x80);
    s[2] = size >> 8;
    s[3] = size & 0xFF;
    std::copy_n(data, size, &s[8]);
  } else {
    s.resize(std::size_t(size) + 14);
    s[0] = char(0x82);
    s[1] = char(127) | char(0x80);
    s[2] = 0;
    s[3] = 0;
    s[4] = 0;
    s[5] = 0;
    s[6] = size >> 24;
    s[7] = size >> 16;
    s[8] = size >> 8;
    s[9] = size & 0xFF;
    std::copy_n(data, size, &s[14]);
  }

  std::fill_n(&s[s.size() - size - 4], 4, 0xf0);
  for (int i = 0; i < size; ++i) {
    s[i + s.size() - size] =
        s[i + s.size() - size] ^ s[s.size() - size - 4 + i % 4];
  }

  return s;
};

struct SockData {
  SOCKET_T sock_{0};

  // recv data //
  int64_t required_length_{sizeof(aris::core::MsgHeader)};
  int64_t received_length_{0};
  std::vector<char> mem_;

  // pack data，多个数据帧（frame）拼成完整的数据后，触发回调 //
  int64_t data_required_length_{0};
  int64_t data_received_length_{0};
  std::vector<char> data_mem_;

  // remote info //
  struct sockaddr_in remote_addr_ {};
  std::string remote_ip_;

  // RAII to close sock
  ~SockData() {
    if (::shutdown(sock_, 2) < 0) ARIS_LOG(SOCKET_SHUT_CLOSE_ERROR, errno);
    sire_close(sock_);
  }
};

auto safe_recv2(SOCKET_T s, SockData& data) -> int {
  data.mem_.resize(std::max<int64_t>(data.required_length_, 1024));
  // WIN32 Winsock.h ::recv(SOCKET s, char *buf, int len, int flags)
  //
  // Return: If no error occurs, recv returns the number of bytes received and
  // the buffer pointed to by the buf parameter will contain this data received.
  // If the connection has been gracefully closed, the return value is zero.
  //
  // Otherwise, a value of SOCKET_ERROR is returned, and a specific error code
  // can be retrieved by calling WSAGetLastError.
  auto ret = ::recv(
      s, data.mem_.data() + data.received_length_,
      (int)std::max<int64_t>(data.required_length_ - data.received_length_,
                             1024 - data.received_length_),
      0);
  if (ret >= 0) data.received_length_ += ret;
  return ret;
};

struct SireSocketServer::Imp {
  SireSocketServer::Type type_{Type::TCP};
  std::string port_;

  SireSocketServer* socket_server_;
  SireSocketServer::State state_{State::IDLE};

  SOCKET_T lisn_socket_;  // 也可以用SOCKET类型
  std::map<SOCKET_T, SockData> sock_datas_;

  // callbacks //
  ReceiveMsgCallback on_receive_msg_;
  ReceiveRawDataCallback on_receive_raw_data_;
  ReceiveConnectionCallback on_receive_connection_;
  LoseConnectionCallback on_lose_connection_;

  // for udp ... //
  struct sockaddr_in server_addr_ {
  }, client_addr_{};
  socklen_t sin_size_;

  // 线程同步变量 //
  std::recursive_mutex state_mutex_;
  std::thread accept_thread_;

  // 连接的socket //
#ifdef WIN32
  WSADATA wsa_data_;  // windows下才用,linux下无该项
#endif
  ~Imp() = default;
  Imp(SireSocketServer* sock)
      : socket_server_(sock),
        lisn_socket_(0),
        sin_size_(sizeof(struct sockaddr_in)),
        state_(SireSocketServer::State::IDLE),
        on_receive_msg_(nullptr),
        on_receive_raw_data_(nullptr),
        on_receive_connection_(nullptr),
        on_lose_connection_(nullptr) {}

  static void acceptThread(SireSocketServer::Imp* imp,
                           std::promise<void> accept_thread_ready);

  auto init_all_socks() -> void {
    lisn_socket_ = 0;
    sock_datas_.clear();
  }
  auto close_all_socks() -> void {
    sock_datas_.clear();

    if (shutdown(lisn_socket_, 2) < 0) ARIS_LOG(SOCKET_SHUT_CLOSE_ERROR, errno);
    sire_close(lisn_socket_);
  }
  auto recv_from_sock2(SOCKET_T recv_sock, SockData& recv_data) -> int {
    aris::core::Msg recv_msg;
    recv_msg.resize(1024);

    int ret = 0;
    // 开启接受数据的循环 //
    switch (type_) {
      case SireSocketServer::Type::TCP: {
        ret = 0;
        if ((ret = safe_recv2(recv_sock, recv_data)) <= 0) return ret;

        while (recv_data.received_length_ >= recv_data.required_length_) {
          // 判断是否已经完整收到了MsgHeader
          if (recv_data.received_length_ >= sizeof(aris::core::MsgHeader)) {
            auto msg_size =
                reinterpret_cast<aris::core::MsgHeader*>(recv_data.mem_.data())
                    ->msg_size_;
            if (msg_size > 0x00100000 || msg_size < 0) return -1;
            // 根据MsgHeader中的包size信息，设置接受的大小
            recv_data.required_length_ =
                msg_size + sizeof(aris::core::MsgHeader);
          }
          if (recv_data.received_length_ >= recv_data.required_length_) {
            recv_msg.resize((aris::core::MsgSize)(
                recv_data.required_length_ - sizeof(aris::core::MsgHeader)));
            std::copy(recv_data.mem_.data(),
                      recv_data.mem_.data() + recv_data.received_length_,
                      (char*)(&recv_msg.header()));
            if (on_receive_msg_)
              on_receive_msg_(socket_server_, recv_sock, recv_msg);
            std::copy_n(recv_data.mem_.data() + recv_data.required_length_,
                        recv_data.received_length_ - recv_data.required_length_,
                        recv_data.mem_.data());
            recv_data.received_length_ -= recv_data.required_length_;
            recv_data.required_length_ = sizeof(aris::core::MsgHeader);
          }
        }
        break;
      }
      case Type::TCP_RAW: {
        char data[1024];
        ret = ::recv(recv_sock, data, 1024, 0);
        if (ret > 0 && on_receive_raw_data_)
          on_receive_raw_data_(socket_server_, recv_sock, data, ret);
        break;
      }
      case Type::WEB: {
        // int ret = 0;
        if ((ret = safe_recv2(recv_sock, recv_data)) <= 0) return ret;

        bool fin;
        char web_head[2];
        std::int64_t payload_len{0};
        while (recv_data.received_length_ >= recv_data.required_length_) {
          // 接受头 //
          int64_t frame_length = 0;
          recv_data.required_length_ = frame_length + 2;
          if (recv_data.received_length_ >= recv_data.required_length_) {
            std::copy_n(recv_data.mem_.data(), 2, web_head);

            // 是否最后一帧 //
            fin = (web_head[0] & 0x80) == 0x80;  // 1bit，1表示最后一帧

            // 获取opcode //
            std::int8_t op_code = web_head[0] & 0x0f;
            if (op_code == 0x08) return -1;

            // 获取数据长度
            payload_len = web_head[1] & 0x7F;  // 数据长度

            frame_length += 2;
          }

          recv_data.required_length_ = frame_length + 2;
          if (payload_len == 126 &&
              recv_data.received_length_ > recv_data.required_length_) {
            // 似乎有大小端的问题 //
            char reverse_char[2];
            for (int i = 0; i < 2; ++i) reverse_char[i] = recv_data.mem_[3 - i];
            payload_len = *reinterpret_cast<std::uint16_t*>(reverse_char);

            frame_length += 2;
          }

          recv_data.required_length_ = frame_length + 8;
          if (payload_len == 127 &&
              recv_data.received_length_ > recv_data.required_length_) {
            char reverse_char[8];
            for (int i = 0; i < 8; ++i)
              reverse_char[i] = recv_data.mem_[10 - i];
            payload_len = *reinterpret_cast<std::int64_t*>(reverse_char);

            frame_length += 8;
          }

          // 保护，数据不能太大 //
          if (payload_len < 0 || payload_len > 0x00080000 ||
              payload_len + recv_data.data_received_length_ > 0x00100000) {
            ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
            return -1;
          }

          // 获取掩码
          bool mask_flag = (web_head[1] & 0x80) == 0x80;  // 是否包含掩码
          char masks[4];
          recv_data.required_length_ = frame_length + 4;
          if (mask_flag &&
              recv_data.received_length_ > recv_data.required_length_) {
            std::copy_n(recv_data.mem_.data() + frame_length, 4, masks);
            frame_length += 4;
          }

          // 读取数据
          recv_data.required_length_ = frame_length + payload_len;
          if (recv_data.received_length_ >= recv_data.required_length_) {
            recv_data.data_mem_.resize(payload_len +
                                       recv_data.data_received_length_);

            if (mask_flag) {
              for (int i{0}; i < payload_len; ++i) {
                recv_data.data_mem_[i + recv_data.data_received_length_] =
                    recv_data.mem_[i + frame_length] ^ masks[i % 4];
              }
            } else {
              std::copy_n(
                  recv_data.mem_.data(), payload_len,
                  recv_data.data_mem_.data() + recv_data.data_received_length_);
            }

            recv_data.data_received_length_ += payload_len;
            frame_length += payload_len;
          }

          // 是否触发回调 //
          if (recv_data.received_length_ >= recv_data.required_length_ && fin) {
            // 把WebSocket接收到的的东西转成 msg //
            recv_msg.resize(static_cast<aris::core::MsgSize>(
                recv_data.data_received_length_ -
                sizeof(aris::core::MsgHeader)));
            std::copy_n(recv_data.data_mem_.data(),
                        recv_data.data_received_length_,
                        reinterpret_cast<char*>(&recv_msg.header()));

            if (recv_msg.size() != recv_data.data_received_length_ -
                                       sizeof(aris::core::MsgHeader)) {
              ARIS_LOG(WEBSOCKET_RECEIVE_WRONG_MSG_SIZE, recv_msg.size(),
                       recv_data.data_received_length_);
              break;
            }

            if (on_receive_msg_)
              on_receive_msg_(socket_server_, recv_sock, recv_msg);

            recv_data.data_received_length_ = 0;
          }

          // 本帧已经收取完毕，准备下一帧
          if (recv_data.received_length_ >= recv_data.required_length_) {
            std::copy_n(recv_data.mem_.data() + recv_data.required_length_,
                        recv_data.received_length_ - recv_data.required_length_,
                        recv_data.mem_.data());
            recv_data.received_length_ -= recv_data.required_length_;
            recv_data.required_length_ = 2;
          }
        }
        return ret;
        break;
      }
      case Type::WEB_RAW: {
        std::int64_t real_length{0};
        std::string payload_data;

        for (bool fin{false}; !fin;) {
          // 接受头 //
          char web_head[2];
          if (safe_recv333(recv_sock, web_head, 2) <= 0) return -1;

          // 是否最后一帧 //
          fin = (web_head[0] & 0x80) == 0x80;  // 1bit，1表示最后一帧

          // 获取opcode //
          std::int8_t op_code = web_head[0] & 0x0f;
          if (op_code == 0x08) return -1;

          // 获取数据长度
          std::int64_t payload_len = web_head[1] & 0x7F;  // 数据长度
          if (payload_len == 126) {
            char length_char[2];
            if (safe_recv333(recv_sock, length_char, 2) <= 0) return -1;

            union {
              std::uint16_t length;
              char reverse_char[2];
            };
            for (int i = 0; i < 2; ++i) reverse_char[i] = length_char[1 - i];
            payload_len = length;
          } else if (payload_len == 127) {
            char length_char[8];
            if (safe_recv333(recv_sock, length_char, 8) <= 0) return -1;

            char reverse_char[8];
            for (int i = 0; i < 8; ++i) reverse_char[i] = length_char[7 - i];
            std::copy_n(reverse_char, 8, reinterpret_cast<char*>(&payload_len));
          }

          //////////////////////////////////保护，数据不能太大///////////////////////////////
          if (payload_len > 0x00100000 ||
              payload_len + payload_data.size() > 0x00200000) {
            ARIS_LOG(WEBSOCKET_RECEIVE_TOO_LARGE_OBJECT, payload_len);
            return -1;
          }

          // 获取掩码
          bool mask_flag = (web_head[1] & 0x80) == 0x80;  // 是否包含掩码
          char masks[4];
          if (mask_flag && safe_recv333(recv_sock, masks, 4) <= 0) return -1;

          // 用掩码读取出数据 //
          auto last_size = payload_data.size();
          payload_data.resize(payload_data.size() +
                              static_cast<std::size_t>(payload_len));
          if (safe_recv333(recv_sock, payload_data.data() + last_size,
                           static_cast<int>(payload_len)) <= 0)
            return -1;

          if (mask_flag) {
            for (int i{0}; i < payload_len; ++i) {
              payload_data[i + last_size] =
                  payload_data[i + last_size] ^ masks[i % 4];
            }
          }
        }

        // if (onReceivedData)onReceivedData(socket_, payload_data.data(),
        // static_cast<int>(payload_data.size()));
        break;
      }
      case Type::UDP: {
        ret = recvfrom(recv_sock, reinterpret_cast<char*>(&recv_msg.header()),
                       1024, 0, (struct sockaddr*)(&client_addr_), &sin_size_);

        // std::unique_lock<std::recursive_mutex> close_lck(close_mutex_,
        // std::defer_lock); if (ret <= 0 && !close_lck.try_lock()) return -1;
        if (ret != sizeof(aris::core::MsgHeader) + recv_msg.size()) {
          ARIS_LOG(SOCKET_UDP_WRONG_MSG_SIZE);
          break;
        }
        // if (ret > 0 && onReceivedMsg)onReceivedMsg(socket_, recv_msg);
        break;
      }
      case Type::UDP_RAW: {
        char data[1024];
        ret = recvfrom(recv_sock, data, 1024, 0,
                       (struct sockaddr*)(&client_addr_), &sin_size_);
        // std::unique_lock<std::recursive_mutex> close_lck(close_mutex_,
        // std::defer_lock); if (ret <= 0 && !close_lck.try_lock()) return -1;
        // if (ret > 0 && onReceivedData)onReceivedData(socket_, data, ret);
        break;
      }
    }
    return ret;
  }
};
auto SireSocketServer::Imp::acceptThread(SireSocketServer::Imp* imp,
                                         std::promise<void> accept_thread_ready)
    -> void {
  // 通知主线程,accept线程已经拷贝完毕,准备监听 //
  accept_thread_ready.set_value();

#ifdef UNIX
  signal(SIGPIPE, SIG_IGN);
#endif

  auto nfds = imp->lisn_socket_ + 1;
  ::fd_set f_s;
  for (;;) {
    FD_ZERO(&f_s);
    FD_SET(imp->lisn_socket_, &f_s);
    for (auto& fd : imp->sock_datas_) {
      FD_SET(fd.first, &f_s);
    }
    // map is sorted
    nfds =
        imp->sock_datas_.size() > 0
            ? std::max(imp->sock_datas_.crbegin()->first, imp->lisn_socket_) + 1
            : imp->lisn_socket_ + 1;

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    auto select_ret = ::select(nfds, &f_s, nullptr, nullptr, &tv);

    // lock data //
    std::unique_lock<std::recursive_mutex> lck(imp->state_mutex_);

    // close all sock //
    if (imp->state_ == State::IDLE) {
      imp->close_all_socks();
      return;
    }

    if (select_ret > 0) {
      if (FD_ISSET(imp->lisn_socket_, &f_s) > 0) {
        auto recv_sock = static_cast<SOCKET_T>(
            ::accept(imp->lisn_socket_, (struct sockaddr*)(&imp->client_addr_),
                     &imp->sin_size_));
        if (recv_sock == -1) {
          ARIS_LOG(SOCKET_FAILED_ACCEPT, (int)recv_sock);
          continue;
        }

        imp->sock_datas_.insert(
            std::pair<SOCKET_T, SockData>(recv_sock, SockData()));
        imp->sock_datas_[recv_sock].sock_ = recv_sock;
        switch (imp->socket_server_->connectType()) {
          case SireSocketServer::Type::TCP:
            imp->sock_datas_[recv_sock].required_length_ = 40;
            break;
          case SireSocketServer::Type::WEB:
            imp->sock_datas_[recv_sock].required_length_ = 2;
            break;
          default:
            break;
        }

#ifdef WIN32
        u_long block = 0;
        if (::ioctlsocket(recv_sock, FIONBIO, &block) == SOCKET_ERROR) {
          imp->sock_datas_.erase(recv_sock);
          continue;
        }
#endif
#ifdef UNIX
        long arg;
        if ((arg = fcntl(recv_sock, F_GETFL, NULL)) < 0) {
          imp->sock_datas_.erase(recv_sock);
          continue;
        }
        arg &= (~O_NONBLOCK);
        if (fcntl(recv_sock, F_SETFL, arg) < 0) {
          imp->sock_datas_.erase(recv_sock);
          continue;
        }
#endif

        if (imp->type_ == Type::WEB || imp->type_ == Type::WEB_RAW) {
          char recv_data[1024]{0};
          int res = ::recv(recv_sock, recv_data, 1024, 0);
          if (res <= 0) {
            ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED, res);
            imp->sock_datas_.erase(recv_sock);
            continue;
          }

          auto header_map = make_header_map2(recv_data);
          std::string server_key;
          try {
            server_key = header_map.at("Sec-WebSocket-Key");
          } catch (std::exception&) {
            ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_INVALID_KEY);
            imp->sock_datas_.erase(recv_sock);
            continue;
          }
          server_key += "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

          // 找到返回的key //
          SHA1 checksum;
          checksum.update(server_key);
          std::string hash = checksum.final();

          std::uint32_t message_digest[5]{};
          for (Size i = 0; i < 20; ++i) {
            char num[5] = "0x00";
            std::copy_n(hash.data() + i * 2, 2, num + 2);
            std::uint8_t n = std::stoi(num, 0, 16);
            *(reinterpret_cast<unsigned char*>(message_digest) + i) = n;
          }

          auto ret_hey = base64_encode2_2(
              reinterpret_cast<const unsigned char*>(message_digest), 20);

          std::string shake_hand;
          shake_hand =
              "HTTP/1.1 101 Switching Protocols\r\n"
              "Upgrade: websocket\r\n"
              "Connection: Upgrade\r\n"
              "Sec-WebSocket-Accept: " +
              ret_hey + std::string("\r\n\r\n");

          auto ret = sire_send(recv_sock, shake_hand.c_str(),
                               static_cast<int>(shake_hand.size()), 0);

          if (ret == -1) {
            ARIS_LOG(WEBSOCKET_SHAKE_HAND_FAILED_LOOSE_CONNECTION);
            imp->sock_datas_.erase(recv_sock);
            continue;
          };
        }

        // CALL BACK //
        if (imp->on_receive_connection_) {
          char buff[16];
          imp->on_receive_connection_(
              imp->socket_server_, recv_sock,
              inet_ntop(imp->client_addr_.sin_family,
                        &imp->client_addr_.sin_addr, buff, 16),
              ntohs(imp->client_addr_.sin_port));
        }
      }

      for (auto recv_iter = imp->sock_datas_.begin();
           recv_iter != imp->sock_datas_.end(); recv_iter++) {
        auto& recv_sock = recv_iter->first;
        auto& recv_sock_data = recv_iter->second;

        if (FD_ISSET(recv_sock, &f_s)) {
          if (imp->recv_from_sock2(recv_sock, recv_sock_data) <= 0) {
            imp->sock_datas_.erase(recv_sock);
            break;
          }
        }
      }
    }
  }

  return;
}
auto SireSocketServer::port() const -> const std::string& {
  return imp_->port_;
}
auto SireSocketServer::setPort(const std::string& port) -> void {
  imp_->port_ = port;
}
auto SireSocketServer::connectType() const -> Type { return imp_->type_; }
auto SireSocketServer::setConnectType(const Type type) -> void {
  imp_->type_ = type;
}

auto SireSocketServer::setOnReceivedMsg(ReceiveMsgCallback on_receive_msg_func)
    -> void {
  imp_->on_receive_msg_ = on_receive_msg_func;
}
auto SireSocketServer::setOnReceivedRawData(
    ReceiveRawDataCallback on_receive_raw_data_func) -> void {
  imp_->on_receive_raw_data_ = on_receive_raw_data_func;
}
auto SireSocketServer::setOnReceivedConnection(
    ReceiveConnectionCallback on_receive_connection) -> void {
  imp_->on_receive_connection_ = on_receive_connection;
}
auto SireSocketServer::setOnLoseConnection(
    LoseConnectionCallback on_lose_connection_func) -> void {
  imp_->on_lose_connection_ = on_lose_connection_func;
}

auto SireSocketServer::state() -> State {
  std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
  return imp_->state_;
}
auto SireSocketServer::startServer(const std::string& port) -> int {
  std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

  if (imp_->accept_thread_.joinable()) {
    ARIS_LOG(SOCKET_SERVER_START_ERROR, "already started");
    return -1;
  }

  switch (imp_->state_) {
    case State::IDLE:
      break;
    default:
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "wrong state");
      return -2;
  }

  if (!port.empty()) setPort(port);
  if (this->port().empty()) {
    ARIS_LOG(SOCKET_SERVER_START_ERROR, "wrong port");
    return -3;
  }

  imp_->init_all_socks();

  //////////////////////////////////////////////////////////////////////////////////////////////
  int sock_type;
  switch (imp_->socket_server_->connectType()) {
    case Type::TCP:
    case Type::TCP_RAW:
    case Type::WEB:
    case Type::WEB_RAW:
      sock_type = SOCK_STREAM;
      break;
    case Type::UDP:
    case Type::UDP_RAW:
      sock_type = SOCK_DGRAM;
      break;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////

  // 服务器端开始建立socket描述符 //
  if (static_cast<int>(imp_->lisn_socket_ = static_cast<SOCKET_T>(
                           socket(AF_INET, sock_type, 0))) == -1) {
    ARIS_LOG(SOCKET_SERVER_START_ERROR, "failed socket");
    return -4;
  }

  // linux 下设置keep alive
#ifdef UNIX
  if (sock_type == SOCK_STREAM) {
    int tcp_timeout = 10000;  // 10 seconds before aborting a write()
    if (setsockopt(imp_->lisn_socket_, SOL_TCP, TCP_USER_TIMEOUT, &tcp_timeout,
                   sizeof(int)) < 0) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_USER_TIMEOUT FAILED");
      return -5;
    }

    // Set the option active //
    int keepAlive = 1;  // 开启keepalive属性
    int keepIdle = 5;  // 如该连接在5秒内没有任何数据往来,则进行探测
    int keepInterval = 1;  // 探测时发包的时间间隔为5 秒
    int keepCount =
        5;  // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发.

    if (setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_KEEPALIVE,
                   (void*)&keepAlive, sizeof(keepAlive)) < 0) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_KEEPALIVE FAILED");
      return -6;
    }
    if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPIDLE,
                   (void*)&keepIdle, sizeof(keepIdle)) < 0) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_KEEPIDLE FAILED");
      return -7;
    }
    if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPINTVL,
                   (void*)&keepInterval, sizeof(keepInterval)) < 0) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_KEEPINTVL FAILED");
      return -8;
    }
    if (setsockopt(imp_->lisn_socket_, IPPROTO_TCP, TCP_KEEPCNT,
                   (void*)&keepCount, sizeof(keepCount)) < 0) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt TCP_KEEPCNT FAILED");
      return -9;
    }
  }
#endif

  // 设置socketopt选项,使得地址在程序结束后立即可用 //
  int nvalue = 1;
  if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<char*>(&nvalue), sizeof(int)) < 0) {
    sire_close(imp_->lisn_socket_);
    ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_REUSEADDR FAILED");
    return -10;
  }

  // 服务器端填充server_addr_结构,并且bind //
  memset(&imp_->server_addr_, 0, sizeof(struct sockaddr_in));
  imp_->server_addr_.sin_family = AF_INET;
  imp_->server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  imp_->server_addr_.sin_port = htons(std::stoi(imp_->port_));
  if (::bind(imp_->lisn_socket_, (struct sockaddr*)(&imp_->server_addr_),
             sizeof(struct sockaddr)) == -1) {
#ifdef WIN32
    int err = WSAGetLastError();
#endif
    sire_close(imp_->lisn_socket_);
    ARIS_LOG(SOCKET_SERVER_START_ERROR, "bind FAILED");
    return -11;
  }

  if (imp_->socket_server_->connectType() == Type::TCP ||
      imp_->socket_server_->connectType() == Type::TCP_RAW ||
      imp_->socket_server_->connectType() == Type::WEB ||
      imp_->socket_server_->connectType() == Type::WEB_RAW) {
    // 监听lisn_socket_描述符 //
    if (::listen(imp_->lisn_socket_, 5) == -1) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "listen FAILED");
      return -12;
    }
  } else {
    // 因为UDP没法shutdown，所以用非阻塞模式 //
#ifdef WIN32
    DWORD read_timeout = 10;
    if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO,
                     reinterpret_cast<char*>(&read_timeout),
                     sizeof(read_timeout)) < 0) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_RCVTIMEO FAILED");
      return -13;
    }
#endif
#ifdef UNIX
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10000;
    if (::setsockopt(imp_->lisn_socket_, SOL_SOCKET, SO_RCVTIMEO,
                     reinterpret_cast<char*>(&read_timeout),
                     sizeof(read_timeout)) < 0) {
      sire_close(imp_->lisn_socket_);
      ARIS_LOG(SOCKET_SERVER_START_ERROR, "setsockopt SO_RCVTIMEO FAILED");
      return -13;
    }
#endif
    std::promise<void> receive_thread_ready;
    auto fut = receive_thread_ready.get_future();
    fut.wait();
  }

  // 改变状态 //
  imp_->state_ = State::WORKING;
  imp_->sock_datas_.clear();

  // 启动等待连接的线程 //
  std::promise<void> accept_thread_ready;
  auto ready = accept_thread_ready.get_future();

  imp_->accept_thread_ = std::thread(Imp::acceptThread, this->imp_.get(),
                                     std::move(accept_thread_ready));

  try {
    ready.get();
    return 0;
  } catch (...) {
    imp_->accept_thread_.join();
    std::rethrow_exception(std::current_exception());
  }
}
auto SireSocketServer::stop() -> int {
  {
    std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);
    imp_->state_ = State::IDLE;
  }

  if (imp_->accept_thread_.joinable()) imp_->accept_thread_.join();

  return 0;
}
auto SireSocketServer::sendMsg(SOCKET_T sock,
                               const aris::core::MsgBase& data) -> int {
#ifdef UNIX
  signal(SIGPIPE, SIG_IGN);
#endif

  std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

  auto ip = imp_->sock_datas_.at(sock).remote_ip_;

  switch (imp_->state_) {
    case State::WORKING: {
      switch (imp_->type_) {
        case Type::TCP:
          if (auto ret =
                  sire_send(sock, reinterpret_cast<const char*>(&data.header()),
                            data.size() + sizeof(aris::core::MsgHeader), 0);
              ret < 0) {
            ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, ret);
            return ret;
          }
          break;
        case Type::WEB: {
          auto packed_data =
              pack_data_server2(reinterpret_cast<const char*>(&data.header()),
                                data.size() + sizeof(aris::core::MsgHeader));
          if (auto ret = sire_send(sock, packed_data.data(),
                                   static_cast<int>(packed_data.size()), 0);
              ret < 0) {
            ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, ret);
            return ret;
          }
          break;
        }
        case Type::UDP: {
          memset(&imp_->server_addr_, 0, sizeof(imp_->server_addr_));
          imp_->server_addr_.sin_family = AF_INET;
          char buff[4];
          imp_->server_addr_.sin_addr.s_addr =
              inet_pton(AF_INET, ip.c_str(), buff);
          imp_->server_addr_.sin_port = htons(std::stoi(this->port()));

          if (sendto(sock, reinterpret_cast<const char*>(&data.header()),
                     data.size() + sizeof(aris::core::MsgHeader), 0,
                     (const struct sockaddr*)&imp_->server_addr_,
                     sizeof(imp_->server_addr_)) == -1)
            THROW_FILE_LINE(
                "SocketMultiIo failed sending data, because network failed\n");
          else
            return 0;

          break;
        }
        default:
          ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, -1);
          return -1;
      }
    }
    default:
      ARIS_LOG(SOCKET_SERVER_SEND_MSG_ERROR, -2);
      return -1;
  }

  return 0;
}
auto SireSocketServer::sendRawData(SOCKET_T sock, const char* data,
                                   int size) -> int {
#ifdef UNIX
  signal(SIGPIPE, SIG_IGN);
#endif
  std::unique_lock<std::recursive_mutex> lck(imp_->state_mutex_);

  auto ip = imp_->sock_datas_.at(sock).remote_ip_;

  switch (imp_->state_) {
    case State::WORKING: {
      switch (imp_->type_) {
        case Type::TCP_RAW: {
          if (auto ret = sire_send(sock, data, size, 0); ret < 0) {
            ARIS_LOG(SOCKET_SERVER_SEND_RAW_DATA_ERROR, ret);
            return ret;
          }
          break;
        }
        case Type::WEB_RAW: {
          auto packed_data = pack_data_client2(data, size);
          if (auto ret = sire_send(sock, packed_data.data(),
                                   static_cast<int>(packed_data.size()), 0);
              ret < 0) {
            ARIS_LOG(SOCKET_SERVER_SEND_RAW_DATA_ERROR, ret);
            return ret;
          }
          break;
        }
        case Type::UDP_RAW: {
          memset(&imp_->server_addr_, 0, sizeof(imp_->server_addr_));
          imp_->server_addr_.sin_family = AF_INET;
          char buff[4];
          imp_->server_addr_.sin_addr.s_addr =
              inet_pton(AF_INET, ip.c_str(), buff);
          imp_->server_addr_.sin_port = htons(std::stoi(this->port()));

          if (sendto(sock, data, size, 0,
                     (const struct sockaddr*)&imp_->server_addr_,
                     sizeof(imp_->server_addr_)) == -1)
            THROW_FILE_LINE(
                "SireSocketServer failed sending data, because network "
                "failed\n");
          else
            return 0;

          break;
        }
        default:
          ARIS_LOG(SOCKET_SERVER_SEND_RAW_DATA_ERROR, -1);
      }
    }
    default:
      ARIS_LOG(SOCKET_SERVER_SEND_RAW_DATA_ERROR, -1);
  }

  return 0;
}
// auto SireSocketServer::remoteIpMap()const->const std::map<SOCKET_T,
// std::string>& { 	return imp_->remote_ip_map_;
// }

SireSocketServer::~SireSocketServer() {
  if (imp_) {
    stop();
#ifdef WIN32
    WSACleanup();
#endif
  }
}
SireSocketServer::SireSocketServer(const std::string& name,
                                   const std::string& port, Type type)
    : imp_(new Imp(this)) {
  // 启动服务器 //
#ifdef WIN32
  if (WSAStartup(0x0101, &imp_->wsa_data_) != 0)
    THROW_FILE_LINE(
        "SireSocketServer can't Start as server, because it can't "
        "WSAstartup\n");
#endif
  setPort(port);
  setConnectType(type);
}
SireSocketServer::SireSocketServer(SireSocketServer&& s) noexcept {
  imp_ = std::move(s.imp_);
  imp_->socket_server_ = this;
};
SireSocketServer& SireSocketServer::operator=(SireSocketServer&& s) noexcept {
  imp_ = std::move(s.imp_);
  imp_->socket_server_ = this;
  return *this;
}
ARIS_REGISTRATION {
  aris::core::class_<SireSocketServer::Type>("SireSocketServer::connect_type")
      .textMethod(
          [](SireSocketServer::Type* v) -> std::string {
            auto type = *reinterpret_cast<SireSocketServer::Type*>(v);
            if (type == SireSocketServer::Type::TCP)
              return "TCP";
            else if (type == SireSocketServer::Type::TCP_RAW)
              return "TCP_RAW";
            else if (type == SireSocketServer::Type::WEB)
              return "WEB";
            else if (type == SireSocketServer::Type::WEB_RAW)
              return "WEB_RAW";
            else if (type == SireSocketServer::Type::UDP)
              return "UDP";
            else if (type == SireSocketServer::Type::UDP_RAW)
              return "UDP_RAW";
            else
              THROW_FILE_LINE("unknown connect type");
          },
          [](SireSocketServer::Type* v, std::string_view str) -> void {
            if (str == "TCP")
              *reinterpret_cast<SireSocketServer::Type*>(v) =
                  SireSocketServer::Type::TCP;
            else if (str == "TCP_RAW")
              *reinterpret_cast<SireSocketServer::Type*>(v) =
                  SireSocketServer::Type::TCP_RAW;
            else if (str == "WEB")
              *reinterpret_cast<SireSocketServer::Type*>(v) =
                  SireSocketServer::Type::WEB;
            else if (str == "WEB_RAW")
              *reinterpret_cast<SireSocketServer::Type*>(v) =
                  SireSocketServer::Type::WEB_RAW;
            else if (str == "UDP")
              *reinterpret_cast<SireSocketServer::Type*>(v) =
                  SireSocketServer::Type::UDP;
            else if (str == "UDP_RAW")
              *reinterpret_cast<SireSocketServer::Type*>(v) =
                  SireSocketServer::Type::UDP_RAW;
            else
              THROW_FILE_LINE("unknown connect type");
          });

  aris::core::class_<SireSocketServer>("SireSocketServer")
      .prop("connect_type", &SireSocketServer::setConnectType,
            &SireSocketServer::connectType)
      .prop("port", &SireSocketServer::setPort, &SireSocketServer::port);
}
}  // namespace sire::server