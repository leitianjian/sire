#ifndef SIRE_MESHCAT_SOCKET_HPP_
#define SIRE_MESHCAT_SOCKET_HPP_

#include <aris/core/object.hpp>
#include <aris/core/msg.hpp>

/**
 *       0                   1                   2                   3
      0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
     +-+-+-+-+-------+-+-------------+-------------------------------+
     |F|R|R|R| opcode|M| Payload len |    Extended payload length    |
     |I|S|S|S|  (4)  |A|     (7)     |             (16/64)           |
     |N|V|V|V|       |S|             |   (if payload len==126/127)   |
     | |1|2|3|       |K|             |                               |
     +-+-+-+-+-------+-+-------------+ - - - - - - - - - - - - - - - +
     |     Extended payload length continued, if payload len == 127  |
     + - - - - - - - - - - - - - - - +-------------------------------+
     |                               |Masking-key, if MASK set to 1  |
     +-------------------------------+-------------------------------+
     | Masking-key (continued)       |          Payload Data         |
     +-------------------------------- - - - - - - - - - - - - - - - +
     :                     Payload Data continued ...                :
     + - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - +
     |                     Payload Data continued ...                |
     +---------------------------------------------------------------+
		 https://datatracker.ietf.org/doc/html/rfc6455
 */

namespace sire::server{
	using SOCKET_T = int;

	/**
	 * WebSocket实现的限制，payload_len < 0x00080000
	 */
	class SireSocketServer {
	public:
		enum class State {
			IDLE = 0,
			WORKING,
		};
		enum class Type {
			TCP,
			UDP,
			WEB,
			TCP_RAW,
			UDP_RAW,
			WEB_RAW
		};

		using ReceiveMsgCallback = std::function<int(SireSocketServer* this_server, SOCKET_T sock, aris::core::Msg&)>;
		using ReceiveRawDataCallback = std::function<int(SireSocketServer* this_server, SOCKET_T sock, const char* data, int size)>;
		using ReceiveConnectionCallback = std::function<int(SireSocketServer* this_server, SOCKET_T sock, const char* remote_ip, int remote_port)>;
		using LoseConnectionCallback = std::function<int(SireSocketServer* this_server, SOCKET_T sock)>;

	public:
		auto port()const->const std::string&;
		auto setPort(const std::string& port) -> void;
		auto connectType()const->Type;
		auto setConnectType(const Type type) -> void;

		auto setOnReceivedMsg(ReceiveMsgCallback on_receive_msg_func = nullptr) -> void;
		auto setOnReceivedRawData(ReceiveRawDataCallback on_receive_raw_data_func = nullptr) -> void;
		auto setOnReceivedConnection(ReceiveConnectionCallback on_receive_connection = nullptr) -> void;
		auto setOnLoseConnection(LoseConnectionCallback on_lose_connection_func = nullptr) -> void;

		auto state() -> State;
		auto startServer(const std::string& port = std::string()) -> int;
		auto stop() -> int;
		auto sendMsg(SOCKET_T sock, const aris::core::MsgBase& data) -> int;
		auto sendRawData(SOCKET_T sock, const char* data, int size) -> int;
		//auto remoteIpMap()const->const std::map<SOCKET_T, std::string>&; // 根据 socket 索引 ip

		virtual ~SireSocketServer();
		SireSocketServer(const std::string& name = "socket", const std::string& port = "", Type type = Type::TCP);
		SireSocketServer(const SireSocketServer& other) = delete;
		SireSocketServer(SireSocketServer&& other)noexcept;
		SireSocketServer& operator=(const SireSocketServer& other) = delete;
		SireSocketServer& operator=(SireSocketServer&& other)noexcept;

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
}

#endif
