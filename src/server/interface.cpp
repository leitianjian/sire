#include "sire/server/interface.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <memory>
#include <queue>
#include <thread>
#include <unordered_map>

#include "md5.h"

#include <aris/control/control.hpp>
#include <aris/core/core.hpp>
#include <aris/server/control_server.hpp>
#include <aris/server/interface.hpp>

#include "sire/core/sire_log.hpp"
#include "sire/ext/fifo_map.hpp"
#include "sire/ext/json.hpp"
#include "sire/middleware/program_middleware.hpp"
#include "sire/server/api.hpp"

namespace sire::server {
auto parse_ret_value(std::vector<std::pair<std::string, std::any>>& ret)
    -> std::string {
#define APPEND_PAIR_TO_JSON(VALUE_TYPE)                          \
  if (auto value = std::any_cast<VALUE_TYPE>(&key_value.second)) \
    js[key_value.first] = *value;                                \
  else

  nlohmann::json js;
  for (auto& key_value : ret) {
    // if (auto value = std::any_cast<std::string>(&key_value.second))
    // std::cout << key_value.first << ":" << *value << std::endl;
    APPEND_PAIR_TO_JSON(bool)
    APPEND_PAIR_TO_JSON(int)
    APPEND_PAIR_TO_JSON(double)
    APPEND_PAIR_TO_JSON(std::string)
    APPEND_PAIR_TO_JSON(std::vector<bool>)
    APPEND_PAIR_TO_JSON(std::vector<int>)
    APPEND_PAIR_TO_JSON(std::vector<double>)
    APPEND_PAIR_TO_JSON(std::vector<std::string>)
    APPEND_PAIR_TO_JSON(nlohmann::json) {
      SIRE_DEBUG_LOG << "unrecognized return value" << std::endl;
    }
  }
#undef APPEND_PAIR_TO_JSON
  SIRE_LOG << js.dump(2) << std::endl;
  return js.dump(2);
}
auto onReceivedMsg(aris::core::Socket* socket, aris::core::Msg& msg) -> int {
  ARIS_COUT << "received " << std::endl;

  auto msg_data = std::string_view(msg.data(), msg.size());

  // LOG_INFO << "receive cmd:"
  //	<< msg.header().msg_size_ << "&"
  //	<< msg.header().msg_id_ << "&"
  //	<< msg.header().msg_type_ << "&"
  //	<< msg.header().reserved1_ << "&"
  //	<< msg.header().reserved2_ << "&"
  //	<< msg.header().reserved3_ << ":"
  //	<< msg_data << std::endl;

  try {
    aris::server::ControlServer::instance().executeCmdInCmdLine(
        std::string(msg.data(), msg.size()),
        [socket, msg](aris::plan::Plan& plan) -> void {
          // make return msg
          aris::core::Msg ret_msg(msg);

          // only copy if it is a str
          if (auto str = std::any_cast<std::string>(&plan.ret())) {
            ret_msg.copy(*str);
          } else if (auto js = std::any_cast<
                         std::vector<std::pair<std::string, std::any>>>(
                         &plan.ret())) {
            js->push_back(std::make_pair<std::string, std::any>(
                "return_code", plan.executeRetCode()));
            js->push_back(std::make_pair<std::string, std::any>(
                "return_message", std::string(plan.executeRetMsg())));
            ret_msg.copy(parse_ret_value(*js));
          }

          // return back to source
          try {
            socket->sendMsg(ret_msg);
          } catch (std::exception& e) {
            ARIS_COUT << e.what() << std::endl;
            // LOG_ERROR << e.what() << std::endl;
          }
        });
  } catch (std::exception& e) {
    std::vector<std::pair<std::string, std::any>> ret_pair;
    ret_pair.push_back(std::make_pair<std::string, std::any>(
        "return_code", int(aris::plan::Plan::PARSE_EXCEPTION)));
    ret_pair.push_back(std::make_pair<std::string, std::any>(
        "return_message", std::string(e.what())));
    std::string ret_str = parse_ret_value(ret_pair);

    ARIS_COUT << ret_str << std::endl;
    // LOG_ERROR << ret_str << std::endl;

    try {
      aris::core::Msg m = msg;
      m.copy(ret_str);
      socket->sendMsg(m);
    } catch (std::exception& e) {
      ARIS_COUT << e.what() << std::endl;
      // LOG_ERROR << e.what() << std::endl;
    }
  }

  return 0;
}
auto onReceivedConnection(aris::core::Socket* sock, const char* ip, int port)
    -> int {
  ARIS_COUT << "socket receive connection" << std::endl;
  // LOG_INFO << "socket receive connection:\n"
  //	<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip <<
  //"\n"
  //	<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port <<
  // std::endl;
  return 0;
}
auto onLoseConnection(aris::core::Socket* socket) -> int {
  ARIS_COUT << "socket lose connection" << std::endl;
  // LOG_INFO << "socket lose connection" << std::endl;
  for (;;) {
    try {
      socket->startServer(socket->port());
      break;
    } catch (std::runtime_error& e) {
      ARIS_COUT << e.what() << std::endl
                << "will try to restart server socket in 1s" << std::endl;
      // LOG_ERROR << e.what() << std::endl << "will try to restart server
      // socket in 1s" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  ARIS_COUT << "socket restart successful" << std::endl;
  // LOG_INFO << "socket restart successful" << std::endl;

  return 0;
}

#define ARIS_PRO_COUT ARIS_COUT << "pro "
struct ProgramWebInterface::Imp {
  std::unique_ptr<aris::core::Socket> sock_{new aris::core::Socket};

  std::function<int(aris::core::Socket*, aris::core::Msg&)> onReceiveMsg_;
  std::function<int(aris::core::Socket*, const char* data, int size)>
      onReceiveConnection_;
  std::function<int(aris::core::Socket*)> onLoseConnection_;
};
auto ProgramWebInterface::resetSocket(aris::core::Socket* sock) -> void {
  imp_->sock_.reset(sock);
  socket().setOnReceivedMsg(imp_->onReceiveMsg_);
  socket().setOnReceivedConnection(imp_->onReceiveConnection_);
  socket().setOnLoseConnection(imp_->onLoseConnection_);
}
auto ProgramWebInterface::socket() -> aris::core::Socket& {
  return *imp_->sock_;
}
auto ProgramWebInterface::open() -> void { socket().startServer(); }
auto ProgramWebInterface::isConnected() const -> bool {
  return imp_->sock_->isConnected();
}
auto ProgramWebInterface::close() -> void { socket().stop(); }
auto ProgramWebInterface::lastError() -> std::string {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->lastError();
  else
    return "";
}
auto ProgramWebInterface::lastErrorCode() -> int {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->lastErrorCode();
  else
    return 0;
}
auto ProgramWebInterface::lastErrorLine() -> int {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->lastErrorLine();
  else
    return 0;
}
auto ProgramWebInterface::isAutoMode() -> bool {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->isAutoMode();
  else
    return false;
}
auto ProgramWebInterface::isAutoRunning() -> bool {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->isAutoRunning();
  else
    return false;
}
auto ProgramWebInterface::isAutoPaused() -> bool {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->isAutoPaused();
  else
    return false;
}
auto ProgramWebInterface::isAutoStopped() -> bool {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->isAutoStopped();
  else
    return true;
}
auto ProgramWebInterface::currentFileLine() -> std::tuple<std::string, int> {
  if (auto pgm_mid = dynamic_cast<middleware::ProgramMiddleware*>(
          &(aris::server::ControlServer::instance().middleWare())))
    return pgm_mid->currentFileLine();
  else
    return std::make_tuple<std::string, int>("", 0);
}
ProgramWebInterface::ProgramWebInterface(const std::string& name,
                                         const std::string& port,
                                         aris::core::Socket::Type type)
    : Interface(name), imp_(new Imp) {
  imp_->onReceiveMsg_ = [this](aris::core::Socket* socket,
                               aris::core::Msg& msg) -> int {
    auto send_ret = [socket, msg](std::string str) -> void {
      try {
        aris::core::Msg ret_msg(msg);
        ret_msg.copy(str);
        socket->sendMsg(ret_msg);
      } catch (std::exception& e) {
        ARIS_COUT << e.what() << std::endl;
        // LOG_ERROR << e.what() << std::endl;
      }
    };

    // LOG_INFO << "receive cmd:"
    //	<< msg.header().msg_size_ << "&"
    //	<< msg.header().msg_id_ << "&"
    //	<< msg.header().msg_type_ << "&"
    //	<< msg.header().reserved1_ << "&"
    //	<< msg.header().reserved2_ << "&"
    //	<< msg.header().reserved3_ << ":"
    //	<< std::string_view(msg.data(), msg.size()) << std::endl;

    aris::server::ControlServer::instance().middleWare().executeCmd(
        std::string_view(msg.data(), msg.size()), send_ret,
        dynamic_cast<Interface*>(this));

    return 0;
  };
  imp_->onReceiveConnection_ = [this](aris::core::Socket* sock, const char* ip,
                                      int port) -> int {
    ARIS_COUT << "socket receive connection" << std::endl;
    // LOG_INFO << "socket receive connection:\n"
    //	<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip <<
    //"\n"
    //	<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port <<
    // std::endl;
    return 0;
  };
  imp_->onLoseConnection_ = [this](aris::core::Socket* socket) -> int {
    ARIS_COUT << "socket lose connection" << std::endl;
    // LOG_INFO << "socket lose connection" << std::endl;
    for (;;) {
      try {
        socket->startServer(socket->port());
        break;
      } catch (std::runtime_error& e) {
        ARIS_COUT << e.what() << std::endl
                  << "will try to restart server socket in 1s" << std::endl;
        // LOG_ERROR << e.what() << std::endl << "will try to restart server
        // socket in 1s" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
    ARIS_COUT << "socket restart successful" << std::endl;
    // LOG_INFO << "socket restart successful" << std::endl;

    return 0;
  };

  resetSocket(new aris::core::Socket("socket", "", port, type));
  imp_->sock_->setOnReceivedMsg(imp_->onReceiveMsg_);
  imp_->sock_->setOnReceivedConnection(imp_->onReceiveConnection_);
  imp_->sock_->setOnLoseConnection(imp_->onLoseConnection_);
}
// ProgramWebInterface::ProgramWebInterface(ProgramWebInterface && other) =
// default; ProgramWebInterface&
// ProgramWebInterface::operator=(ProgramWebInterface&& other) = default;
ProgramWebInterface::~ProgramWebInterface() = default;
}  // namespace sire::server

//////////////////////////////////////////////////////////////////////////////////////////////////////
#include "mongoose.h"

#ifdef WIN32
#undef min
#undef max
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace sire::server {
template <class K, class V, class dummy_compare, class A>
using my_workaround_fifo_map =
    nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
using my_json = nlohmann::basic_json<my_workaround_fifo_map>;

struct HttpInterface::Imp {
  std::thread http_thread_;
  std::string port_;
  std::string root_dir_;
  struct mg_mgr mgr;
  struct mg_connection* nc;
  struct mg_http_serve_opts s_http_server_opts;
  const char* err_str;

  std::mutex mu_running_;
  std::atomic_bool is_running_{false};

  static void event_handle_for_sire_ui(struct mg_connection* nc, int ev,
                                       void* ev_data, void* fn_data) {
    try {
      struct mg_http_message* hm = (struct mg_http_message*)ev_data;

      switch (ev) {
        case MG_EV_HTTP_MSG: {
          auto method = std::string(hm->method.ptr, hm->method.len);
          auto uri = std::string(hm->uri.ptr, hm->uri.len);

          std::cout << method << "    " << uri << std::endl;

          if (method == "GET" && uri == "/api/config/interface") {
            auto ret = fetchInterfaceConfig();

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "PUT" &&
                     uri.find("/api/dashboards") != std::string::npos) {
            auto ret = updateDashboard(uri.substr(16),
                                       std::string(hm->body.ptr, hm->body.len));

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "POST" &&
                     uri.find("/api/dashboards") != std::string::npos &&
                     uri.find("cells") != std::string::npos) {
            auto dashid = uri.substr(16, uri.substr(16).find_first_of('/'));
            auto ret =
                createCell(dashid, std::string(hm->body.ptr, hm->body.len));

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "DELETE" &&
                     uri.find("/api/dashboards") != std::string::npos &&
                     uri.find("cells") != std::string::npos) {
            auto ret = deleteCell(uri.substr(16, uri.substr(16).find("/cells")),
                                  uri.substr(uri.find("cells/") + 6));

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "GET" && uri == "/api/programs") {
            auto ret = fetchPrograms();

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: text/plain; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "POST" && uri == "/api/programs") {
            auto ret = createProgram(std::string(hm->body.ptr, hm->body.len));

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "PUT" && uri.size() > 13 &&
                     uri.substr(0, 13) == "/api/programs") {
            auto ret = updateProgram(uri.substr(14),
                                     std::string(hm->body.ptr, hm->body.len));

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "DELETE" && uri.size() > 13 &&
                     uri.substr(0, 13) == "/api/programs") {
            auto ret = deleteProgram(uri.substr(14));

            if (ret.empty()) {
              mg_printf(nc,
                        "HTTP/1.1 500 failed create\r\n"
                        "Content-Type: application/json; charset=utf-8\r\n"
                        "Content-Length: %ld\r\n\r\n",
                        ret.size());

              mg_send(nc, ret.c_str(), (int)ret.size());
            } else {
              mg_printf(nc,
                        "HTTP/1.1 200 OK\r\n"
                        "Content-Type: application/json; charset=utf-8\r\n"
                        "Content-Length: %ld\r\n\r\n",
                        ret.size());

              mg_send(nc, ret.c_str(), (int)ret.size());
            }

            break;
          } else if (method == "PATCH" && uri.size() > 13 &&
                     uri.substr(0, 13) == "/api/programs") {
            auto ret = renameProgram(uri.substr(14),
                                     std::string(hm->body.ptr, hm->body.len));

            if (ret.empty()) {
              mg_printf(nc,
                        "HTTP/1.1 500 failed create\r\n"
                        "Content-Type: application/json; charset=utf-8\r\n"
                        "Content-Length: %ld\r\n\r\n",
                        ret.size());

              mg_send(nc, ret.c_str(), (int)ret.size());
            } else {
              mg_printf(nc,
                        "HTTP/1.1 200 OK\r\n"
                        "Content-Type: application/json; charset=utf-8\r\n"
                        "Content-Length: %ld\r\n\r\n",
                        ret.size());

              mg_send(nc, ret.c_str(), (int)ret.size());
            }

            break;
          } else if (method == "GET" && uri == "/api/config/xml") {
            auto ret = fetchConfigXml();

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Accept-Ranges: bytes"
                      "Content-Type: text/xml; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "GET" && uri == "/api/esi/path") {
            auto ret = fetchESIPath();

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "GET" && uri == "/api/obj_picture_list") {
            auto ret = fetchObjPictureList();

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else if (method == "POST" && uri == "/api/obj_picture") {
            // std::cout << std::string(hm->body.p, hm->body.len) << std::endl;

            auto ret = postObjPicture(std::string(hm->body.ptr, hm->body.len));

            mg_printf(nc,
                      "HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json; charset=utf-8\r\n"
                      "Content-Length: %ld\r\n\r\n",
                      ret.size());

            mg_send(nc, ret.c_str(), (int)ret.size());
            break;
          } else {
            mg_http_serve_dir(
                nc, hm,
                &reinterpret_cast<Imp*>(nc->mgr->userdata)->s_http_server_opts);
          }

          break;
        }
        default:
          break;
      }
    } catch (std::exception& e) {
      SIRE_DEBUG_LOG << "http error:" << e.what() << std::endl;
    } catch (...) {
      SIRE_DEBUG_LOG << "http error: unknown" << std::endl;
    }
  }
};
auto HttpInterface::open() -> void {
  std::unique_lock<std::mutex> running_lck(imp_->mu_running_);

  setRootPath(imp_->root_dir_);

  if (imp_->is_running_.exchange(true) == false) {
    mg_mgr_init(&imp_->mgr);
    imp_->nc = mg_http_listen(&imp_->mgr, ("0.0.0.0:" + imp_->port_).c_str(),
                              Imp::event_handle_for_sire_ui, NULL);
    if (imp_->nc == NULL) {
      fprintf(stderr, "Error starting server on http\n");
      exit(1);
    }
    std::memset(&imp_->s_http_server_opts, 0, sizeof(mg_http_serve_opts));
    imp_->s_http_server_opts.root_dir = imp_->root_dir_.c_str();
    imp_->mgr.userdata = imp_.get();
    imp_->http_thread_ = std::thread([this]() {
      for (; imp_->is_running_;) {
        mg_mgr_poll(&imp_->mgr, 1000);
      }
      mg_mgr_free(&imp_->mgr);
    });
  }
}
auto HttpInterface::close() -> void {
  std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
  if (imp_->is_running_.exchange(false) == true) {
    imp_->http_thread_.join();
  }
}
auto HttpInterface::isConnected() const -> bool { return true; }
auto HttpInterface::documentRoot() -> std::string { return imp_->root_dir_; }
auto HttpInterface::port() -> std::string { return imp_->port_; }
auto HttpInterface::setPort(const std::string& port) -> void {
  imp_->port_ = port;
}
auto HttpInterface::setDocumentRoot(const std::string& root) -> void {
  imp_->root_dir_ = root;
}
HttpInterface::~HttpInterface() = default;
HttpInterface::HttpInterface(const std::string& name, const std::string& port,
                             const std::string& document_root)
    : Interface(name), imp_(new Imp) {
  imp_->root_dir_ = document_root;
  imp_->port_ = port;
}

ARIS_REGISTRATION {
  aris::core::class_<ProgramWebInterface>("SireProgramWebInterface")
      .inherit<aris::server::Interface>()
      .prop("socket", &ProgramWebInterface::resetSocket,
            &ProgramWebInterface::socket);

  aris::core::class_<HttpInterface>("SireHttpInterface")
      .inherit<aris::server::Interface>()
      .prop("document_root", &HttpInterface::setDocumentRoot,
            &HttpInterface::documentRoot)
      .prop("port", &HttpInterface::setPort, &HttpInterface::port);
}
}  // namespace sire::server