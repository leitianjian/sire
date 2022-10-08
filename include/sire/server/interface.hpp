#ifndef SIRE_SERVER_INTERFACE_H_
#define SIRE_SERVER_INTERFACE_H_

#include <sire_lib_export.h>
#include <aris/core/core.hpp>
#include <aris/core/object.hpp>
#include <aris/server/interface.hpp>
#include <future>
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace sire::server {
class SIRE_API ProgramWebInterface : public aris::server::Interface {
 public:
  auto virtual open() -> void override;
  auto virtual close() -> void override;
  auto virtual isConnected() const -> bool override;
  auto resetSocket(aris::core::Socket* sock) -> void;
  auto socket() -> aris::core::Socket&;
  auto isAutoMode() -> bool;
  auto isAutoRunning() -> bool;
  auto isAutoPaused() -> bool;
  auto isAutoStopped() -> bool;
  auto lastError() -> std::string;
  auto lastErrorCode() -> int;
  auto lastErrorLine() -> int;
  auto currentFileLine() -> std::tuple<std::string, int>;

  ~ProgramWebInterface();
  ProgramWebInterface(
      const std::string& name = "pro_interface",
      const std::string& port = "5866",
      aris::core::Socket::Type type = aris::core::Socket::Type::WEB);
  ARIS_DELETE_BIG_FOUR(ProgramWebInterface)

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
auto SIRE_API parse_ret_value(
    std::vector<std::pair<std::string, std::any>>& ret) -> std::string;
class SIRE_API HttpInterface : public aris::server::Interface {
 public:
  auto virtual open() -> void override;
  auto virtual close() -> void override;
  auto virtual isConnected() const -> bool override;
  auto documentRoot() -> std::string;
  auto setDocumentRoot(const std::string& root) -> void;
  auto port() -> std::string;
  auto setPort(const std::string& root) -> void;

  virtual ~HttpInterface();
  HttpInterface(const std::string& name = "http_interface",
                const std::string& port = "8000",
                const std::string& document_root = "./");
  ARIS_DELETE_BIG_FOUR(HttpInterface)

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::server

#endif