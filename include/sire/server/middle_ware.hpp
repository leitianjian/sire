#ifndef SIRE_SERVER_MIDDLE_WARE_HPP_
#define SIRE_SERVER_MIDDLE_WARE_HPP_

#include "sire/server/interface.hpp"
#include <aris/server/interface.hpp>
#include <aris/server/middle_ware.hpp>
#include <sire_lib_export.h>
#include <functional>
#include <string_view>

namespace sire::server {
class SIRE_API ProgramMiddleware : public aris::server::MiddleWare {
 public:
  auto isAutoMode() -> bool;
  auto isAutoRunning() -> bool;
  auto isAutoPaused() -> bool;
  auto isAutoStopped() -> bool;
  auto lastError() -> std::string;
  auto lastErrorCode() -> int;
  auto lastErrorLine() -> int;
  auto currentFileLine() -> std::tuple<std::string, int>;
  auto executeCmd(std::string_view str,
                  std::function<void(std::string)> send_ret,
                  aris::server::Interface* interface) -> int override;
  ~ProgramMiddleware();

  ProgramMiddleware();
  ProgramMiddleware(ProgramMiddleware&& other);
  ProgramMiddleware& operator=(ProgramMiddleware&& other);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};

}  // namespace aris::server

#endif  // ARIS_SERVER_MIDDLE_WARE_HPP_