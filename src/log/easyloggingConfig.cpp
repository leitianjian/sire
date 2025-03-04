#include "easylogging++.h"
namespace sire {
namespace log {
struct LogConfig {
  LogConfig() {
    el::Configurations defaultConf;
    defaultConf.setToDefault();
    // Values are always std::string
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Format,
                    "%datetime %fbase:%line %msg");
    // default logger uses default configurations
    el::Loggers::reconfigureLogger("default", defaultConf);
  }
};
static const LogConfig config;
}  // namespace log
}  // namespace sire
