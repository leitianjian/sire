#include <filesystem>
#include <iostream>

#include <aris.hpp>

#include "sire/core/sire_log.hpp"

auto xmlpath = std::filesystem::absolute(".");  // 获取当前工程所在的路径
const std::string xmlfile = "sire_three_part_fall.xml";
int main(int argc, char* argv[]) {
  auto& cs = aris::server::ControlServer::instance();
  xmlpath = xmlpath / xmlfile;
  aris::core::fromXmlFile(cs, xmlpath);
  cs.init();
  // Start Web Socket
  cs.open();
  // Receive Command
  cs.runCmdLine();
  return 0;
}