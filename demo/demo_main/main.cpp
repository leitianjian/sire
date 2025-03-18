#include <filesystem>
#include <iostream>

#include <aris.hpp>

#include "sire/core/sire_log.hpp"

// auto xmlpath = std::filesystem::absolute(".");  // 获取当前工程所在的路径
// const std::string xmlfile = "config.xml";
// int main(int argc, char* argv[]) {
//   auto& cs = aris::server::ControlServer::instance();
//   xmlpath = xmlpath / xmlfile;
//   aris::core::fromXmlFile(cs, xmlpath);
//   cs.init();
//   // Start Web Socket
//   cs.open();
//   // Receive Command
//   cs.runCmdLine();
//   return 0;
// }
// Copyright 2022 Arthur Sonzogni. All rights reserved.
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.
#include <cstdlib>   // for system, EXIT_SUCCESS
#include <iostream>  // for operator<<, basic_ostream, basic_ostream::operator<<, cout, endl, flush, ostream, basic_ostream<>::__ostream_type, cin
#include <memory>    // for shared_ptr, __shared_ptr_access, allocator
#include <string>    // for getline, string

#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"  // for Button, Horizontal, Renderer
#include "ftxui/component/component_base.hpp"      // for ComponentBase
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for operator|, filler, Element, borderEmpty, hbox, size, paragraph, vbox, LESS_THAN, border, center, HEIGHT, WIDTH

int main() {
  using namespace ftxui;

  auto screen = ScreenInteractive::Fullscreen();

  // When pressing this button, "screen.WithRestoredIO" will execute the
  // temporarily uninstall the terminal hook and execute the provided callback
  // function. This allow running the application in a non-interactive mode.
  auto& cs = aris::server::ControlServer::instance();
  std::string configPath;
  bool csLoaded{false};
  auto btn_load = Button("Load Config", [&csLoaded, &cs, &configPath] {
    if (csLoaded || configPath.size() == 0) {
      return;
    }
    if (configPath.front() == '"') {
      configPath.erase(0, 1);                   // erase the first character
      configPath.erase(configPath.size() - 1);  // erase the last character
    }
    auto xmlpath = std::filesystem::path(configPath);  // 获取当前工程所在的路径
    aris::core::fromXmlFile(cs, xmlpath);
    cs.init();
    csLoaded = true;
  });
  auto btn_run = Button("Run CmdLine", screen.WithRestoredIO([&cs] {
    // Start Web Socket
    cs.open();
    // Receive Command
    cs.runCmdLine();
  }));
  auto btn_quit = Button("Quit", screen.ExitLoopClosure());
  Component config_path =
      Input(&configPath, "Path to Config file (Drag and drop is ok)");
  auto layout =
      Container::Vertical({config_path, Container::Horizontal({
                                            btn_load,
                                            btn_run | Maybe(&csLoaded),
                                            btn_quit,
                                        })});

  auto renderer = Renderer(layout, [&] {
    auto explanation = paragraph("Please select configuration file");
    auto element = vbox({
        explanation | borderEmpty,
        hbox(text(" Path : "), config_path->Render()),
        hbox({
            btn_load->Render(),
            btn_run->Render(),
            filler(),
            btn_quit->Render(),
        }),
    });

    element = element | borderEmpty | border | size(WIDTH, LESS_THAN, 80) |
              size(HEIGHT, LESS_THAN, 20) | center;
    return element;
  });

  screen.Loop(renderer);
  return EXIT_SUCCESS;
}