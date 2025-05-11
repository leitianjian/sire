#include "sire/plan/sim_reset.hpp"

#include <random>

#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
struct SimReset::Imp {
  uint32_t frame_skip_{25};
};
auto SimReset::prepareNrt() -> void {
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  auto& middleware = dynamic_cast<sire::middleware::SireMiddleware&>(
      controlServer()->middleWare());
  auto& simulator = middleware.simulationLoop();
  simulator.pause();
  auto& physicsEngine = middleware.physicsEngine();
  physicsEngine.resetInitialModel();
  physicsEngine.initPartContactForce2Model();
  middleware.simulatorModules().init(&middleware);
  simulator.reset();
  model()->init();
  return;
}
// default deconstructor
SimReset::~SimReset() = default;
SimReset::SimReset(const std::string& name) : imp_(new Imp) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"sim_reset\">"
                            "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(SimReset);

ARIS_REGISTRATION {
  aris::core::class_<SimReset>("SimReset").inherit<aris::plan::Plan>();
}
}  // namespace sire::plan