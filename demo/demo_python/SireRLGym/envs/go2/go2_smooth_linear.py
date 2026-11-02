"""Old linear smooth law in the same V5 pipeline as the frozen baseline."""
from SireRLGym.envs.go2.go2_config import GO2RoughCfg
from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire


class GO2SmoothLinearCfg(GO2RoughCfg):
    class sim(GO2RoughCfg.sim):
        sire_contact_solver = "smooth_linear_shared_v5"


class LeggedRobotSmoothLinear(LeggedRobotSire):
    def _apply_sire_domain_randomization(self, sim, model):
        engine = sim.physicsEngine()
        previous = engine.contactSolver()
        mode = previous.contactModelMode
        time_method = previous.contact_time_method
        initial_policy = previous.initial_penetration_policy
        solver = engine.addSmoothLinearContactSolver()
        solver.setContactModelMode(mode)
        solver.setContactTimeMethod(time_method)
        solver.setInitialPenetrationPolicy(initial_policy)
        result = super()._apply_sire_domain_randomization(sim, model)
        if type(solver).__name__ != "SmoothLinearContactSolver":
            raise RuntimeError("The shared-pipeline smooth solver was not installed")
        return result
