import mujoco

LEGGED_GYM_ROOT_DIR = "D:/code/sire/demo/demo_python/mujocoDogRL"


if __name__ == "__main__":
    # Load robot model
    m = mujoco.MjModel.from_xml_path(f"{LEGGED_GYM_ROOT_DIR}/go2_test.xml")
    d = mujoco.MjData(m)
    
    # 添加调试代码到deploy_dog.py中
    print("Model info:")
    print(f"Number of joints: {m.njnt}")
    print(f"Joint names: {[m.joint(i).name for i in range(m.njnt)]}")
    print(f"Number of actuators: {m.nu}")
    print(f"Actuator names: {[m.actuator(i).name for i in range(m.nu)]}")
    # load policy
    # print(d.qacc)
    print(d.qacc[6:])
    d.ctrl[:] = [2.13250872,  6.34638789, -5.04130703, -2.13250872,  6.34638789, -5.04130703, 1.95357383,  9.85797647, -3.97620019, -1.95357383,  9.85797647, -3.97620019]
    mujoco.mj_forward(m, d)
    print(d.ctrl, d.qacc[6:])
    # print(d.ctrl, d.qacc)