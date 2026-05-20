class BaseConfig:
    """
    强化学习（RL）环境的基础配置类。
    它会自动初始化嵌套的配置子类（比如 env, terrain, control 分类），
    这样我们就可以非常方便地通过点号来访问参数（例如 config.env.num_envs 表示并行环境的数量）。
    """
    def __init__(self) -> None:
        self.init_member_classes(self)

    @staticmethod
    def init_member_classes(obj) -> None:
        for key in dir(obj):
            if key.startswith("__"):
                continue
            val = getattr(obj, key)
            if isinstance(val, type):
                instance = val()
                setattr(obj, key, instance)
                BaseConfig.init_member_classes(instance)
