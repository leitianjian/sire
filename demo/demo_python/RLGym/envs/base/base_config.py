class BaseConfig:
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
