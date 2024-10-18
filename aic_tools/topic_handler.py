from abc import ABC, abstractmethod

from tf_transformations import euler_from_quaternion


class TopicHandlerRegistry:
    _registry = {}
    _active_handlers = set()

    @classmethod
    def register(cls, handler_class):
        topic_name = handler_class.TOPIC_NAME
        cls._registry[topic_name] = handler_class

    @classmethod
    def activate_handlers(cls, handler_classes):
        cls._active_handlers = {
            cls._registry[handler.TOPIC_NAME] for handler in handler_classes
        }

    @classmethod
    def get_handler(cls, topic):
        handler_class = cls._registry.get(topic, None)
        if handler_class and handler_class in cls._active_handlers:
            return handler_class()
        return None

    @classmethod
    def get_active_handlers(cls):
        return cls._active_handlers

    @classmethod
    def get_active_topics(cls):
        # アクティブなハンドラのトピック名をリストで返す
        return [handler_class.TOPIC_NAME for handler_class in cls._active_handlers]

    @classmethod
    def get_all_topics(cls):
        # 登録されているすべてのトピック名をリストで返す
        return list(cls._registry.keys())


class TopicHandler(ABC):
    TOPIC_NAME = None  # 各クラスで定義

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if cls.TOPIC_NAME is not None:
            TopicHandlerRegistry.register(cls)

    @abstractmethod
    def extract_data(self, msg):
        pass

    @classmethod
    def get_topic_name(cls):
        return cls.TOPIC_NAME

    @classmethod
    def get_under_scored_topic_name(cls):
        return '_'.join(cls.TOPIC_NAME.strip('/').split('/'))

    def process_message(self, msg, timestamp):
        # 共通で timestamp を含める
        data = self.extract_data(msg)
        data["stamp"] = timestamp
        return data


class LocalizationHandler(TopicHandler):
    TOPIC_NAME = "/localization/kinematic_state"

    def extract_data(self, msg):
        e = euler_from_quaternion(
            (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
        )
        return {
            "ekf_x": msg.pose.pose.position.x,
            "ekf_y": msg.pose.pose.position.y,
            "ekf_yaw": e[2],
        }


class VelocityStatusHandler(TopicHandler):
    TOPIC_NAME = "/vehicle/status/velocity_status"

    def extract_data(self, msg):
        return {"vx": msg.longitudinal_velocity, "vy": msg.lateral_velocity}


class SteeringStatusHandler(TopicHandler):
    TOPIC_NAME = "/vehicle/status/steering_status"

    def extract_data(self, msg):
        return {"steer": msg.steering_tire_angle}


class ImuDataHandler(TopicHandler):
    TOPIC_NAME = "/sensing/imu/imu_data"

    def extract_data(self, msg):
        return {"gyro_z": msg.angular_velocity.z}


class GnssPoseHandler(TopicHandler):
    TOPIC_NAME = "/sensing/gnss/pose"

    def extract_data(self, msg):
        return {
            "gnss_x": msg.pose.position.x,
            "gnss_y": msg.pose.position.y,
        }
