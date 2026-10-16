class BaseKinematics:

    def joints_to_transformations(self, joints):
        raise NotImplementedError

    def joints_to_tcp(self, joints):
        raise NotImplementedError

    def joints_to_pose(self, joints):
        raise NotImplementedError

    def tcp_to_joints(self, T06, prev_joints, shoulder_pref=None, elbow_pref=None, wrist_pref=None):
        raise NotImplementedError

    def pose_to_joints(self, pose, prev_joints, shoulder_pref=None, elbow_pref=None, wrist_pref=None):
        raise NotImplementedError

    def jacobian(self, joints):
        raise NotImplementedError
