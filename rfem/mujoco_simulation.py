import mujoco as mj
from mujoco import viewer

class MujocoSimulation:
    def __init__(self, urdf_path):
        self.model = mj.MjModel.from_xml_path(urdf_path)
        self.data = mj.MjData(self.model)

    def run(self):
        # 윈도우 위치 관련 설정 추가 (Wayland 호환성 개선)
        with viewer.launch(self.model, self.data) as v:
            while self.data.time < 10:
                mj.mj_step(self.model, self.data)
                v.sync()
