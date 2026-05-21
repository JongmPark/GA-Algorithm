import os
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
import mujoco

# 1) XML 수정 함수
def modify_xml_damping_stiffness(input_xml, output_xml, damping, stiffness):
    tree = ET.parse(input_xml)
    root = tree.getroot()
    for joint in root.iter('joint'):
        joint.set('damping', str(damping))
        joint.set('stiffness', str(stiffness))
    tree.write(output_xml)

# 2) 시뮬레이션 및 CSV 저장 함수
def simulate_and_save(xml_path, csv_path, steps=1000):
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    joint_names = ["active_joint", "sde_joint_1", "sde_joint_2",
                   "sde_joint_3", "sde_joint_4", "sde_joint_5"]
    body_names  = ["rfe_1", "rfe_2", "rfe_3",
                   "rfe_4", "rfe_5", "rfe_6"]

    # CSV 헤더 구성
    header = ["step"] + [f"{j}_pos" for j in joint_names] \
                   + [f"{b}_{ax}" for b in body_names for ax in ('x','y','z')]
    records = []

    for step in range(steps):
        mujoco.mj_step(model, data)
        row = [step]
        # joint positions
        for j in joint_names:
            idx = model.joint(j).qposadr
            row.append(data.qpos[idx])
        # body positions
        for b in body_names:
            bid = model.body(b).id
            row.extend(data.xpos[bid].tolist())
        records.append(row)

    df = pd.DataFrame(records, columns=header)
    df.to_csv(csv_path, index=False)

# 3) 메인 실행: 랜덤 D,K 100개 생성 및 시뮬레이션
def main():
    
    D_RANGE    = (0.05, 1.0)
    K_RANGE    = (1.0, 20.0)
    N_SAMPLES  = 100
    BASE_XML   = "[RFEM] pendulum_5.xml"
    OUTPUT_DIR = "simulations"
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for _ in range(N_SAMPLES):
        d = round(np.random.uniform(*D_RANGE), 5)
        k = round(np.random.uniform(*K_RANGE), 5)
        xml_out = os.path.join(OUTPUT_DIR, f"pendulum_d{d}_k{k}.xml")
        csv_out = os.path.join(OUTPUT_DIR, f"[D:{d}],[K:{k}].csv")
        modify_xml_damping_stiffness(BASE_XML, xml_out, d, k)
        simulate_and_save(xml_out, csv_out)
        ## print(f"Completed D={d}, K={k}")

if __name__ == "__main__":
    main()
