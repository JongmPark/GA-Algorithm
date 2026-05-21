import os
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
import mujoco
import datetime
import matplotlib.pyplot as plt
from concurrent.futures import ProcessPoolExecutor

# ──────────────────────────────────────────────
# 1) XML 수정: 각 조인트별 damping, stiffness 적용
def modify_xml_joint_params(input_xml, output_xml, d_values, k_values):
    try:
        tree = ET.parse(input_xml)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"❌ XML 파싱 오류 발생: {e} → 파일: {input_xml}")
        return False
    joints = list(root.iter('joint'))
    if len(joints) != len(d_values):
        print(f"❌ 조인트 개수 불일치: XML={len(joints)}, d_values={len(d_values)}")
        return False
    for i, joint in enumerate(joints):
        joint.set('damping', str(d_values[i]))
        joint.set('stiffness', str(k_values[i]))
    try:
        tree.write(output_xml)
        if not os.path.exists(output_xml):
            print(f"❌ XML 생성 실패: {output_xml}")
            return False
    except Exception as e:
        print(f"❌ XML 생성 중 예외 발생: {e}")
        return False
    return True

# ──────────────────────────────────────────────
# 2) 시뮬레이션 및 CSV 저장 (공식 mujoco API 이용)
def simulate_and_save(xml_path, csv_path, steps=1000):
    try:
        if not os.path.exists(xml_path):
            print(f"❌ 시뮬레이션 실패: XML 파일이 없습니다 → {xml_path}")
            return False
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        joint_names = ["active_joint", "sde_joint_1", "sde_joint_2", "sde_joint_3", "sde_joint_4", "sde_joint_5"]
        body_names = ["rfe_1", "rfe_2", "rfe_3", "rfe_4", "rfe_5", "rfe_6"]
        header = ["step"] + [f"{j}_pos" for j in joint_names] + [f"{b}_{ax}" for b in body_names for ax in ('x', 'y', 'z')]
        records = []
        for step in range(steps):
            mujoco.mj_step(model, data)
            if (np.isnan(data.qpos).any() or np.isinf(data.qpos).any() or
                np.isnan(data.xpos).any() or np.isinf(data.xpos).any()):
                print(f"❌ [ERROR] NaN 또는 Inf 발생 → {xml_path}")
                return False
            row = [step]
            for j in joint_names:
                idx = model.joint(j).qposadr
                row.append(data.qpos[idx])
            for b in body_names:
                bid = model.body(b).id
                row.extend(data.xpos[bid].tolist())
            records.append(row)
        if len(records) > 0:
            pd.DataFrame(records, columns=header).to_csv(csv_path, index=False)
            return True
        else:
            print(f"❌ 기록된 시뮬레이션 데이터가 없습니다: {csv_path}")
            return False
    except Exception as e:
        print(f"❌ 시뮬레이션 실패: {xml_path} → 이유: {e}")
        return False

# ──────────────────────────────────────────────
# 3) RMSE 계산 함수 (모든 링크 위치 반영)
def compute_rmse_full(gt_csv, sim_csv):
    gt = pd.read_csv(gt_csv)
    sim = pd.read_csv(sim_csv)
    body_cols = [c for c in gt.columns if c.startswith("rfe_")]
    gt_vals = gt[body_cols].values
    sim_vals = sim[body_cols].values
    n = min(len(gt_vals), len(sim_vals))
    diffs = gt_vals[:n] - sim_vals[:n]
    if np.isnan(diffs).any() or np.isinf(diffs).any():
        print(f"❌ NaN 또는 Inf 발견: {sim_csv}")
        return np.inf
    frame_err = np.linalg.norm(diffs, axis=1)
    return np.sqrt(np.mean(frame_err**2))

# ──────────────────────────────────────────────
# 4) population 전체 평가 (캐싱 포함)
def evaluate_population_fitness(gt_csv, sim_folder, population, cache, n_joints):
    fitness = []
    for ind in population:
        d_values = np.round(ind[:n_joints], 5)
        k_values = np.round(ind[n_joints:], 5)
        d_str = "_".join([f"{v:.3f}" for v in d_values])
        k_str = "_".join([f"{v:.3f}" for v in k_values])
        csv_path = os.path.join(sim_folder, f"result_d{d_str}_k{k_str}.csv")
        key = (tuple(d_values), tuple(k_values))
        if key in cache:
            fitness.append(cache[key])
            continue
        if not os.path.exists(csv_path):
            fitness.append(np.inf)
            cache[key] = np.inf
            continue
        try:
            rmse = compute_rmse_full(gt_csv, csv_path)
            fitness.append(rmse)
            cache[key] = rmse
        except Exception as e:
            fitness.append(np.inf)
            cache[key] = np.inf
    return np.array(fitness)

# ──────────────────────────────────────────────
# 5) GA 유틸리티 (BLX-α 교차, 적응적 변이율/교차율)
def initialize_population(size, d_ranges, k_ranges):
    pop = np.empty((size, len(d_ranges) + len(k_ranges)))
    for i in range(len(d_ranges)):
        pop[:,i] = np.random.uniform(*d_ranges[i], size=size)
    for i in range(len(k_ranges)):
        pop[:,i+len(d_ranges)] = np.random.uniform(*k_ranges[i], size=size)
    return pop

def select_parents(pop, fitness, n_parents):
    idx = np.argsort(fitness)
    return pop[idx[:n_parents]]

def blx_alpha_crossover(parents, n_offspring, alpha=0.5, cross_rate=1.0):
    offspring = []
    for _ in range(n_offspring):
        if np.random.rand() < cross_rate:
            p1, p2 = parents[np.random.choice(len(parents), 2, replace=False)]
            child = np.empty_like(p1)
            for i in range(len(p1)):
                min_val = min(p1[i], p2[i]) - alpha * abs(p1[i] - p2[i])
                max_val = max(p1[i], p2[i]) + alpha * abs(p1[i] - p2[i])
                child[i] = np.random.uniform(min_val, max_val)
            offspring.append(child)
        else:
            # 교차 안 하고 부모 중 하나 복사
            offspring.append(parents[np.random.choice(len(parents))].copy())
    return np.array(offspring)

def mutate(offspring, d_ranges, k_ranges, mut_rate=0.3, gen=0, n_gen=15):
    n_d = len(d_ranges)
    n_k = len(k_ranges)
    for child in offspring:
        for i in range(n_d):
            if np.random.rand() < mut_rate:
                child[i] += np.random.normal(0, 0.02)
                child[i] = np.clip(child[i], *d_ranges[i])
        for i in range(n_k):
            if np.random.rand() < mut_rate:
                child[i+n_d] += np.random.normal(0, 0.5)
                child[i+n_d] = np.clip(child[i+n_d], *k_ranges[i])
    return offspring

# ──────────────────────────────────────────────
# 6) 시뮬레이션 병렬 실행 함수
def run_simulation_task(args):
    d_values, k_values, base_xml, xml_out, csv_out, steps = args
    success = modify_xml_joint_params(base_xml, xml_out, d_values, k_values)
    if success:
        return simulate_and_save(xml_out, csv_out, steps)
    return False

# ──────────────────────────────────────────────
# 7) GA 메인 루프 (적응적 연산자 확률 적용)
def main():
    #np.random.seed(42)
    N_JOINTS = 6
    POP_SIZE = 50
    N_PARENTS = 12
    INIT_MUT_RATE = 0.4
    INIT_CROSS_RATE = 0.8
    N_GENERATION = 20
    D_RANGES = [(0.01, 1.0)] * N_JOINTS
    K_RANGES = [(1.0, 20.0)] * N_JOINTS
    STEPS = 1000
    GT_CSV = "joint_and_body_positions.csv"
    BASE_XML = "[FINAL]pendulum_5.xml"
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    RESULTS_DIR = f"ga_sims_{timestamp}"
    os.makedirs(RESULTS_DIR)
    population = initialize_population(POP_SIZE, D_RANGES, K_RANGES)
    cache = {}
    best_fitness_list = []
    avg_fitness_list = []

    for gen in range(N_GENERATION):
        # 적응적 확률 계산 (세대 진행에 따라 선형 감소)
        mut_rate = INIT_MUT_RATE * (1 - gen / N_GENERATION)
        cross_rate = INIT_CROSS_RATE * (1 - gen / N_GENERATION)

        print(f"\n=== Generation {gen} === (mut_rate={mut_rate:.3f}, cross_rate={cross_rate:.3f})")
        gen_folder = os.path.join(RESULTS_DIR, f"gen_{gen}")
        os.makedirs(gen_folder)
        sim_args = []
        for ind in population:
            d_values = np.round(ind[:N_JOINTS], 5)
            k_values = np.round(ind[N_JOINTS:], 5)
            d_str = "_".join([f"{v:.3f}" for v in d_values])
            k_str = "_".join([f"{v:.3f}" for v in k_values])
            xml_out = os.path.join(gen_folder, f"pendulum_d{d_str}_k{k_str}.xml")
            csv_out = os.path.join(gen_folder, f"result_d{d_str}_k{k_str}.csv")
            sim_args.append((d_values, k_values, BASE_XML, xml_out, csv_out, STEPS))
        with ProcessPoolExecutor() as executor:
            list(executor.map(run_simulation_task, sim_args))
        fitness = evaluate_population_fitness(GT_CSV, gen_folder, population, cache, N_JOINTS)
        best_idx = np.nanargmin(fitness)
        print(f" Best: d={population[best_idx,:N_JOINTS]}, k={population[best_idx,N_JOINTS:]}, RMSE={fitness[best_idx]:.5f}")
        best_fitness_list.append(fitness[best_idx])
        avg_fitness_list.append(np.nanmean(fitness))
        parents = select_parents(population, fitness, N_PARENTS)
        offspring = blx_alpha_crossover(parents, POP_SIZE - N_PARENTS, alpha=0.5, cross_rate=cross_rate)
        offspring = mutate(offspring, D_RANGES, K_RANGES, mut_rate, gen, N_GENERATION)
        population = np.vstack((parents, offspring))

    # 결과 시각화
    plt.figure(figsize=(8,5))
    plt.plot(best_fitness_list, label='Best RMSE')
    plt.plot(avg_fitness_list, label='Average RMSE')
    plt.xlabel('Generation')
    plt.ylabel('RMSE')
    plt.title('GA Convergence Curve')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.xticks(range(len(best_fitness_list)))
    plt.savefig(os.path.join(RESULTS_DIR, "convergence_curve.png"))
    plt.show()

    best_idx = np.nanargmin(fitness)
    print("\n=== 최종 최적 파라미터 ===")
    print(f" Damping:   {population[best_idx,:N_JOINTS]}")
    print(f" Stiffness: {population[best_idx,N_JOINTS:]}")
    print(f" RMSE:      {fitness[best_idx]:.5f}")

if __name__ == "__main__":
    main()

