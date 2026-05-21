import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# CSV 파일 읽기
df = pd.read_csv("joint_and_body_positions.csv")

# rfe_6 위치 데이터 추출
x = df["rfe_6_x"]
y = df["rfe_6_y"]
z = df["rfe_6_z"]

# 3D 그래프 그리기
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='rfe_6 trajectory', color='red')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("rfe_6 링크의 궤적")
ax.legend()
plt.show()
