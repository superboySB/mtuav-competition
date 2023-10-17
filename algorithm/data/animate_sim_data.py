import os
import imageio
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


path = "/workspace/mtuav-competition/algorithm/data"
frame_directory = "sub_data"

frame_directory_path = os.path.join(path, frame_directory)
if not os.path.exists(frame_directory_path):
    os.makedirs(frame_directory_path)
else:
    # 清空目录中的文件
    for filename in os.listdir(frame_directory_path):
        file_path = os.path.join(frame_directory_path, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(f'Failed to delete {file_path}. Reason: {e}')


sim_data = np.loadtxt(path + "/sim_data.txt")
sim_info = np.loadtxt(path + "/sim_info.txt")

dt = float(sim_info[0][2])
num_obs = int(sim_info[0][1])
num_drone = int(sim_info[0][0])
dim_drone = sim_info[1]

a_drone, b_drone, c_drone = dim_drone

init_drone = sim_info[2:2+num_drone]
goal_drone = sim_info[2+num_drone:2+2*num_drone]

pos_obs = sim_info[2+2*num_drone:2+2*num_drone+num_obs]
dim_obs = sim_info[2+2*num_drone+num_obs:]

x_obs = pos_obs[:,0]
y_obs = pos_obs[:,1]
z_obs = pos_obs[:,2]

a_obs = dim_obs[:,0]
b_obs = dim_obs[:,1]
c_obs = dim_obs[:,2]

x_lim = [-2, 2]
y_lim = [-2, 2]
z_lim = [+0.2, 2.2]



fig = plt.figure(0)
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Trajectory')
ax.set_xlabel('x in m')
ax.set_ylabel('y in m')
ax.set_zlabel('z in m')
ax.view_init(elev=60, azim=-77)

ax.set_xlim(x_lim[0], x_lim[1])
ax.set_ylim(y_lim[0], y_lim[1])
ax.set_zlim(z_lim[0], z_lim[1])


# fig2 = plt.figure(1)
# ax2 = fig2.add_subplot(111)
# ax2.set_title("Acceleration in g m/s^2")
# ax2.set_xlabel("Time index")
# ax2.set_ylabel("Value in x g")

# mng = plt.get_current_fig_manager()
# mng.full_screen_toggle()

phi_obs = np.linspace(0,2*np.pi, 10).reshape(10, 1) 
theta_obs = np.linspace(0, np.pi/2, 10).reshape(-1, 10) 

phi_drone = np.linspace(0,2*np.pi, 10).reshape(10, 1) 
theta_drone = np.linspace(0, np.pi, 10).reshape(-1, 10) 

colors=(np.random.choice(range(255),size=[num_drone, 3]))/255.0

if num_obs > 0:    
    for k in range(len(x_obs)):
        if c_obs[k] > z_lim[1]:
            z = np.linspace(z_obs[k], 2.0, 15)
            theta = np.linspace(0, 2*np.pi, 15)
            theta_obs, z_ell_obs=np.meshgrid(theta, z)
        
            x_ell_obs = x_obs[k] + a_obs[k] * np.cos(theta_obs)
            y_ell_obs = y_obs[k] + b_obs[k] * np.sin(theta_obs)
        else:
            x_ell_obs = x_obs[k] + a_obs[k] * np.sin(theta_obs)*np.cos(phi_obs)
            y_ell_obs = y_obs[k] + b_obs[k] * np.sin(theta_obs)*np.sin(phi_obs)
            z_ell_obs = z_obs[k] + c_obs[k] * np.cos(theta_obs)
        ax.plot_surface(x_ell_obs, y_ell_obs, z_ell_obs,  rstride=10, cstride=2, color='#2980b9', alpha=0.4)
        
    ax.plot([x_lim[0], x_lim[0], x_lim[1], x_lim[1], x_lim[0] ], [y_lim[0], y_lim[1], y_lim[1], y_lim[0], y_lim[0]], color='red', alpha=0.1)
    ax.plot([x_lim[0], x_lim[0], x_lim[1], x_lim[1], x_lim[0] ], [y_lim[0], y_lim[1], y_lim[1], y_lim[0], y_lim[0]], [z_lim[1], z_lim[1], z_lim[1], z_lim[1], z_lim[1]],color='red', alpha=0.1)
    ax.plot([x_lim[0], x_lim[0]], [y_lim[0], y_lim[0]], [z_lim[0], z_lim[1]], color='red', alpha=0.6)
    ax.plot([x_lim[0], x_lim[0]], [y_lim[1], y_lim[1]], [z_lim[0], z_lim[1]], color='red', alpha=0.6)
    ax.plot([x_lim[1], x_lim[1]], [y_lim[1], y_lim[1]], [z_lim[0], z_lim[1]], color='red', alpha=0.6)
    ax.plot([x_lim[1], x_lim[1]], [y_lim[0], y_lim[0]], [z_lim[0], z_lim[1]], color='red', alpha=0.6)

collision_count_agent = 0
collision_count_obs = 0
sim_steps = int(len(sim_data)/num_drone/3)


for i in range(sim_steps):
    body = []
    predictions = []

    x_data = sim_data[3*i*num_drone + 0: 3*i*num_drone + num_drone]
    y_data = sim_data[3*i*num_drone + num_drone:3*i*num_drone + 2*num_drone]
    z_data = sim_data[3*i*num_drone + 2*num_drone: 3*i*num_drone + 3*num_drone]
    
    for k in range(0, num_drone):
        x_ell_drone = x_data[k, 0] + a_drone * np.sin(theta_drone)*np.cos(phi_drone)
        y_ell_drone = y_data[k, 0] + b_drone * np.sin(theta_drone)*np.sin(phi_drone)
        z_ell_drone = z_data[k, 0] + c_drone * np.cos(theta_drone)
    
        body_temp = ax.plot_surface(x_ell_drone, y_ell_drone, z_ell_drone,  rstride=4, cstride=6, color = colors[k], alpha=0.6)
        body.append(body_temp)

        predictions_temp, = ax.plot(x_data[k], y_data[k], z_data[k], color=colors[k], linestyle='--')
        predictions.append(predictions_temp) 

    for m in range(0,num_drone):
        x_check = x_data[m, 0]
        y_check = y_data[m, 0]
        z_check = z_data[m, 0]
        for n in range(0,num_drone):
            if m == n:
                continue
            val = (x_check - x_data[n, 0])**2/(2*a_drone)**2 + (y_check - y_data[n, 0])**2/(2*b_drone)**2 + (z_check - z_data[n, 0])**2/(2*c_drone)**2 
            if val < 1:
                collision_count_agent += 1 
        if num_obs > 0:        
            for n in range(0,len(a_obs)):  
                val = (x_check - x_obs[n])**2/(a_drone+a_obs[n])**2 + (y_check - y_obs[n])**2/(b_drone+b_obs[n])**2 + ((z_check - z_obs[n])**2/(c_drone+c_obs[n])**2)
                if val < 1:
                    collision_count_obs += 1 
                    print(x_check, y_check, z_check, x_obs[n], y_obs[n], z_obs[n], a_obs[n], b_obs[n], c_obs[n])        
                    print(val)
            
    stats = ax.text(0.2, y_lim[1]+0.2, z_lim[1]+0.2,'Obstacle Collision Count = {obs}\nInter-Agent Collision Count = {col}\nAgents = {n}\nSim-Step = {step}'.format(obs=collision_count_obs, col=collision_count_agent, n=num_drone, step=i)) 
        

    plt.draw()
    frame_path = os.path.join(frame_directory_path, f"frame_{i:04d}.png")
    plt.savefig(frame_path) 
    
    if collision_count_agent > 0 or collision_count_obs > 0:
        break
    if i != sim_steps - 1:
        [items.remove() for items in body]
        [items.remove() for items in predictions]
        stats.remove()



# 获取文件夹中所有的帧
frames = [f for f in os.listdir(frame_directory_path) if f.startswith("frame_") and f.endswith(".png")]

# 按文件名排序
frames.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))

images = []
for frame in frames:
    frame_path = os.path.join(frame_directory_path, frame)
    images.append(imageio.imread(frame_path))

imageio.mimsave(path+'/animate.gif', images, duration=0.1,loop=0) # 0.1 是每帧的持续时间，您可以根据需要进行调整




