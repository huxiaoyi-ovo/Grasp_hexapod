
from legged_gym.envs import HexClimb, HexGround
from legged_gym.envs import HexClimbCfg, HexGroundCfg, HexGroundCfgPPO
from legged_gym.envs import HexTerrain, HexTerrainCfg, HexTerrainCfgPPO
from legged_gym.utils.joy_stick import JoyStick
from legged_gym.utils import get_args
from legged_gym.utils.helpers import update_cfg_from_args, class_to_dict, parse_sim_params,get_load_path
from legged_gym import LEGGED_GYM_ROOT_DIR
from rsl_rl.modules import ActorCritic, ActorCriticEncoder
from rsl_rl.storage import RolloutStorageMemory

from isaacgym import gymapi,gymutil

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset, Dataset
from torch.utils.tensorboard import SummaryWriter

from collections import deque

import rospy
from std_msgs.msg import Float32MultiArray

import statistics

import os

import sys

#按时间顺序发布指令
def GenCommand(step:int):
    vx=0.0
    vy=0.0
    yaw=0.0
    if step<7/0.02:
        vx=0.7
    elif step<14/0.02:
        vy=0.6
    elif step<21/0.02:
        vx=0.7
        vy=0.6
    vx =vx*env_cfg.commands.ranges.lin_vel_x[1]
    vy =vy*env_cfg.commands.ranges.lin_vel_y[1]
    yaw =yaw*env_cfg.commands.ranges.ang_vel_yaw[1]
    return vx, vy, yaw
#在固定时间段发送扰动指令
def GenDisturbance(step:int):
    return 0.0
    time_pairs=[[4.0,7.0],[11.0,14.0],[18.0,21.0]]
    disturbance=0.0
    for time_pair in time_pairs:
        if step>time_pair[0]/0.02 and step<time_pair[1]/0.02:
            disturbance=torch.randn(env_cfg.env.num_envs,env_cfg.env.num_actions).to(device)*0.2
    return disturbance
def WriteLog(title,vx,vy,q_list,step):
    # writer.add_scalar(title+'/q1',q_list[0],step)
    # writer.add_scalar(title+'/q2',q_list[1],step)
    # writer.add_scalar(title+'/q3',q_list[2],step)
    # writer.add_scalar(title+'/vx',vx,step)
    # writer.add_scalar(title+'/vy',vy,step)
    pass

#手柄控制测试程序
def Expert_Play(env:HexGround,cfg:HexGroundCfg,mode):
    reward_buffer = deque(maxlen=100)
    total_reward = torch.zeros(env.num_envs,dtype=torch.float,device=env.device)
    joystick = JoyStick()

    if mode=='expert_ground':
        _,_ = env.reset()
    elif mode=='expert_terrain':
        _,_ ,_= env.reset_separate()

    reset=False
    time_steps=0
    while not env.gym.query_viewer_has_closed(env.viewer):
        reset,vx,vy,vz,yaw =joystick.get_commands()
        vx = cfg.commands.ranges.lin_vel_x[1]*vx
        vy = cfg.commands.ranges.lin_vel_y[1]*vy
        yaw = cfg.commands.ranges.ang_vel_yaw[1]*yaw
        # vx,vy,yaw=GenCommand(time_steps)
        # if time_steps%(7/0.02)==0 and time_steps>0:
        #     reset=True
        # if time_steps > 21/0.02:
        #     print("time ends")
        #     exit(0)
        command = torch.tensor([[reset,vx,vy,0,yaw]],dtype=torch.float,device=env.device)
        # command = command.repeat(env.num_envs,1)
        # print("command=",command)
        if reset:
            _,_ = env.reset()
            # _,_ ,_= env.reset_separate()
            reset=False

        else:
            # dof_pos_expect = expert.ProcessCommand(command,dof_pos_cur)
            #使用ros进行可视化
            base_msgs.data.extend(env.base_lin_vel[0])#xyz
            # base_msgs.data.append(env.base_ang_vel[0,2])#omega_z
            base_msgs.data.extend(command[0][1:4])
            # base_msgs.data.append(env.base_ang_vel[0,2])
            # base_msgs.data.append(yaw)
            # joints_msgs.data.extend(env.root_states[0,7:10].cpu().numpy())
            # joints_msgs.data.extend(env.base_lin_vel[0].cpu().numpy())
            # joints_msgs.data.extend(command[0][1:4].cpu().numpy())
            # joints_msgs.data[0]=env.expert.B_e_des[0,0,0]
            # joints_msgs.data[1]=env.expert.B_e_des[0,0,1]
            # joints_msgs.data[2]=env.expert.B_e_des[0,0,2]
            # joints_msgs.data[3]=env.expert.B_e_cur[0,0,0]
            # joints_msgs.data[4]=env.expert.B_e_cur[0,0,1]
            # joints_msgs.data[5]=env.expert.B_e_cur[0,0,2]
            # joints_msgs.data[6]=env.expert.gaits[0,0]*0.25
            # print("joints_msgs.data=",joints_msgs.data)
            # joints_pub.publish(joints_msgs)
            base_pub.publish(base_msgs)
            base_msgs.data=[]
            # joints_msgs.data=[]


            # print("dof_pos_expect=",dof_pos_expect)
            # actions = (dof_pos_expect-env.default_dof_pos)/cfg.control.action_scale
            env.commands[:,0]=vx
            env.commands[:,1]=vy
            env.commands[:,2]=yaw
            env.reset_buf.fill_(reset)
            expert_actions=env.get_expert_actions()
            #为专家动作添加正态分布噪声
            # expert_actions += torch.randn_like(expert_actions)*0.1
            if mode == 'expert_ground':
                _,_,reward,dones,infos = env.step(expert_actions)
            elif mode =='expert_terrain':
                _,_,_,reward,dones,infos = env.step_separate(expert_actions)
            # dof_pos_cur=env.dof_pos

            #统计专家能获取多少奖励
            total_reward+=reward
            terminal_idx=torch.where(dones)[0]
            # print("terminal_idx=",terminal_idx)
            reward_buffer.extend(total_reward[terminal_idx].cpu().numpy().tolist())
            total_reward[terminal_idx]=0.0


            if len(reward_buffer)>0 and env.reset_buf.any():
                print("reward_buffer.mean()=",statistics.mean(reward_buffer))
                for key in infos['episode'].keys():
                    print(f"episode_info: {key}={infos['episode'][key]}")
            pos_des=expert_actions*env.cfg.control.action_scale + env.default_dof_pos
            pos_des = pos_des.reshape(-1,6,3)
            pos_cur = env.dof_pos.reshape(-1,6,3)
            torques = env.torques.reshape(-1,6,3)
            joint_msgs.data.extend(pos_des[0,4])
            joint_msgs.data.extend(pos_cur[0,4])
            joint_msgs.data.extend(torques[0,4])
            joint_pub.publish(joint_msgs)
            joint_msgs.data=[]
            #记录专家质心速度和一条腿关节的实际角度变化
            # writer.add_scalars('Expert',{
            #     'vx':env.base_lin_vel[0,0],
            #     'vy':env.base_lin_vel[0,1],
            # },time_steps)
            #env.base_ang_vel[0,2],
            # WriteLog('Expert',env.base_lin_vel[0,0],
            #          env.base_lin_vel[0,1],
            #          [env.dof_pos[0,0],env.dof_pos[0,1],env.dof_pos[0,2]],
            #          time_steps)
        time_steps+=1


#在线生成数据行为克隆程序
def BC_train(env_cfg:HexGroundCfg,train_cfg,device='cpu'):
    actor_critic = ActorCritic(env_cfg.env.num_observations,env_cfg.env.num_privileged_obs,
                               env_cfg.env.num_actions,**train_cfg['policy']).to(device)
    
    #设置环境长度,一个step0.02s，96s一个episode(1500步)，收集10个episode
    num_episodes = 1
    num_steps = 1500
    #TODO：初始化buffer
    
    # os.listdir(LEGGED_GYM_ROOT_DIR+'/resources/expert_data/')
    if os.path.exists(LEGGED_GYM_ROOT_DIR+'/resources/expert_data/bc_episode_0.pth'):
        print("Find expert data, loading......")
        buffer = torch.load(LEGGED_GYM_ROOT_DIR+'/resources/expert_data/bc_episode_0.pth',weights_only=True)
        print("Load expert data done!")
    else:
        with torch.no_grad():
            for i in range(num_episodes):
                print(f"----------->collecting {float(i+1)/num_episodes*100:.2f}% expert data<----------")
                obs,priv_obs = env.reset()
                buffer = {'obs':[],'expert_actions':[]}
                for _ in range(num_steps):
                    expert_actions = env.get_expert_actions()
                    # print("obs=",obs[0])
                    #TODO:把obs，priv_obs，和expert_actions 放入数据Buffer中
                    buffer['obs'].append(obs.clone().cpu())
                    buffer['expert_actions'].append(expert_actions.clone().cpu())
                    obs,priv_obs,reward,dones,infos = env.step(expert_actions)
                #TODO: 将buffer的数据保存在resources文件夹下，
                buffer['obs']=torch.cat(buffer['obs'],dim=0)
                buffer['expert_actions']=torch.cat(buffer['expert_actions'],dim=0)
                torch.save(buffer,LEGGED_GYM_ROOT_DIR+f'/resources/expert_data/bc_episode_{i}.pth')
                del buffer
                torch.cuda.empty_cache()

    #TODO：对actor_critic中的actor网络进行训练，保存模型在resource文件夹下
    print("buffer['obs'].shape=",buffer['obs'].shape) #6144000 768000
    buffer['obs']=buffer['obs'][:644000,:]
    buffer['expert_actions']=buffer['expert_actions'][:644000,:]
    dataset = TensorDataset(buffer['obs'].to(device), buffer['expert_actions'].to(device))
    dataloader = DataLoader(dataset, batch_size=1024, shuffle=True)
    optimizer = optim.Adam(actor_critic.parameters(), lr=1e-3)
    actor_critic.actor.train()
    # loss_fn=nn.MSELoss()
    for epoch in range(10):
        total_loss = 0.0
        for obs_batch, expert_actions in dataloader:
            actor_critic.update_distribution(obs_batch)
            log_probs = actor_critic.get_actions_log_prob(expert_actions)
            mask=log_probs>0
            if mask.any():
                print("log_probs= ",log_probs[mask][0])
                print("actor's mean\n{}\n std\n{}",actor_critic.action_mean,actor_critic.action_std)
                print("expert's action\n",expert_actions[mask][0])
                exit(0)
            loss = -log_probs.mean()

            # actions=actor_critic.act(obs_batch)
            # loss=loss_fn(actions,expert_actions)


            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()
        print(f"epoch={epoch},average_loss={total_loss/len(dataloader)}")
        print("action std=",actor_critic.std)
    print("Save actor model")
    torch.save(actor_critic.state_dict(),LEGGED_GYM_ROOT_DIR+'/resources/expert_data/bc_actor3.pth')


def BC_play(env:HexGround,env_cfg,train_cfg,device='cpu'):
    joystick = JoyStick()

    actor_critic = ActorCritic(env.num_obs,env.num_privileged_obs,
                            env.num_actions,**train_cfg['policy']).to(device)
    actor_critic.eval()
    actor_critic.load_state_dict(torch.load(LEGGED_GYM_ROOT_DIR+'/resources/expert_data/bc_actor2.pth',weights_only=True))
    obs,priv_obs = env.reset()
    time_steps=0
    reset=False
    while not env.gym.query_viewer_has_closed(env.viewer):
        with torch.no_grad():
            # if time_steps > 21/0.02:
            #     print("time ends")
            #     exit(0)
            # if time_steps%(7/0.02)==0 and time_steps>0:
            #     reset=True
            #     print("reset robot")
            reset,vx,vy,vz,yaw =joystick.get_commands()
            if reset:
                obs,priv_obs = env.reset()
                # reset=False
            else:
                #从手柄获取信息
                vx = env_cfg.commands.ranges.lin_vel_x[1]*vx
                vy = env_cfg.commands.ranges.lin_vel_y[1]*vy
                yaw = env_cfg.commands.ranges.ang_vel_yaw[1]*yaw
                #根据时段指定信息
                # vx,vy,yaw=GenCommand(time_steps)
                obs[:,-1]=yaw*env.obs_scales.ang_vel
                obs[:,-2]=vy*env.obs_scales.lin_vel
                obs[:,-3]=vx*env.obs_scales.lin_vel
                #加入扰动
                disturbance=GenDisturbance(time_steps)
                actions = actor_critic.act_inference(obs)+disturbance
                # actions = torch.randn_like(actions)*0.3+actions
                obs,priv_obs,reward,dones,infos = env.step(actions)

                base_msgs.data.extend(env.root_states[0,7:9].cpu().numpy())#xyz
                base_msgs.data.extend([vx,vy])
                base_pub.publish(base_msgs)
                base_msgs.data=[]
                # writer.add_scalars('BC',{
                #                     'vx':env.base_lin_vel[0,0],
                #                     'vy':env.base_lin_vel[0,1],
                #                     },time_steps)
                # writer.add_scalar('/BC/vx',env.base_lin_vel[0,0],time_steps)
                # writer.add_scalar('/BC/vy',env.base_lin_vel[0,1],time_steps)
                # writer.add_scalar('/BC/q1',env.dof_pos[0,0],time_steps)
                WriteLog('BC',env.base_lin_vel[0,0],
                        env.base_lin_vel[0,1],
                        [env.dof_pos[0,0],env.dof_pos[0,1],env.dof_pos[0,2]],
                        time_steps)
            time_steps+=1

def Hex_Ground_Play(env:HexGround,env_cfg:HexGroundCfg,train_cfg,device='cpu'):
    global last_dof_vel
    if env.num_privileged_obs==None:
        env.num_privileged_obs=env.num_obs
    actor_critic = ActorCritic(env.num_obs,env.num_privileged_obs,
                            env.num_actions,**train_cfg['policy']).to(device)
    actor_critic.eval()
    log_root = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg['runner']['experiment_name'])
    resume_path = get_load_path(log_root, load_run=train_cfg['runner']['load_run'], checkpoint=train_cfg['runner']['checkpoint'])
    print("============>load model {}<===============".format(resume_path))
    actor_critic.load_state_dict(torch.load(resume_path,weights_only=True)['model_state_dict'])
    obs,priv_obs = env.reset()
    joystick = JoyStick()
    # env.foot_traj_viz=True

    time_steps=0
    reset=False
    while not env.gym.query_viewer_has_closed(env.viewer):
        with torch.no_grad():
            # if time_steps > 21/0.02:
            #     print("time ends")
            #     exit(0)
            # if time_steps%(7/0.02)==0 and time_steps>0:
            #     reset=True
            #     print("reset robot")
            # # reset,vx,vy,vz,yaw =joystick.get_commands()
            # if reset:
            #     obs,priv_obs = env.reset()
            #     reset=False    

            reset,vx,vy,vz,yaw =joystick.get_commands()
            if reset:
                obs,priv_obs = env.reset()
            else:

                # vx,vy,yaw=GenCommand(time_steps)
                vx = env_cfg.commands.ranges.lin_vel_x[1]*vx
                vy = env_cfg.commands.ranges.lin_vel_y[1]*vy
                yaw = env_cfg.commands.ranges.ang_vel_yaw[1]*yaw
                # print("vx={},vy={},yaw={}".format(vx,vy,yaw))
                
                obs[:,74]=yaw*0.25
                obs[:,73]=vy*2.0
                obs[:,72]=vx*2.0
                disturbance=GenDisturbance(time_steps)
                actions = actor_critic.act_inference(obs)+disturbance
                # actions = actor_critic.act(obs)

                
                obs,priv_obs,reward,dones,infos = env.step(actions)

                pos_des=(actions*env.cfg.control.action_scale+env.default_dof_pos)[0]
                
                base_msgs.data.extend(env.base_lin_vel[0])#xyz
                base_msgs.data.extend([vx,vy,0])
                base_msgs.data.append(yaw)
                base_msgs.data.append(env.base_ang_vel[0,2])
                joint_msgs.data.extend(pos_des[0:3])
                joint_msgs.data.extend(env.dof_pos[0,0:3])
                joint_msgs.data.extend(env.dof_vel[0,0:3])
                joint_msgs.data.extend(((env.dof_vel-last_dof_vel)/env.dt)[0,0:3])
                last_dof_vel[:]=env.dof_vel[:]

                
                
                base_pub.publish(base_msgs)
                joint_pub.publish(joint_msgs)
                base_msgs.data=[] 
                joint_msgs.data=[]
                # writer.add_scalars('RL',{
                #                     'vx':env.base_lin_vel[0,0],
                #                     'vy':env.base_lin_vel[0,1],
                #                     },time_steps)
                # WriteLog('EGPO',env.base_lin_vel[0,0],
                #         env.base_lin_vel[0,1],
                #         [env.dof_pos[0,0],env.dof_pos[0,1],env.dof_pos[0,2]],
                #         time_steps)
            time_steps+=1
            # debug_msgs.data.extend([vx,vy,0])
            # debug_msgs.data.extend(env.base_lin_vel[0].cpu().numpy())
            # pub.publish(debug_msgs)
            # debug_msgs.data=[]

def Hex_Terrain_Play(env:HexGround, env_cfg:HexGroundCfg,train_cfg_dict,device='cpu'):
    actor_critic = ActorCriticEncoder(env.num_obs,env.num_actions,**train_cfg_dict['policy']).to(device)
    actor_critic.eval()
    log_root = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg_dict['runner']['experiment_name'])
    resume_path = get_load_path(log_root, load_run=train_cfg_dict['runner']['load_run'], checkpoint=train_cfg_dict['runner']['checkpoint'])
    print("============>load model {}<===============".format(resume_path))
    actor_critic.load_state_dict(torch.load(resume_path,weights_only=True)['model_state_dict'])
    storage=RolloutStorageMemory(env_cfg.env.num_envs,
                                 train_cfg.runner.num_steps_per_env,
                                 10,[env_cfg.env.num_observations+30],[11*13],[env_cfg.env.num_actions],device)
    obs,obs_vgf,obs_terrain = env.reset_separate()
    joystick = JoyStick()
    # env.foot_traj_viz=True

    time_steps=0
    reset=False
    while not env.gym.query_viewer_has_closed(env.viewer):
        with torch.no_grad():
            # if time_steps > 21/0.02:
            #     print("time ends")
            #     exit(0)
            # if time_steps%(7/0.02)==0 and time_steps>0:
            #     reset=True
            #     print("reset robot")
            # # reset,vx,vy,vz,yaw =joystick.get_commands()
            # if reset:
            #     obs,priv_obs = env.reset()
            #     reset=False    

            reset,vx,vy,vz,yaw =joystick.get_commands()
            if reset:
                obs,obs_vgf,obs_terrain = env.reset_separate()
            else:

                # vx,vy,yaw=GenCommand(time_steps)
                vx = env_cfg.commands.ranges.lin_vel_x[1]*vx
                vy = env_cfg.commands.ranges.lin_vel_y[1]*vy
                yaw = env_cfg.commands.ranges.ang_vel_yaw[1]*yaw
                # print("vx={},vy={},yaw={}".format(vx,vy,yaw))
                # print("base_lin_acc=",env.base_lin_acc)
                obs[:,-1]=yaw*env_cfg.normalization.obs_scales.command
                obs[:,-2]=vy*env_cfg.normalization.obs_scales.command
                obs[:,-3]=vx*env_cfg.normalization.obs_scales.command
                obs_vgf_estimates=actor_critic.actor_obs_priv_estimator(obs)
                # obs_vgf_estimates.fill_(0.0)
                obs_splice=torch.cat([obs,obs_vgf_estimates],dim=-1)
                storage.update_obs_hist(obs_splice)
                obs_hist,mask=storage.get_current_obs_hist()
                obs_terrain_lstm_latent=actor_critic.LSTM_encode(obs_hist,mask)
                # obs_terrain_lstm_latent.fill_(0.0)
                actions = actor_critic.act_inference(obs_splice,obs_terrain_lstm_latent)

                obs,obs_vgf,obs_terrain,reward,dones,infos = env.step_separate(actions)
                storage.update_dones_hist(dones)
                


                pos_des=(actions*env.cfg.control.action_scale+env.default_dof_pos)[0]
                
                base_msgs.data.extend(env.base_lin_vel[0])#xyz
                estimates_xyz=obs_vgf_estimates[0,0:3]/env_cfg.normalization.obs_scales.lin_vel
                base_msgs.data.extend(estimates_xyz)
                # obs_vgf_estimates[0,2]/env_cfg.commands.ranges.lin_vel_
                # base_msgs.data.extend([vx,vy])
                joint_msgs.data.extend(env.dof_pos[0,0:3])
                joint_msgs.data.extend(pos_des[0:3])

                rb_msgs.data.extend(env.contact_forces[0,env.feet_indices[0],:]) #0个环境的第一个脚的xyz接触力
                
                base_pub.publish(base_msgs)
                joint_pub.publish(joint_msgs)
                rb_pub.publish(rb_msgs)
                rb_msgs.data=[]
                base_msgs.data=[] 
                joint_msgs.data=[]
                # writer.add_scalars('RL',{
                #                     'vx':env.base_lin_vel[0,0],
                #                     'vy':env.base_lin_vel[0,1],
                #                     },time_steps)
                # WriteLog('EGPO',env.base_lin_vel[0,0],
                #         env.base_lin_vel[0,1],
                #         [env.dof_pos[0,0],env.dof_pos[0,1],env.dof_pos[0,2]],
                #         time_steps)
            time_steps+=1
            # debug_msgs.data.extend([vx,vy,0])
            # debug_msgs.data.extend(env.base_lin_vel[0].cpu().numpy())
            # pub.publish(debug_msgs)
            # debug_msgs.data=[]
if __name__ == '__main__':
    mode = 'expert_ground'
    for i, arg in enumerate(sys.argv):
        if arg.startswith('--mode'):
            mode = arg.split('=')[1]
            sys.argv.pop(i)
            break
    

    if 'terrain' in mode:
        env_cfg = HexTerrainCfg()
        train_cfg = HexTerrainCfgPPO()
        env_cfg.env.num_envs = 3
        env_cfg.terrain.num_rows = 1
        env_cfg.terrain.num_cols = 1
        env_cfg.terrain.curriculum = False
        env_cfg.noise.add_noise = False
        env_cfg.domain_rand.randomize_friction = False
        env_cfg.domain_rand.push_robots = False  
    else:
        env_cfg = HexGroundCfg()
        train_cfg = HexGroundCfgPPO()
        env_cfg.env.num_envs = 3
        env_cfg.terrain.num_rows = 5
        env_cfg.terrain.num_cols = 5
        env_cfg.terrain.curriculum = False
        env_cfg.noise.add_noise = False
        env_cfg.domain_rand.randomize_friction = False
        env_cfg.domain_rand.push_robots = False    
    args = get_args()
    env_cfg,train_cfg = update_cfg_from_args(env_cfg,train_cfg,args) 
    sim_params = {"sim":class_to_dict(env_cfg.sim)}
    sim_params = parse_sim_params(args, sim_params)



    train_cfg_dict = class_to_dict(train_cfg)
    if mode != 'bc_train':
        if 'ground' in mode:
            env = HexGround(env_cfg,sim_params,args.physics_engine,args.sim_device,args.headless)
        elif 'terrain' in mode:
            env = HexTerrain(env_cfg,sim_params,args.physics_engine,args.sim_device,args.headless)
    device=args.sim_device

    #用于记录tensorboard
    # log_data_dir=f"{LEGGED_GYM_ROOT_DIR}/logs/motion_data"
    # writer = SummaryWriter(log_data_dir,flush_secs=7)

    rospy.init_node('control_test', anonymous=True)
    base_msgs=Float32MultiArray()
    joint_msgs=Float32MultiArray()
    rb_msgs=Float32MultiArray()
    base_pub = rospy.Publisher('/base_msg',Float32MultiArray,queue_size=10)
    joint_pub = rospy.Publisher('/joint_msg',Float32MultiArray,queue_size=10)
    rb_pub = rospy.Publisher('/rb_msg',Float32MultiArray,queue_size=10)
    base_msgs.data=[] #real xyz omega_z expected x y omega_z
    joint_msgs.data=[]
    last_dof_vel=torch.zeros_like(env.dof_vel)


    #用于计算expert能获得多少奖励
    if mode == 'hex_ground':
        Hex_Ground_Play(env,env_cfg,train_cfg_dict,device='cuda')
    elif mode == 'hex_terrain':
        Hex_Terrain_Play(env,env_cfg,train_cfg_dict,device='cuda')        
    elif mode == 'expert_terrain' or mode == 'expert_ground':
        Expert_Play(env,env_cfg,mode)
    elif mode == 'bc_train':
        BC_train(env_cfg,train_cfg_dict,device='cuda')
    elif mode == 'bc_play':
        BC_play(env,env_cfg,train_cfg_dict,device='cuda')

    
