#六足简单位置控制器，用于早期训练引导到专家策略附近
import torch
from typing import Tuple
from .hex_climb_config import HexClimbCfg
from .hex_ground_config import HexGroundCfg
from legged_gym.utils.kinematic import Kinematic
import time

class ExpertClimb:
    def __init__(self,cfg:HexClimbCfg,device,env_nums):
        self.device=device
        self.env_nums=env_nums
        # init params and kinematic utils
        self.cfg = cfg
        # self.dt = self.cfg.control.decimation * self.cfg.sim.dt
        #理论上的时间应该按照上面计算，但是下面这样会使运动速度变化比较平滑，奖励更高
        self.dt = 0.02
        
        self.kin=Kinematic(self.cfg.asset.links.l1,
                           self.cfg.asset.links.l2,
                           self.cfg.asset.links.l3,device)

        # init curv variables for multi-env
        self.set_init_done=torch.zeros(env_nums,dtype=torch.bool,device=device)
        # self.swing_reach_high=torch.zeros(env_nums,dtype=torch.bool,device=device)
        self.swing_reach_point=torch.zeros(env_nums,6,dtype=torch.bool,device=device)
        self.swing_init_point=torch.zeros(6,3,dtype=torch.float32,device=device)
        self.swing_init_point[...,0]=0.175 # x
        self.swing_init_point[...,2]=0.02
        self.swing_init_point[[2,5],0]=0.2
        self.swing_init_point[[0,4],1]=0.09
        self.swing_init_point[[1,3],1]=-0.09


        self.gaits=torch.zeros(env_nums,6,dtype=torch.bool,device=device)
        self.A_group_index=torch.tensor([0,1,5],dtype=torch.int64,device=device)
        self.B_group_index=torch.tensor([2,3,4],dtype=torch.int64,device=device)
        self.gaits_groups=torch.tensor([[0,1,5],[2,3,4]],dtype=torch.int64,device=device)
        self.stance_group_index=torch.zeros(env_nums,dtype=torch.int64,device=device)
        self.B_e_des=torch.ones(env_nums,6,4,dtype=torch.float32,device=device)
        self.B_e_cur=torch.ones(env_nums,6,4,dtype=torch.float32,device=device)
        self.q_init=torch.zeros(24,dtype=torch.float32,device=device)
        self.B_e_init=torch.ones(6,4,dtype=torch.float32,device=device)
        self.R1_T_R=torch.stack([torch.eye(4,dtype=torch.float32,device=device) for _ in range(env_nums)])
        self.R1_T_R_swing=torch.stack([torch.eye(4,dtype=torch.float32,device=device) for _ in range(env_nums)])
        self.body_shape=torch.zeros(6,4,dtype=torch.float32,device=device)
        # self.vec_z=(torch.rand(env_nums,6,dtype=torch.float32,device=device)-0.5)*0.05+self.cfg.max_vec.z
        self.vec_z=0.6
        #for using kinematics api
        self.B_e_des_flat=self.B_e_des.view(env_nums*6,4)
        self.B_e_cur_flat=self.B_e_cur.view(env_nums*6,4)


        #init returned actions
        self.adhesions=torch.zeros(env_nums,6,dtype=torch.bool,device=device)
        self.q_des=torch.zeros(env_nums,6,4,dtype=torch.float32,device=device)
        self.q_des_flat=self.q_des.view(env_nums*6,4)

        #init outer tensors for multi-env
        # self.q_des=joint_pos_des # env_nums*6*4 float32 tensor
        # self.q_cur=joint_pos_cur # env_nums*6*3 float32 tensor
        # self.adhesions=adhesions # env_nums*6 bool tensor
        # self.suction_forces=suction_forces # env_nums*6 float32 tensor

        # self.q_cur_flat=self.q_cur.view(env_nums*6,3)

        #set body shape initial
        self.body_shape[0:3,0]=-self.cfg.asset.body_shape.x
        self.body_shape[3:6,0]=self.cfg.asset.body_shape.x
        self.body_shape[[0,3],1]=-self.cfg.asset.body_shape.y
        self.body_shape[[1,4],1]=self.cfg.asset.body_shape.y

        #set initial foot end pos joint pos
        self.leg_names=['lb','lf','lm','rb','rf','rm']
        self.joint_names=['thigh','knee','ankle','foot']
        i=0
        for leg_name in self.leg_names:
            for joint_name in self.joint_names:
                q_name='j_'+leg_name+'_'+joint_name
                if q_name in self.cfg.init_state.default_joint_angles.keys():
                    self.q_init[i]=self.cfg.init_state.default_joint_angles[q_name]
                    i+=1

        self.kin.ForwardKin(self.q_init.view(6,4),self.B_e_init)

        self._GetFootAngle(self.q_init.view(6,4).repeat(env_nums,1))
        # self.q_des[:]=self.q_init
        self.B_e_des[:]=self.B_e_init.unsqueeze(0)
        
        # print("-------------initial B_e_des-------------\n",self.B_e_des)


    # def ResetJoint(self,env_indx:torch.Tensor):
    #     self.set_init_done[env_indx]=False

    def ProcessCommand(self,command:torch.Tensor,
                       q_cur:torch.Tensor,q_dot_cur:torch.Tensor,
                       suction_forces:torch.Tensor)->Tuple[torch.Tensor,torch.Tensor]:
        """
        @input: command [set_init,vx,vy,vz,omega], q_cur suction_force

        @output: adhesions env_nums*6, q_des_flat env_nums*24
        """
        #update current position of foot end in leg base frame
        q_cur_flat=q_cur.view(self.env_nums*6,3)
        q_dot_cur_flat = q_dot_cur.view(self.env_nums*6,3)
        suction_forces=suction_forces.view(self.env_nums,-1)
        self.kin.ForwardKin(q_cur_flat,self.B_e_cur_flat)


        reset_mask=command[:,0].to(torch.bool)
        reset_index=torch.where(reset_mask)[0]
        if reset_mask.any():
            #reset hex states
            self.set_init_done[reset_mask]=False
            self.swing_reach_point[reset_mask]=False
            self.stance_group_index[reset_mask]=0
            self.gaits[reset_mask]=0
            self.gaits[reset_index.unsqueeze(-1),self.A_group_index]=1
            # self.q_des[reset_index,...]=self.q_init[reset_index,...]
            self.B_e_des[reset_index,...]=self.B_e_init
            self.adhesions[reset_index,...]=0
            self.adhesions[reset_index.unsqueeze(-1),self.A_group_index]=1
            #judge footend if reach desired point
            reset_done_mask=(torch.norm((self.B_e_des-self.B_e_cur),p=2,dim=2)<0.02).all(dim=1)
            reset_done_mask.fill_(True)
            # print(torch.norm((self.B_e_des-self.B_e_cur),p=2,dim=2))
            self.set_init_done[reset_done_mask]=True
        moving_env_mask=command[:,1].to(torch.bool)|command[:,2].to(torch.bool)|command[:,4].to(torch.bool)
        if self.set_init_done.any() and moving_env_mask.any():
            self.GaitPlanning(command,suction_forces)
            self.CalJointPoses(q_cur_flat,q_dot_cur_flat)

        return self.adhesions,self.q_des_flat.view(self.env_nums,-1)
            
    def GaitPlanning(self,command:torch.Tensor,suction_forces):
        # print(">>>>>>>>>>>>>GaitPlanning<<<<<<<<<<<<")

        self.adhesions[self.gaits]=1
        self.adhesions[~self.gaits]=0
        # print("adhesions before=",self.adhesions)

        # print("gaits=",self.gaits)
        #calculate Transform of robot body
        self._TargetTransInterp(command)
        #calculate next B_e_des env_nums*6*4
        next_B_e_des=self._CalB_e_des(command)

        # print("next_B_e_des\n",next_B_e_des)
        #check working range and collide feasible
        range_feasi=self._FeasiCheck(next_B_e_des) # env_nums*6 bool
        collide_feasi=self._CollideCheck(next_B_e_des) # env_nums*6 bool
        
        #get Adsorb and release results
        adsorb_release_mask=self._AdsorbReleaseDetection(suction_forces) # env_nums*6 bool tensor
        # print("adsorb_release_mask",adsorb_release_mask)
        # init buffer
        stance_done_env=torch.zeros(self.env_nums,dtype=torch.bool,device=self.device)
        swing_done_env=torch.zeros_like(stance_done_env,dtype=torch.bool,device=self.device)


        # select swing_done(finish contact) legs
        swing_done_mask=~self.gaits&self._ContactDetection(next_B_e_des) 
        self.adhesions[swing_done_mask]=1

        # select env continue siwng
        have_siwng_continue_env_mask=swing_done_mask.sum(dim=1)<3
        if have_siwng_continue_env_mask.any():
            # set_all_mask=(~swing_done_mask)&(~self.gaits)&(~self.swing_reach_point.unsqueeze(1))
            # self.B_e_des[set_all_mask]=next_B_e_des[set_all_mask]

            # set_z_mask=(~collide_feasi|~range_feasi)&(have_siwng_continue_env_mask.unsqueeze(-1))
            set_all_mask=((~swing_done_mask)&(~self.gaits)&(collide_feasi&range_feasi)) | ((~self.gaits)&(~self.swing_reach_point))
            set_z_mask=(~swing_done_mask)&(~self.gaits)&(~collide_feasi|~range_feasi)

            self.B_e_des[...,2][set_z_mask]=next_B_e_des[...,2][set_z_mask]
            self.B_e_des[set_all_mask]=next_B_e_des[set_all_mask]
            # print("self.swing_reach_point",self.swing_reach_point)
            # print("set_all_mask",set_all_mask)
        # every leg touch ground and reach hight, set the env swing done
        swing_done_env_count=( (~self.gaits)&swing_done_mask&(self.swing_reach_point) ).sum(dim=1)
        swing_done_env[swing_done_env_count==3]=1
        # print("swing_done_mask",swing_done_mask)
        # print("swing_done_env",swing_done_env)

        #for env have swing done leg, if reach hight, set this env's stance done
        set_stance_done_env_mask=(swing_done_mask.any(dim=1))
        stance_done_env[set_stance_done_env_mask]=1 #stance done because swing leg touch ground
        # stance_mask=self.gaits
        # choose envs that all stance legs within range and collide free & not stance done
        # this way cannot keep the original dimension range_feasi[stance_mask]&collide_feasi[stance_mask]
        # cont numbers instead
        # choose gaits & collide free & within range
        range_collide_free_env_mask=(self.gaits&range_feasi&collide_feasi).sum(dim=1)==3

        #for those not within range and collide free, set stance done
        stance_done_env[~range_collide_free_env_mask]=1
        # print("stance_done_env",stance_done_env)
        #select envs that satisfy range collide free and not stance done(some stance done is caused by swing)
        set_stance_env_mask=range_collide_free_env_mask & (~stance_done_env)
        # print("range_collide_free_env_mask",range_collide_free_env_mask)
        # print("set_stance_env_mask",set_stance_env_mask)
        #set_stance_env_mask: env_nums bool; gaits: env_nums*6 bool 
        #use mask to choose which B_e_des to set
        #if set_stance_env_mask[i]=True, then the i-th env of stance leg need to be set B_e_des
        #calculate every column of set_stance_env_mask with(&) set_stance_env_mask, result mask to set B_e_des
        set_B_e_des_mask=self.gaits&(set_stance_env_mask.unsqueeze(1))
        # print("set_B_e_des_maks\n",set_B_e_des_mask)
        # print("before set B_e_des\n",self.B_e_des)
        if set_B_e_des_mask.any():
            self.B_e_des[set_B_e_des_mask]=next_B_e_des[set_B_e_des_mask]
        # print("after set B_e_des\n",self.B_e_des)
        #find swing done envs, and judge which of them finish adsorb
        if swing_done_env.any():
            #select envs all swing legs have adsorb done
            # adsorb_done_env_mask=adsorb_release_mask[~self.gaits].all(dim=1)
            adsorb_done_env_mask=(adsorb_release_mask&(~self.gaits)).sum(dim=1)==3

            # print("adsorb_done_env_mask:",adsorb_done_env_mask)
            # for these envs, set stance adhesions to 0
            if adsorb_done_env_mask.any():
                #choose stance gaits & choose envs have adsorb done by colume-wise
                set_adhesion_mask=self.gaits&(adsorb_done_env_mask.unsqueeze(1))
                self.adhesions[set_adhesion_mask]=0

        #find judge which of tenvs finish release
        # stance_done_env&
        if stance_done_env.any():
            # release_done_env_mask=adsorb_release_mask[self.gaits].all(dim=1)
            release_done_env_mask=(self.gaits&adsorb_release_mask).sum(dim=1)==3
            # release_done_env_mask=release_done_env_mask&swing_done_env
            if release_done_env_mask.any():
                self.stance_group_index[release_done_env_mask]=1-self.stance_group_index[release_done_env_mask]
                self.swing_reach_point[release_done_env_mask]=False
                self.gaits[release_done_env_mask]=0
                # print("release_done_env_mask:\n",release_done_env_mask)
                # print("self.gaits\n",self.gaits)
                release_done_env_index=torch.where(release_done_env_mask)[0]
                #[0,1,4,5]->[[0,0,0],[1,1,1],[4,4,4],[5,5,5]] 4->4*1->4*3 3 to set stance
                exapnd_env_index=(release_done_env_index.unsqueeze(1)).expand(-1,3)
                stance_set_index=self.gaits_groups[self.stance_group_index[release_done_env_index]]
                # print("expand_env_index\n",exapnd_env_index)
                # print("stance_set_index\n",stance_set_index)
                self.gaits[exapnd_env_index,stance_set_index]=1

        # gait_list=['swing','stance']
        # for i in range(self.env_nums):
            # print("env ",i)
            # print([gait_list[self.gaits[i,j]] for j in range(6)])
            # print("swing_done",swing_done_env[i])
            # print("stance_done",stance_done_env[i])
            # print("adhesions",self.adhesions[i])
            # print("swing reach high",self.swing_reach_high[i])
        # print("B_e_des\n",self.B_e_des)
        # print("next_B_e_des\n",next_B_e_des)
        # print("collide_feasi\n",collide_feasi)
        # print("range_feasi\n",range_feasi)            
        # print("\n")
        # self.adhesions[:]=0
        # print("adhesions after=",self.adhesions)
        # print("\n")
        # print("swing_done_env:",swing_done_env[0])
        # print("stance_done_env:",stance_done_env[0])

    def CalJointPoses(self,q_cur_flat,q_dot_cur_flat):
        #damp_inv_jac_env: (env_nums*6)*3*3 q_cur_flat:(env_nums*6)*3
        damp_inv_jac_env=self.kin.DampInvJac(q_cur_flat) 
        pos_err=(self.B_e_des_flat[...,0:3]-self.B_e_cur_flat[...,0:3]).unsqueeze(-1) # (env_nums*6)*3*1
        self.q_des_flat[:,0:3]=q_cur_flat+60*((damp_inv_jac_env@pos_err).squeeze(-1))*self.dt\
                                -5.0*q_dot_cur_flat
        # self._GetFootAngle(self.q_des_flat)
        # self.B_e_des[...,3]=self.B_e_des[...,3]+(torch.rand_like(self.B_e_des[...,3])-0.5)
        # print(">>>>>>>>>>>>>>>>>>In curv_adapt_multi.py<<<<<<<<<<<<<<<<<<")
        # print("---------------obs-----------------")
        # pos=torch.zeros(6,3,dtype=torch.float32,device=self.device)
        # self.kin.ForwardKin(self.q_cur[0],pos)
        # print("q_cur\n",self.q_cur[0],"\nB_e_cur\n",pos,"\n suction force\n",self.suction_forces[0])
        # print("---------------action------------------")
        # self.kin.ForwardKin(self.q_des[0,:,0:3],pos)
        # print("q_des\n",self.q_des[0,:,0:3],"\n action_B_e\n",pos,"\n self.B_e_des\n",self.B_e_des[0])
        # print("adhesions\n",self.adhesions[0])

    def _BuildInvT(self,x, y, omega):
        cos_o = torch.cos(omega)
        sin_o = torch.sin(omega)

        R = torch.zeros(x.size(0), 3, 3, device=x.device)
        R[:, 0, 0] = cos_o
        R[:, 0, 1] = -sin_o
        R[:, 1, 0] = sin_o
        R[:, 1, 1] = cos_o
        R[:, 2, 2] = 1.0

        t = torch.stack([x, y, torch.zeros_like(x)], dim=1).unsqueeze(-1)
        R_T = R.transpose(1, 2)
        inv_t = -torch.bmm(R_T, t).squeeze(-1)
        T = torch.eye(4, device=x.device).repeat(x.size(0), 1, 1)
        T[:, 0:3, 0:3] = R_T
        T[:, 0:3, 3] = inv_t
        return T
    def _TargetTransInterp(self,command:torch.Tensor):#command: env_nums*[set_init,vx,vy,vz,omega]
        for i in range(2):
            sign = 1 if i == 0 else -1
            x_env = command[:,1] * self.dt * sign  #考虑到机器人运动顿挫，因此尽可能提高此时的速度
            y_env = command[:,2] * self.dt * sign  
            omega_env = command[:,4] * self.dt * sign *1.5

            inv_T = self._BuildInvT(x_env, y_env, omega_env)
            if i == 0:
                self.R1_T_R = inv_T
            else:
                self.R1_T_R_swing = inv_T        

    def _CalB_e_des(self,command:torch.Tensor)->torch.Tensor:
        swing_index_tuple=torch.where(~self.gaits)
        stance_index_tuple=torch.where(self.gaits) 

        #select swing and stance leg
        R_e_des=self._B2R(self.B_e_des) #env_nums*6*4
        # print("B_e_des:\n",self.B_e_des)
        next_R_e_des=torch.zeros_like(R_e_des)
        idx1=stance_index_tuple[0]
        idx2=stance_index_tuple[1]
        next_R_e_des[idx1,idx2]=( (self.R1_T_R[idx1,...])@((R_e_des[idx1,idx2]).unsqueeze(-1)) ).squeeze(-1)
        idx1=swing_index_tuple[0]
        idx2=swing_index_tuple[1]
        next_R_e_des[idx1,idx2]=( (self.R1_T_R_swing[idx1,...])@((R_e_des[idx1,idx2]).unsqueeze(-1)) ).squeeze(-1)
        next_B_e_des=self._R2B(next_R_e_des)

        # print("next_R_e_des:\n",next_R_e_des)
        # before reach swing initial point, set straight line to point
        # swing legs not reach initial point
        B_e_des_xyz=self.B_e_des[...,0:3]
        # B_e_des_xyz=self.B_e_cur[...,0:3]
        dis_norm=torch.norm(self.swing_init_point-B_e_des_xyz,p=2,dim=2)
        # print("B_e_des:\n",B_e_des_xyz)
        self.swing_reach_point=(self.B_e_des[...,2]>=self.swing_init_point[0,2]-0.005) | self.swing_reach_point
        # self.swing_reach_point=((dis_norm<self.dt).sum(dim=1)==3) | self.swing_reach_point
        back_to_init_mask=(~self.swing_reach_point) & (~self.gaits)
        #先把腿太高，再收回
        # raise_leg_mask=(self.B_e_des[...,2]<0.00) &(~self.gaits)&(~self.swing_reach_point)
        # back_to_init_B_e_des=self.B_e_des[...,0:3].clone()
        # back_to_init_B_e_des[...,2]=self.B_e_des[...,2]+self.vec_z*self.dt
        # next_B_e_des[...,0:3][raise_leg_mask]=back_to_init_B_e_des[raise_leg_mask]

        # leg_high_mask=(self.B_e_des[...,2]>=0.0) &(~self.gaits)&(~self.swing_reach_point)
        #mutiply by 1.3 to make trajectory get faster to the desired point

        # vec_z = torch.norm(command[:,[1,2]],dim=1)*2.5
        # zero_mask = vec_z==0.0
        # vec_z[zero_mask] = (command[zero_mask,4]*0.17*3.5).abs()
        # vec_z = vec_z.unsqueeze(-1).unsqueeze(-1)

        # print("swing_init_point size=",self.swing_init_point.shape)
        # print("B_e_des size=",B_e_des_xyz.shape)
        # print("dis_norm size=",dis_norm.shape)
        # print("vec_z size=",vec_z.shape)
        v_cmd=torch.norm(command[:,1:3],dim=1) #num_envs
        v_cmd_omega=torch.abs(command[:,4])*0.8
        v_z = torch.max(v_cmd,v_cmd_omega)
        # v_cmd[v_cmd==0]=torch.abs(command[:,4][v_cmd==0])*0.8
        # v_cmd += torch.abs(command[:,4])*0.3*3.0
        v_z = torch.clamp(v_z,max=1.2)

        # v_cmd[v_cmd==0]=(command[:,3]*self.swing_init_point[0,0])[v_cmd==0]

        back_to_init_B_e_des=((self.swing_init_point.unsqueeze(0)-B_e_des_xyz)/dis_norm.unsqueeze(2))\
                                *self.dt*(v_z.unsqueeze(1).unsqueeze(1))\
                                +B_e_des_xyz
        # next_B_e_des[...,0:3][leg_high_mask]=back_to_init_B_e_des[leg_high_mask]
        next_B_e_des[...,0:3][back_to_init_mask]=back_to_init_B_e_des[back_to_init_mask]

        # print("swing reach point:\n",self.swing_reach_point)
        # print("back to init mask:\n",back_to_init_mask)

        # after reach point, set swing xy normal set z sub
        v_z = v_z.unsqueeze(1).expand(-1,6)
        sub_z_mask=~self.gaits&(self.swing_reach_point)
        #根据高度设置不同的z值0.095 -0.1
        # speed0_mask=(next_B_e_des[...,2]>-0.08)
        # speed1_mask=(next_B_e_des[...,2]<-0.0) & (next_B_e_des[...,2]>-0.08)
        # speed2_mask=(next_B_e_des[...,2]<-0.08)
        # next_B_e_des[...,2][sub_z_mask&speed0_mask]-=v_cmd[sub_z_mask&speed0_mask]*0.9*self.dt
        # next_B_e_des[...,2][sub_z_mask&speed1_mask]-=v_cmd[sub_z_mask&speed1_mask]*0.8*self.dt
        # next_B_e_des[...,2][sub_z_mask&speed2_mask]-=v_cmd[sub_z_mask&speed2_mask]*0.4*self.dt

        # next_B_e_des[...,2][sub_z_mask]-=self.vec_z*self.dt
        # print("sub_z_mask=",sub_z_mask)
        # print("sub_z_mask.any(dim=1)",sub_z_mask.any(dim=1))
        # print("next_B_e_des[...,2][sub_z_mask].shape=",next_B_e_des[...,2][sub_z_mask].shape)
        # print("v_cmd[sub_z_mask.any(dim=1)].shape=",v_cmd[sub_z_mask.any(dim=1)].shape)
        next_B_e_des[...,2][sub_z_mask]-=v_z[sub_z_mask]*self.dt
        # print("sub_z_mask\n",sub_z_mask[0])
        # print("next_B_e_des\n",next_B_e_des[0])
        # print("v_z\n",v_z)

        # time.sleep(0.2)
        return next_B_e_des

    def _GetFootAngle(self,joint_pos:torch.Tensor):
        self.q_des_flat[:,3]=-(joint_pos[:,1]+joint_pos[:,2])-torch.pi/2.0
        # joint_pos[:,3]=-(joint_pos[:,1]+joint_pos[:,2])-torch.pi/2.0

    #return env_nums*6 bool tensor
    def _FeasiCheck(self,B_e_des:torch.Tensor)->torch.Tensor:
        xy_len=(B_e_des[...,0].pow(2)+B_e_des[...,1].pow(2)).sqrt()
        xy_feasi=(xy_len>=0.14) & (xy_len<=0.24)
        # x_feasi=(B_e_des[...,0]>=0.16) & (B_e_des[...,0]<=0.28)
        z_feasi=(B_e_des[...,2]>=-0.2) & (B_e_des[...,2]<=0.1)
        thigh_angle=torch.atan(B_e_des[...,1]/B_e_des[...,0])
        angle_feasi=(thigh_angle>=-torch.pi/4.0) & (thigh_angle<=torch.pi/4.0)
        # print("x_feasi\n",x_feasi,"\nz_feasi\n",z_feasi,"\nangle_feasi\n",angle_feasi)
        # return x_feasi&z_feasi&angle_feasi
        return xy_feasi&z_feasi&angle_feasi

    #return env_nums*6 bool tensor
    def _CollideCheck(self,B_e_des:torch.Tensor)->torch.Tensor:
        # LB LF LM RB RF RM
        # LM xy check if collide with lF xy LB xy
        collide_feasi_flag=torch.zeros(self.env_nums,6,dtype=torch.bool,device=self.device)
        #transfer to R frame
        R_e_des=self._B2R(B_e_des)
        # print("collide_feasi norm calclualtion:\n",R_e_des[:,0,0:2]-R_e_des[:,2,0:2])
        collide_feasi_flag[:,0]=torch.norm(R_e_des[:,0,0:2]-R_e_des[:,2,0:2],dim=1)>0.12
        collide_feasi_flag[:,1]=torch.norm(R_e_des[:,1,0:2]-R_e_des[:,2,0:2],dim=1)>0.12
        collide_feasi_flag[:,2]=collide_feasi_flag[:,0]&collide_feasi_flag[:,1] #feasible when both feet not collide

        collide_feasi_flag[:,3]=torch.norm(R_e_des[:,3,0:2]-R_e_des[:,5,0:2],dim=1)>0.12
        collide_feasi_flag[:,4]=torch.norm(R_e_des[:,4,0:2]-R_e_des[:,5,0:2],dim=1)>0.12
        collide_feasi_flag[:,5]=collide_feasi_flag[:,3]&collide_feasi_flag[:,4]

        # print(">>>>>>>_CollideCheck<<<<<<<<<<,\n, R_e_des:\n",R_e_des)
        # print("collide_feasi_flag:=",collide_feasi_flag)

        return collide_feasi_flag

    def _ContactDetection(self,next_B_e_des:torch.Tensor)->torch.Tensor:
        # swing reach the high point and z<-0.08
        # return (foot_contact_force!=0)
        return (next_B_e_des[...,2]<-0.12)&(self.swing_reach_point)
        # return (self.B_e_cur[...,2]<-0.084)&(self.swing_reach_point)
    
    def _AdsorbReleaseDetection(self,suction_forces)->torch.Tensor:
        # print("suction_forces:\n",self.suction_forces)
        stance_release_done=self.gaits&(suction_forces<self.cfg.control.adhesion.max_force*0.1)
        swing_adsorb_done=~self.gaits&(suction_forces>self.cfg.control.adhesion.max_force*0.85)
        # print("suction_forces:",suction_forces)
        # print("stance_release_done|swing_adsorb_done:\n",stance_release_done|swing_adsorb_done)
        return stance_release_done|swing_adsorb_done
        # return torch.ones(self.env_nums,6,dtype=torch.bool,device=self.device)

    def _R2B(self,R_e:torch.Tensor):
        B_e=R_e.clone()
        B_e[:,0:3,0:2]=self.body_shape[0:3,0:2]-R_e[:,0:3,0:2]
        B_e[:,3:6,0:2]=R_e[:,3:6,0:2]-self.body_shape[3:6,0:2]
        return B_e

    def _B2R(self,B_e:torch.Tensor):
        R_e=B_e.clone()
        R_e[:,0:3,0:2]=self.body_shape[0:3,0:2]-B_e[:,0:3,0:2]
        R_e[:,3:6,0:2]=self.body_shape[3:6,0:2]+B_e[:,3:6,0:2]
        return R_e


class ExpertGround(ExpertClimb):
    def __init__(self,cfg:HexGroundCfg,device,env_nums):
        super().__init__(cfg,device,env_nums)
        #不需要吸附力和吸附力大小
        self.adhesions=None
        #期望的关节角度位置共18个
        self.q_des = torch.zeros(env_nums,6,3,dtype=torch.float32,device=device)
        self.q_des_flat = self.q_des.view(env_nums*6,3)
        self.q_init = torch.zeros(18,dtype=torch.float32,device=device)
        #set initial foot end pos joint pos
        self.leg_names=['lb','lf','lm','rb','rf','rm']
        self.joint_names=['thigh','knee','ankle']
        i=0
        for leg_name in self.leg_names:
            for joint_name in self.joint_names:
                q_name='j_'+leg_name+'_'+joint_name
                if q_name in self.cfg.init_state.default_joint_angles.keys():
                    self.q_init[i]=self.cfg.init_state.default_joint_angles[q_name]
                    i+=1
        self.q_des[:]=self.q_init.view(6,3)
        self.kin.ForwardKin(self.q_init.view(6,3),self.B_e_init)

        self.B_e_des[:]=self.B_e_init.unsqueeze(0)

    def ProcessCommand(self, command:torch.Tensor, 
                       q_cur:torch.Tensor,
                       q_dot_cur:torch.Tensor,)->torch.Tensor:
        """
        @input: command [set_init,vx,vy,vz,omega], q_cur suction_force

        @output: q_des_flat env_nums*24
        """
        # print("expert reset command",command[0,0])
        #update current position of foot end in leg base frame
        q_cur_flat=q_cur.view(-1,3)
        q_dot_cur_flat = q_dot_cur.view(self.env_nums*6,3)
        self.kin.ForwardKin(q_cur_flat,self.B_e_cur_flat)


        reset_mask=command[:,0].to(torch.bool)
        reset_index=torch.where(reset_mask)[0]
        if reset_mask.any():
            #reset hex states
            self.set_init_done[reset_mask]=False
            self.swing_reach_point[reset_mask]=False
            self.stance_group_index[reset_mask]=0
            self.gaits[reset_mask]=0
            self.gaits[reset_index.unsqueeze(-1),self.A_group_index]=1
            # self.q_des[reset_index,...]=self.q_init[reset_index,...]
            self.B_e_des[reset_index,...]=self.B_e_init
            #judge footend if reach desired point
            reset_done_mask=(torch.norm((self.B_e_des-self.B_e_cur),p=2,dim=2)<0.1).all(dim=1)
            reset_done_mask.fill_(True)
            # print(torch.norm((self.B_e_des-self.B_e_cur),p=2,dim=2))
            self.set_init_done[reset_done_mask]=True
        moving_env_mask=command[:,1].to(torch.bool)|command[:,2].to(torch.bool)|command[:,4].to(torch.bool)
        if self.set_init_done.any() and moving_env_mask.any():
            # print("--------process command------------")
            self.GaitPlanning(command)
            self.CalJointPoses(q_cur_flat,q_dot_cur_flat)
        return self.q_des_flat.view(self.env_nums,-1)
    
    def CalJointPoses(self,q_cur_flat:torch.Tensor,q_dot_cur_flat):
        # self.q_des_flat = q_cur_flat.clone()
        # self.kin.InverseKin2(self.B_e_des_flat[:,0:3],self.q_des_flat)

        # damp_inv_jac_env: (env_nums*6)*3*3 q_cur_flat:(env_nums*6)*3
        damp_inv_jac_env=self.kin.DampInvJac(q_cur_flat) 
        pos_err=(self.B_e_des_flat[...,0:3]-self.B_e_cur_flat[...,0:3]).unsqueeze(-1) # (env_nums*6)*3*1
        self.q_des_flat[:,0:3]=q_cur_flat+32*((damp_inv_jac_env@pos_err).squeeze(-1))*self.dt #-0.008*q_dot_cur_flat

    #没有吸附力了，需要重新定义步态切换规划方式
    def GaitPlanning(self, command):
        # print("gaits=",self.gaits)
        #calculate Transform of robot body
        self._TargetTransInterp(command)
        #calculate next B_e_des env_nums*6*4
        next_B_e_des=self._CalB_e_des(command)

        # print("next_B_e_des\n",next_B_e_des)
        #check working range and collide feasible
        range_feasi=self._FeasiCheck(next_B_e_des) # env_nums*6 bool
        collide_feasi=self._CollideCheck(next_B_e_des) # env_nums*6 bool
        

        stance_done_env=torch.zeros(self.env_nums,dtype=torch.bool,device=self.device)
        swing_done_env=torch.zeros_like(stance_done_env,dtype=torch.bool,device=self.device)


        # select swing_done(finish contact) legs
        swing_done_mask=~self.gaits&self._ContactDetection(next_B_e_des) 

        # select env continue siwng
        have_siwng_continue_env_mask=swing_done_mask.sum(dim=1)<3
        if have_siwng_continue_env_mask.any():
            # set_all_mask=(~swing_done_mask)&(~self.gaits)&(~self.swing_reach_point.unsqueeze(1))
            # self.B_e_des[set_all_mask]=next_B_e_des[set_all_mask]

            # set_z_mask=(~collide_feasi|~range_feasi)&(have_siwng_continue_env_mask.unsqueeze(-1))
            set_all_mask=((~swing_done_mask)&(~self.gaits)&(collide_feasi&range_feasi)) | ((~self.gaits)&(~self.swing_reach_point))
            set_z_mask=(~swing_done_mask)&(~self.gaits)&(~collide_feasi|~range_feasi)

            self.B_e_des[...,2][set_z_mask]=next_B_e_des[...,2][set_z_mask]
            self.B_e_des[set_all_mask]=next_B_e_des[set_all_mask]
            # print("self.swing_reach_point",self.swing_reach_point)
            # print("set_all_mask",set_all_mask)
        # every leg touch ground and reach hight, set the env swing done
        swing_done_env_count=( (~self.gaits)&swing_done_mask&(self.swing_reach_point) ).sum(dim=1)
        swing_done_env[swing_done_env_count==3]=1
        # print("swing_done_mask",swing_done_mask)
        # print("swing_done_env",swing_done_env)

        #for env have swing done leg, if reach hight, set this env's stance done
        set_stance_done_env_mask=(swing_done_mask.any(dim=1))
        stance_done_env[set_stance_done_env_mask]=1 #stance done because swing leg touch ground
        # stance_mask=self.gaits
        # choose envs that all stance legs within range and collide free & not stance done
        # this way cannot keep the original dimension range_feasi[stance_mask]&collide_feasi[stance_mask]
        # cont numbers instead
        # choose gaits & collide free & within range
        range_collide_free_env_mask=(self.gaits&range_feasi&collide_feasi).sum(dim=1)==3

        #for those not within range and collide free, set stance done
        stance_done_env[~range_collide_free_env_mask]=1
        # print("stance_done_env",stance_done_env)
        #select envs that satisfy range collide free and not stance done(some stance done is caused by swing)
        set_stance_env_mask=range_collide_free_env_mask & (~stance_done_env)
        # print("range_collide_free_env_mask",range_collide_free_env_mask)
        # print("set_stance_env_mask",set_stance_env_mask)
        #set_stance_env_mask: env_nums bool; gaits: env_nums*6 bool 
        #use mask to choose which B_e_des to set
        #if set_stance_env_mask[i]=True, then the i-th env of stance leg need to be set B_e_des
        #calculate every column of set_stance_env_mask with(&) set_stance_env_mask, result mask to set B_e_des
        set_B_e_des_mask=self.gaits&(set_stance_env_mask.unsqueeze(1))
        # print("set_B_e_des_maks\n",set_B_e_des_mask)
        # print("before set B_e_des\n",self.B_e_des)
        if set_B_e_des_mask.any():
            self.B_e_des[set_B_e_des_mask]=next_B_e_des[set_B_e_des_mask]
        # print("after set B_e_des\n",self.B_e_des)
        #find swing done envs, and judge which of them finish adsorb

        if swing_done_env.any():
            self.stance_group_index[swing_done_env]=1-self.stance_group_index[swing_done_env]
            self.swing_reach_point[swing_done_env] = False
            self.gaits[swing_done_env] = 0
            swing_done_env_index=torch.where(swing_done_env)[0]
            expand_env_index=(swing_done_env_index.unsqueeze(1)).expand(-1,3)
            stance_set_index=self.gaits_groups[self.stance_group_index[swing_done_env_index]]
            self.gaits[expand_env_index,stance_set_index]=1
