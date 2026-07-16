from torch import sin,cos,acos,atan,atan2,asin,pi,sqrt
import torch

class Kinematic:
    def __init__(self,l1,l2,l3,device='cpu'):
        self.l1=l1
        self.l2=l2
        self.l3=l3
        self.device=device

    def ForwardKin(self,joints,pos:torch.Tensor):
        """joints:[batch_size,3],pos:[batch_size,3]"""
        q1=joints[:,0]
        q2=joints[:,1]
        q3=joints[:,2]
        pos[:,0]=self.l1*cos(q1) + self.l2*cos(q1)*cos(q2) + self.l3*cos(q1)*cos(q2+q3)
        pos[:,1]=self.l1*sin(q1) + self.l2*sin(q1)*cos(q2) + self.l3*sin(q1)*cos(q2+q3)
        pos[:,2]=self.l2*sin(q2) + self.l3*sin(q2+q3)
    

    def DampInvJac(self,joints):
        """joints:[batch_size,3],parallel_num:batch_size return [batch_size,3,3]"""
        Jac=torch.zeros(joints.shape[0],3,3,dtype=torch.float32,device=self.device)
        q1=joints[:,0]
        q2=joints[:,1]
        q3=joints[:,2]
        Jac[:,0,0]=-self.l1*sin(q1)-( self.l2*cos(q2)+self.l3*cos(q2+q3) )*sin(q1)
        Jac[:,1,0]=self.l1*cos(q1)+( self.l2*cos(q2)+self.l3*cos(q2+q3) )*cos(q1)
        Jac[:,2,0]=0

        Jac[:,0,1]=-(self.l2*sin(q2)+self.l3*sin(q2+q3))*cos(q1)
        Jac[:,1,1]=-(self.l2*sin(q2)+self.l3*sin(q2+q3))*sin(q1)
        Jac[:,2,1]=self.l2*cos(q2)+self.l3*cos(q2+q3)

        Jac[:,0,2]=-self.l3*cos(q1)*sin(q2+q3)
        Jac[:,1,2]=-self.l3*sin(q1)*sin(q2+q3)
        Jac[:,2,2]=self.l3*cos(q2+q3)
        JJT=Jac@Jac.transpose(1,2)+torch.eye(3,3,dtype=torch.float32,device=self.device)*0.0001
        return Jac.transpose(1,2)@torch.inverse(JJT)

    def InverseKin1(self,pos:torch.Tensor,joints_cur:torch.Tensor):
        """use iterative jacobian to get desired joints, joints_cur is modified to the desired joints"""
        pos_cur=torch.zeros_like(pos,dtype=torch.float32,device=self.device)
        self.ForwardKin(joints_cur,pos_cur)
        for _ in range(1000):
            diff_norm=torch.linalg.norm(pos-pos_cur,dim=1)
            indices = torch.where(diff_norm < 0.005)[0]
            if indices.numel() == pos.shape[0]:
                # print("Inverse kinematics success, joints_cur\n",joints_cur)
                #every joints is solved
                return

            damp_inv_jacs=self.DampInvJac(joints_cur)
            # print(joints_cur,pos,pos_cur,damp_inv_jac)
            joints_cur.add_( 0.01*(damp_inv_jacs@((pos-pos_cur).unsqueeze(-1))).squeeze(-1) )
            self.ForwardKin(joints_cur,pos_cur)
        diff_norm=torch.linalg.norm(pos-pos_cur,dim=1)
        indices=torch.where(diff_norm>0.005)[0]
        if indices.numel() > 0:
            print("IK failed,indices\n:{}, pos_cur[indices]:\n{}".format(indices,pos_cur[indices]))
    
    def InverseKin2(self,pos:torch.Tensor,joints_cur:torch.Tensor):
        """use analytical solution to get desired joints, find the nearest solution to joints_cur
           pos:[batch_size,3], joints_cur:[batch_size,3], this will modify joints_cur"""
        x=pos[:,0]
        y=pos[:,1]
        z=pos[:,2]
        q1=atan2(y,x)
        # q1_possible=torch.stack([q1,q1+pi,q1-pi],dim=0)
        # joints_cur[:,0]=self._SelectCloestAngles(q1_possible,joints_cur[:,0])
        # min_index=torch.abs(q1_possible-joints_cur[:,0]).argmin(dim=0)
        # q1=torch.gather(q1_possible,dim=0,index=min_index.unsqueeze(0)).squeeze(0)


        q3=acos( ( (cos(q1)*x+sin(q1)*y-self.l1)**2 + z**2-self.l3**2-self.l2**2 )/( 2*self.l2*self.l3 ) )
        q3_possible=torch.stack([q3,-q3],dim=0)
        q3=self._SelectCloestAngles(q3_possible,joints_cur[:,2])
        # min_index=torch.abs(q3_possible-joints_cur[:,2]).argmin(dim=0)
        # q3=torch.gather(q3_possible,dim=0,index=min_index.unsqueeze(0)).squeeze(0)
        # q3 = q3 if abs(q3-joints_cur[:,2])<abs(-q3-joints_cur[:,2]) else -q3

        q2_back=atan2( (self.l3*sin(q3)),( self.l3*cos(q3)+self.l2) )
        q2_front=asin( z/sqrt( (self.l3*cos(q3)+self.l2)**2+(self.l3*sin(q3))**2 ) )
        # q2_b_possible=torch.stack([q2_back,q2_back+pi,q2_back-pi],dim=0)
        q2_f_possible=torch.stack([q2_front,-q2_front+pi,-q2_front-pi],dim=0) #3*batch_size?????????
        # q2_possible=(q2_f_possible.unsqueeze(0)-q2_b_possible.unsqueeze(1)).reshape(-1,pos.shape[0]) #3*3*batch_size
        q2_possible = q2_f_possible-q2_back

        # print("q2_f_possible\n",q2_f_possible)
        # print("q2_b_possible\n",q2_b_possible)
        q2=self._SelectCloestAngles(q2_possible,joints_cur[:,1])
        # min_index=torch.abs(q2_possible-joints_cur[:,1]).argmin(dim=0)
        # q2=torch.gather(q2_possible,dim=0,index=min_index.unsqueeze(0)).squeeze(0)
        joints_cur[:,0]=q1
        joints_cur[:,1]=q2
        joints_cur[:,2]=q3
        # print("q1={}\n; q2={}\n, q3={}\n".format(q1,q2_possible,q3_possible))

    def _SelectCloestAngles(self,possible_angles,current_angles):
        """possible_angles:[possible_num,batch_size],current_angles:[batch_size]
           return selected_angles:[batch_size]"""
        min_index=torch.abs(possible_angles-current_angles).argmin(dim=0)
        selected_angles=possible_angles[min_index,torch.arange(current_angles.shape[0])]
        # same as below, gather is suitable for large possible_num
        # selected_angles=torch.gather(possible_angles,dim=0,index=min_index.unsqueeze(0)).squeeze(0)
        return selected_angles
    
    
if __name__=="__main__":
    kin=Kinematic(0.072,0.13,0.17)
    # joints_cur=torch.zeros(2,3,dtype=torch.float32)
    # pos=torch.zeros(2,3,dtype=torch.float32)
    # joints_cur[:,0]=0.2
    # joints_cur[:,1]=-0.5
    # joints_cur[:,2]=0.6
    # kin.ForwardKin(joints_cur,pos)
    # print("fkin pos=",pos)

    pos_des=torch.tensor([[0.2,0.0,0.0]])
    joint_cur=torch.tensor([[0.0,1.0,-1.5]])
    kin.InverseKin1(pos_des,joint_cur)
    print("inv1 joints_cur=",joint_cur)
    kin.ForwardKin(joint_cur,pos_des)
    print("verify result, the kin res=",pos_des)

    joint_cur=torch.tensor([[0.0,1.0,-1.5]])
    kin.InverseKin2(pos_des,joint_cur)
    print("inv1 joints_cur=",joint_cur)
    kin.ForwardKin(joint_cur,pos_des)
    print("verify result, the kin res=",pos_des)
    # pos_des=torch.tensor([[0.2,0.08,0.04]])
    # kin.InverseKin1(pos_des,joint_cur)
    # print("inv1 joints_cur=",joint_cur)    
    # damp_inv_jac=kin.DampInvJac(joints_cur)
    # print("joints_cur\n",joints_cur,"damp_inv_jac\n",damp_inv_jac)
# # print("fkin pos=",pos)


# kin.InverseKin2(pos,joints_cur)
# print("inv1 joints_cur=",joints_cur)
# joints_cur=torch.zeros(1,3,dtype=torch.float32,device='cuda:0')
# joints_cur[:,0]=0.2
# joints_cur[:,1]=-0.7
# joints_cur[:,2]=-1.4
# pos=torch.zeros(1,3,dtype=torch.float32,device='cuda:0')
# # pos[:,0]=0.19
# # pos[:,1]=0
# # pos[:,2]=-0.08
# kin.ForwardKin(joints_cur,pos)
# print("fkin pos=",pos)
# joints_cur[:,0]=0
# joints_cur[:,1]=-0.74
# joints_cur[:,2]=-2.0
# # # joints_cur[0,0]=-1
# # # joints_cur[0,1]=2.0
# # # joints_cur[0,2]=-3.0
# kin.InverseKin2(pos,joints_cur)
# print("inv2 joints_cur=",joints_cur)
# kin.ForwardKin(joints_cur,pos)
# print("fkin pos=",pos)

# mat1=torch.rand(3,3,dtype=torch.float32,device='cuda:0')
# mat2=torch.rand(3,3,dtype=torch.float32,device='cuda:0')
# vec1=torch.rand(3,dtype=torch.float32,device='cuda:0')
# vec2=torch.rand(3,dtype=torch.float32,device='cuda:0')
# mats1=torch.stack([mat1,mat1],dim=0)
# mats2=torch.stack([mat2,mat2],dim=0)
# vecs1=torch.stack([vec1,vec1],dim=0)
# vecs2=torch.stack([vec2,vec2],dim=0)
# # print(mat1@vec1)
# print(vecs1)
# diff=torch.linalg.norm(vecs1-vecs2,dim=1)
# print(diff)
# print(torch.where(diff>0))
# print((mats1@vecs1.unsqueeze(-1)).squeeze(-1))



# joints_cur=torch.tensor([0.1,-1.2,-1.4],dtype=torch.float32,device='cuda:0')
# pos=kin.ForwardKin(joints_cur)
# joints_cur[0]=0.1
# joints_cur[1]=-1
# joints_cur[2]=-1.2
# print(kin.InverseKin2(pos,joints_cur))
