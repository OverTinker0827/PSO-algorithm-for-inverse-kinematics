import numpy as np


class PSA:
    def __init__(self,dim,fk):
      
        self.dim=dim
        self.fk=fk
        self.cache={}
        self.last_pose=np.array([0,0,0,0,0,0])
    
    def optimize(self,iterations,N,a,b,c,target):
        dim=self.dim
        pos = np.random.uniform(-np.pi, np.pi, size=(N, dim))

        vel=np.array([[0 for _ in range(dim)] for _ in range(N)])
        best_pos=pos.copy()
        local_min_pos=pos.copy()
        for _ in range(iterations):
            
            best_pos=pos[np.argmin([self.evaluate(p,target) for p in pos])]


            for i in range(N):
                r1,r3=np.random.uniform(),np.random.uniform()
                vel[i]= (a*vel[i] + b*r1*(best_pos-pos[i]) + c*r3*(local_min_pos[i]-pos[i]))
                local_min_pos[i]=pos[i] if self.evaluate(pos[i],target)<self.evaluate(local_min_pos[i],target) else local_min_pos[i]

                pos[i]=pos[i]+vel[i]
        self.last_pose=best_pos
        return np.round(best_pos,2)
    def evaluate(self,pos,target):
        pos_tuple=tuple(np.round(pos,2))
        if pos_tuple in self.cache:
            return self.cache[pos_tuple]
        if np.any(np.abs(pos) > np.pi):
            return np.inf

        final_pos=self.fk(*pos)
        # print(np.linalg.norm(final_pos - np.array(self.target)))
        target_error = np.sum((final_pos - target)**2)
        motion_penalty = np.sum((pos- self.last_pose)**2)

        eval = 5.0 * target_error + 0.1 * motion_penalty

        self.cache[pos_tuple]=eval
        return eval
        
    
# solver=PSA([[1,0,0],[2,0,0],[3,0,0],[4,0,0]])
# best_parameters=solver.optimize(iterations=10000,N=200,a=0.70,b=1.2,c=1.9,target=[0,4,0])
# print(best_parameters)
# best_parameters=np.round(best_parameters,2)
# value=solver.evaluate(best_parameters,target=[0,4,0])
# print("Best Parameters:",best_parameters)
# print("Final Position Error:",value)
