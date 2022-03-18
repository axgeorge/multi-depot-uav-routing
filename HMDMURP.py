# TODAY'S TSP

import numpy as np

# Class TSP helps us represent the HMDMURP as a Multiple One-In-A-Set ATSP

class TSP:

    def __init__(self, m, n, Veh, Tar): 
        self.m = m       # the number of vehicles
        self.n = n       # the number of targets
        self.Veh = Veh   # user defined array of vehicles, its coordinates, and types
        self.Tar = Tar   # user defined targets and their coordinates
        self.V = ["V_%s" % (elem + 1) for elem in range(m)]
        self.V_d = ["V_%s_d" % (elem + 1) for elem in range(m)]
        self.T = [["T_%s_%s" % ((row + 1), (elem + 1)) for elem in range(n)] for row in range(m)]
        self.V_i = [[self.V[i]] + self.T[i] + [self.V_d[i]] for i in range(m)]

    def create_zero_matrix(self, m, n):
        Z = []
        for row in range(m):
            Z.append([])
            for col in range(n):
                Z[row].append(0)
        return Z
        
    def find_E_i(self, veh_num):
        E_i = []
        for row in range(self.n + 2):
            E_i.append([])
            for elem in self.V_i[veh_num - 1]:
                E_i[row].append((self.V_i[veh_num - 1][row], elem))
        return E_i

    def find_C_i_initial(self):
        C_i = []
        for row in range(self.n + 2):
            C_i.append([])
            for col in range(self.n + 2):
                C_i[row].append('None')
        return C_i

    def find_C_initial(self):
        C = []
        for row in range(self.m*(self.n + 2)):
            C.append([])
            for col in range(self.m*(self.n + 2)):
                C[row].append('None')
        return C

    def append_cost_edge(self, C_i, veh_num, ver1, ver2, dist):
        E_i = self.find_E_i(veh_num)
        elem = -1
        i = 0
        while elem == -1:
            try:
                elem = E_i[i].index((ver1, ver2))
            except ValueError:
                i += 1   
        C_i[i][elem] = dist
        return C_i

    def distance(self, x_1, y_1, x_2, y_2, typ):
        dist = typ*((x_2 - x_1)**2 + (y_2 - y_1)**2)**0.5
        return dist

    def create_cost_i(self, veh_num, ret_cost):
        stack = []
        p_1 = self.Veh[veh_num - 1]
        stack.append(p_1)
        for row in range(len(self.Tar)):
            stack.append(self.Tar[row])
        C_i = self.find_C_i_initial()
        for row in range(self.n + 1):
            for col in range(self.n + 1):
                if row != col:
                    dist = self.distance(stack[row][0], stack[row][1], stack[col][0], stack[col][1], p_1[2])
                    if row < col:
                        C_i[row][col] = dist
                    if row > col:
                        C_i[row][col] = dist + ret_cost           
        return C_i
        
    def find_max_cost_edge_C_i(self, C_i):
        M = 0
        for i in range(len(C_i)):
            for j in range(len(C_i[i])):
                if C_i[i][j] != 'None':
                    if C_i[i][j] > M:
                        M = C_i[i][j]
        return M

    def create_multiple_one_in_a_set_ATSP(self, ret_cost):
        C = []
        for i in range(self.m):
            C_i = self.create_cost_i(i, ret_cost)
            C.append(C_i)
        return C 

# test functions helps me test some random examples and improve my understanding of how code is working

#A = [[(1, 2), (2, 3)], [(2, 3), (3, 4)], [(3, 4), (4, 5)]]
#print(A[2].index((4, 5)))
    


# test main functions

Veh = [[1, 2, 1], [2, 3, 2], [3, 4, 1]] #[4, 5, 2]]
Tar = [[10, 20], [30, 40]] #[40, 50], [50, 60]]

G = TSP(len(Veh), len(Tar), Veh, Tar)

print(G.V)
print(G.V_d)
print(G.T)
print(G.V_i)
print(G.find_E_i(1))
#C_1 = G.find_C_i_initial()
#G.append_cost_edge(C_1, 1, 'T_1_3', 'T_1_2', 5)
#C_2 = G.find_C_i_initial()
#G.append_cost_edge(C_2, 2, 'T_2_3', 'T_2_2', 5)
#C = [C_1, C_2]
#print(C)
#M = G.find_C_initial()
#print(M)
#print(G.create_cost_i(2, 10))
#print(G.distance(30, 40, 40, 50, 2))
#G.create_multiple_one_in_a_set_ATSP(10)


# Transform the multiple one-in-a-set ATSP to a single ATSP using the modified Noon Bean Transformation

def transformation_algorithm(G):
    C_t = G.find_C_initial()
    C = G.create_multiple_one_in_a_set_ATSP(10)
    MAX = [G.find_max_cost_edge_C_i(C[i]) for i in range(len(C))]
    M = 2*(G.n + G.m)*(max(MAX))

    for i in range(G.m):
        for j in range(1, (G.n + 2) - 1, 1):  
            C_t[i*(G.n + 2)][i*(G.n + 2) + j] = C[i][0][j] + M  # transformation step 1
            C_t[i*(G.n + 2)][i*(G.n + 2) + (G.n + 2) - 1] = M  # transformation step 2
    
    for i in range(G.m - 1):
        for j in range(G.n):
            C_t[i*(G.n + 2) + (G.n + 2) - 1][(i + 1)*(G.n + 2)] = 0 # transformation step 3

    C_t[G.m*(G.n + 2) - 1][0] = 0 # transformation step 4

    for i in range(G.m - 1):
        for j in range(1, (G.n + 2) - 1, 1):
            for k in range(1, (G.n + 2) - 1, 1):
                if (i*(G.n + 2) + j) != (i*(G.n + 2) + k):
                    C_t[i*(G.n + 2) + j][(i + 1)*(G.n + 2) + k] = C[i + 1][j][k] + M # transformation step 5
                else:
                    C_t[i*(G.n + 2) + j][(i + 1)*(G.n + 2) + k] = 0 # transformation step 7

                if ((G.m - 1)*(G.n + 2) + j) != ((G.m - 1)*(G.n + 2) + k):
                    C_t[(G.m - 1)*(G.n + 2) + j][0*(G.n + 2) + k] = C[0][j][k] + M # transformation step 6
                else:
                    C_t[(G.m - 1)*(G.n + 2) + j][0*(G.n + 2) + k] = 0 # transformation step 8

                C_t[i*(G.n + 2) + j][(i + 1)*(G.n + 2) + (G.n + 1)] = C[i + 1][j][0] + M # transformation step 9
                C_t[(G.m - 1)*(G.n + 2) + j][G.n + 1] = C[0][j][0] # transformation step 10


    print(C_t)
    print(C)
    print(M)
    
    
transformation_algorithm(G)

