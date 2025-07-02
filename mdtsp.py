# TODAY'S TSP

import string
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import os

# Create a file generator class for LKH

class LKH_file_generator:

    def __init__(self, C, filename_tsp, filename_par, filename_sol):
        self.C = C
        self.filename_tsp = filename_tsp
        self.filename_par = filename_par
        self.filename_sol = filename_sol

    def compile_row_string(self, a_row):
        return str(a_row).strip(']').strip('[').replace(',','')

    def create_TSP(self, name = 'test'): # here, the user inputs the cities, and their coords into the C matrix.
        with open(self.filename_tsp, 'w') as f:
            f.write('NAME : %s\n' % name)
            f.write('COMMENT : few cities test problem\n')
            f.write('TYPE : ATSP\n')
            f.write('DIMENSION : %d\n' % len(self.C))
            f.write('EDGE_WEIGHT_TYPE : EUC_2D\n')
            f.write('NODE_COORD_SECTION\n')
            for row in self.C:
                f.write('%d %d %d\n' % (row[0], row[1], row[2]))
            f.write('EOF\n')

    def create_cost_matrix_TSP(self, name = 'test_1'): # here, the user can input the cost matrix directly.
        with open(self.filename_tsp, 'w') as f:
            f.write('NAME : %s\n' % name)
            f.write('COMMENT : few cities test problem\n')
            f.write('TYPE : ATSP\n')
            f.write('DIMENSION : %d\n' % len(self.C))
            f.write('EDGE_WEIGHT_TYPE : EXPLICIT\n')
            f.write('EDGE_WEIGHT_FORMAT : FULL_MATRIX\n')
            f.write('EDGE_WEIGHT_SECTION\n')
            for row in self.C:
                f.write(self.compile_row_string(row) + '\n')
            f.write('EOF\n')

    def create_PAR(self, name = 'test.tsp', tour = 'testsol'):
        with open(self.filename_par, 'w') as f:
            f.write('PROBLEM_FILE = %s\n' % name)
            f.write('TOUR_FILE = %s\n' % tour)
            f.write('RUNS = 10')

    def create_cost_matrix_PAR(self, name = 'test_1.tsp', tour = 'test_1sol'):
        with open(self.filename_par, 'w') as f:
            f.write('PROBLEM_FILE = %s\n' % name)
            f.write('TOUR_FILE = %s\n' % tour)
            f.write('RUNS = 10')

    def read_sol(self):
        F = []
        with open(self.filename_sol) as f:
            for index, line in enumerate(f):
                if index > 5 and index < len(self.C) + 6:
                    F.append(int(line))
        return F      

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
        disjoint = 9999999
        C = []
        for row in range(self.m*(self.n + 2)):
            C.append([])
            for col in range(self.m*(self.n + 2)):
                C[row].append(disjoint)
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
        p_1 = self.Veh[veh_num]
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
                C_t[(G.m - 1)*(G.n + 2) + j][G.n + 1] = C[0][j][0] + M # transformation step 10

    for i in range(len(C_t)):
        for j in range(len(C_t)):
            C_t[i][j] = int(C_t[i][j]) # need to make all the entries integer values for LKH to work
    
    return C_t

# Plot the optimal tour to the HMDMURP   

def plot_transformed_LKH(G, C, F):

    Plot_Matrix = []
    Plot_Matrix_X = []
    Plot_Matrix_Y = []
    veh_x = []
    veh_y = []
    tar_x = []
    tar_y = []
    E_1 = []
    E_2 = []
    Veh = []
    Veh_depo = []
    Tar = []

    for i in range(G.m):
        Plot_Matrix.append([])
        Plot_Matrix_X.append([])
        Plot_Matrix_Y.append([])
        veh_x.append(G.Veh[i][0])
        veh_y.append(G.Veh[i][1])

    for i in range(G.n):
        tar_x.append(G.Tar[i][0])
        tar_y.append(G.Tar[i][1])

    for index in range(1, len(F), 1):
        E_1.append(C[F[index - 1] - 1][F[index] - 1])

    for i in range(G.m):   
        Veh.append(i*(G.n + 2) + 1)
        Veh_depo.append((i + 1)*(G.n + 2))

    for j in range(G.n):
        Tar.append([])
        for i in range(G.m):
            Tar[j].append((j + 2) + i*(G.n + 2))
    
    for elem in F:
        for i in range(len(Tar)):
            if elem in Tar[i]:
                E_2.append(i + 1)
        if elem in Veh:
            E_2.append('V_%s' %(Veh.index(elem) + 1))
        elif elem in Veh_depo:
            E_2.append('V_%s' %(Veh_depo.index(elem) + 1))

    for elem in E_2:
        if elem in G.V:
            row = G.V.index(elem)
            Plot_Matrix[row].append(elem)
            Plot_Matrix_X[row].append(G.Veh[row][0])
            Plot_Matrix_Y[row].append(G.Veh[row][1])
        else:
            Plot_Matrix[row].append(elem)
            Plot_Matrix_X[row].append(G.Tar[elem - 1][0])
            Plot_Matrix_Y[row].append(G.Tar[elem - 1][1])

    print(Veh)
    print(Veh_depo)
    print(Tar)
    print(E_2)
    print(Plot_Matrix)
    print(Plot_Matrix_X)
    print(Plot_Matrix_Y)
    print(tar_x)

    for i in range(len(Plot_Matrix)):
        plt.plot(Plot_Matrix_X[i], Plot_Matrix_Y[i], 'r')
    plt.scatter(veh_x, veh_y)
    plt.scatter(tar_x, tar_y)
    plt.title('Optimal Solution to HMDMURP')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.show()
    

# test main functions

Veh = [[1, 2, 1], [15, 60, 1.3], [40, 38, 1], [80, 10, 2]]
Tar = [[1, 3], [6, 5]] #, [15, 65], [20, 40], [80, 12]]

G = TSP(len(Veh), len(Tar), Veh, Tar)

print(G.V)
print(G.V_d)
print(G.T)
print(G.V_i)
print(G.find_E_i(1))    
C_t = transformation_algorithm(G)
print(C_t)

LKH_1 = LKH_file_generator(C_t, '/home/nykabhishek/George_Allen/LKH/LKH-2.0.9/test_1.tsp', 
'/home/nykabhishek/George_Allen/LKH/LKH-2.0.9/test_1.par', '/home/nykabhishek/George_Allen/LKH/LKH-2.0.9/test_1sol')
LKH_1.create_cost_matrix_TSP()
LKH_1.create_cost_matrix_PAR()

# try to automate the linux terminal operation

F = LKH_1.read_sol()
print(F)
plot_transformed_LKH(G, C_t, F)
