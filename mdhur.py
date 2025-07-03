"""
A Heuristic Solver for Multi-Depot Heterogeneous UAV Routing.
Reference: Today's Traveling Salesman Problem -- published in IEEE RAM Dec 2010. 
"""

import string
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import os

matplotlib.use('Qt5Agg')


class LKH_file_generator:
    """
    Instance file generator for LKH solver.
    """

    def __init__(self, instance, filename_tsp, filename_par, filename_sol):
        self.inst = instance
        self.filename_tsp = filename_tsp
        self.filename_par = filename_par
        self.filename_sol = filename_sol

    def compile_row_string(self, a_row):
        return str(a_row).strip(']').strip('[').replace(',','')

    def create_coord_TSP(self, name = 'test_coord'):
        """
        Represent instance as cities, and their coords.
        """ 
        with open(self.filename_tsp, 'w') as f:
            f.write('NAME : %s\n' % name)
            f.write('COMMENT : few cities test problem\n')
            f.write('TYPE : ATSP\n')
            f.write('DIMENSION : %d\n' % len(self.inst))
            f.write('EDGE_WEIGHT_TYPE : EUC_2D\n')
            f.write('NODE_COORD_SECTION\n')
            for row in self.inst:
                f.write('%d %d %d\n' % (row[0], row[1], row[2]))
            f.write('EOF\n')

    def create_cost_matrix_TSP(self, name = 'test_matrix'):
        """
        Represent instance as cost matrix.
        """ 
        with open(self.filename_tsp, 'w') as f:
            f.write('NAME : %s\n' % name)
            f.write('COMMENT : few cities test problem\n')
            f.write('TYPE : ATSP\n')
            f.write('DIMENSION : %d\n' % len(self.inst))
            f.write('EDGE_WEIGHT_TYPE : EXPLICIT\n')
            f.write('EDGE_WEIGHT_FORMAT : FULL_MATRIX\n')
            f.write('EDGE_WEIGHT_SECTION\n')
            for row in self.inst:
                f.write(self.compile_row_string(row) + '\n')
            f.write('EOF\n')

    def create_coord_PAR(self, name = 'test_coord.tsp', tour = 'test_coord_sol'):
        with open(self.filename_par, 'w') as f:
            f.write('PROBLEM_FILE = %s\n' % name)
            f.write('TOUR_FILE = %s\n' % tour)
            f.write('RUNS = 10')

    def create_cost_matrix_PAR(self, name = 'test_matrix.tsp', tour = 'test_matrix_sol'):
        with open(self.filename_par, 'w') as f:
            f.write('PROBLEM_FILE = %s\n' % name)
            f.write('TOUR_FILE = %s\n' % tour)
            f.write('RUNS = 10')

    def read_sol(self):
        F = []
        with open(self.filename_sol) as f:
            for index, line in enumerate(f):
                if index > 5 and index < len(self.inst) + 6:
                    F.append(int(line))
        return F      


class TSP:
    """
    Transform MDHUR to a Multiple One-Of-A-Set ATSP.
    """

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


def transformation_algorithm(G):
    """
    Transform multiple one-of-a-set ATSP to a single ATSP using the modified Noon Bean Transformation.
    """
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
   

def plot_transformed_LKH(G, C, F, route_col='gray', veh_col='k', tar_col='k'):
    """
    Extract and plot MDHUR solution from the LKH ATSP solution.
    """

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

    # print(Veh)
    # print(Veh_depo)
    # print(Tar)
    print(E_2)
    # print(Plot_Matrix)
    # print(Plot_Matrix_X)
    # print(Plot_Matrix_Y)
    # print(tar_x)

    for i in range(len(Plot_Matrix)):
        plt.plot(Plot_Matrix_X[i], Plot_Matrix_Y[i], color=route_col)
    plt.scatter(veh_x, veh_y, color=veh_col, label="UAV", marker="^", s=50)
    plt.scatter(tar_x, tar_y, color=tar_col, label="Target", marker="x", s=80)
    plt.title('UAV ROUTES')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid()
    plt.legend()
    plt.show()
    
