import numpy as np
import json
import os
import pandas as pd
import matplotlib.pyplot as plt

def get_logs_of_case(idx_case):
    base_dir = '/home/lwcubuntu/workspaces/topoexpsearch/src/topoexpsearch_meas/log/' + str(idx_case) + '/'
    data_list = []
    for file in os.listdir(base_dir):
        full_filename = "%s/%s" % (base_dir, file)
        with open(full_filename, 'r') as fi:
            dict = json.load(fi)
            data_list.append(dict)

    # sort data_list by 'idx'
    data_list = sorted(data_list, key=lambda k: k['idx'])
    return data_list

def hist_dist_vs_targetarea(idx_data, P_E, res):
    # c_t : target class index
    data_list = get_logs_of_case(idx_data)

    # get list of travel distance
    TD = [data['travel_dist'] for data in data_list]

    # cumulated area of categories
    c_t = np.argmax(P_E)+1
    A_t = [data['cells_per_categories'][str(c_t)]*(res**2) for data in data_list]

    return TD, A_t

def mc_hist_dist_vs_targetarea(idx_list, P_E, res):
    TD_list = []
    A_t_list = []
    A_t_max = 0
    TD_max = 0
    for idx_data in idx_list:
        TD, A_t = hist_dist_vs_targetarea(idx_data, P_E, res)
        TD_list.append(TD)
        A_t_list.append(A_t)
        if A_t_max < A_t[-1]:
            A_t_max = A_t[-1]
        if TD_max < TD[-1]:
            TD_max = TD[-1]

    TD_refined = []
    A_t_list_refined = [[] for i in range(len(idx_list))]

    d_TD = 1
    for d in range(0, int(np.ceil(TD_max)), d_TD):
        TD_refined.append(d)
        for i_mc in range(0,len(idx_list)):
            i_now = [i_d for i_d, data in enumerate(TD_list[i_mc]) if data <= d][-1]
            A_t_list_refined[i_mc].append(A_t_list[i_mc][i_now])

    return TD_refined, A_t_list_refined

def plot_mc_hist_dist_vs_targetarea(MBD_datalist, FSE_list, hector_list, P_E, xlim, sigma, case):
    d_max = xlim[1]

    # MBD
    TD_MBD, A_t_mc_MBD = mc_hist_dist_vs_targetarea(MBD_datalist, P_E, res)
    i_MBD_max = [i_d for i_d, data in enumerate(TD_MBD) if data <= d_max][-1]
    A_t_mc_MBD_max = []

    list_nan = []
    for i_mc, A_t_MBD in enumerate(A_t_mc_MBD):
        A_t_mc_MBD_max.append(A_t_MBD[i_MBD_max])
        if A_t_MBD[i_MBD_max] == 0:
            list_nan.append(i_mc)

    # FSE
    TD_FSE, A_t_mc_FSE = mc_hist_dist_vs_targetarea(FSE_list, P_E, res)

    # hector
    TD_hector, A_t_mc_hector = mc_hist_dist_vs_targetarea(hector_list, P_E, res)

    # # # delete nan index
    # for i_nan in list_nan[::-1]:
    #     A_t_mc_MBD.pop(i_nan)
    #     A_t_mc_FSE.pop(i_nan)
    #     A_t_mc_hector.pop(i_nan)
    #     A_t_mc_MBD_max.pop(i_nan)

    # mean
    mean_A_t_mc_MBD = np.mean(A_t_mc_MBD, axis=0)
    mean_A_t_mc_FSE = np.mean(A_t_mc_FSE, axis=0)
    mean_A_t_mc_hector = np.mean(A_t_mc_hector, axis=0)

    std_A_t_mc_MBD = np.std(A_t_mc_MBD, axis=0)
    std_A_t_mc_FSE = np.std(A_t_mc_FSE, axis=0)
    std_A_t_mc_hector = np.std(A_t_mc_hector, axis=0)

    max_mean_A_t_mc_MBD = mean_A_t_mc_MBD[i_MBD_max]

    # normalize
    mean_A_t_mc_MBD = [original/max_mean_A_t_mc_MBD for original in mean_A_t_mc_MBD]
    std_A_t_mc_MBD = [sigma*original/max_mean_A_t_mc_MBD for original in std_A_t_mc_MBD]

    mean_A_t_mc_FSE = [original/max_mean_A_t_mc_MBD for original in mean_A_t_mc_FSE]
    std_A_t_mc_FSE = [sigma*original/max_mean_A_t_mc_MBD for original in std_A_t_mc_FSE]

    mean_A_t_mc_hector = [original/max_mean_A_t_mc_MBD for original in mean_A_t_mc_hector]
    std_A_t_mc_hector = [sigma*original/max_mean_A_t_mc_MBD for original in std_A_t_mc_hector]

    # plt.figure(dpi=1200)
    plt.plot(TD_MBD, mean_A_t_mc_MBD, 'r')
    plt.plot(TD_FSE, mean_A_t_mc_FSE, 'g')
    plt.plot(TD_hector, mean_A_t_mc_hector, 'b')

    plt.fill_between(TD_MBD, [a-b for a,b in zip(mean_A_t_mc_MBD, std_A_t_mc_MBD)], [a+b for a,b in zip(mean_A_t_mc_MBD, std_A_t_mc_MBD)], color='r', alpha=0.1)
    plt.fill_between(TD_FSE, [a-b for a,b in zip(mean_A_t_mc_FSE, std_A_t_mc_FSE)], [a+b for a,b in zip(mean_A_t_mc_FSE, std_A_t_mc_FSE)], color='g', alpha=0.1)
    plt.fill_between(TD_hector, [a-b for a,b in zip(mean_A_t_mc_hector, std_A_t_mc_hector)], [a+b for a,b in zip(mean_A_t_mc_hector, std_A_t_mc_hector)], color='b', alpha=0.1)

    fontsize = 15
    plt.xlim(xlim[0], xlim[1])
    plt.ylim(bottom = 0, top = 1)
    plt.xticks(fontsize=fontsize)
    plt.yticks(fontsize=fontsize)
    plt.xlabel('travel distance', fontsize=fontsize)
    plt.ylabel('$A_{c_t}$ covarage', fontsize=fontsize)
    plt.grid(True)
    plt.gca().set_aspect(30*xlim[1]/100)
    plt.savefig('case'+str(case)+'.png', dpi=1200)
    plt.show()

def get_score(idx_case, P_E, d_f, gamma, res):
    data_list = get_logs_of_case(idx_case)

    # get list of travel distance
    TD = [data['travel_dist'] for data in data_list]

    # cumulated area of categories
    N_c = len(P_E)
    A_C = {}
    for c in range(1,N_c+1):
        A_C[c] = [data['cells_per_categories'][str(c)] for data in data_list]

    score = 0.0
    i_max_P = np.argmax(P_E)
    for i, d in enumerate(TD):
        if d < d_f:
            for c in range(1,N_c+1):
                score = score + P_E[c-1]*gamma**(1.0+d)*max(A_C[c][i] - A_C[c][i-1], 0.0)*(res**2)
            area = sum([A_C[c][i] for c in range(1, N_c + 1)])
            area_maxP = A_C[i_max_P + 1][i]

    return score, area, area_maxP


def table_of_score(idx_list_MBD_GSIM, idx_list_FSE_GSIM, idx_list_hector, d_f_list, P_E, gamma, res, N_s ):
    sum_score_list = [0, 0, 0]
    min_score_list = [10000000, 10000000, 10000000]
    max_score_list = [0, 0, 0]
    print('name  : ' + 'MBD-GSIM,       FSE-GSIM,       hector,       MBD-GSIM/FSE-GSIM,  MBD-GSIM/hector')
    print('----------------------------------------------------------------------------------------')
    for i_c, (idx_MBD_GSIM, idx_FSE_GSIM, idx_hector) in enumerate(
            zip(idx_list_MBD_GSIM, idx_list_FSE_GSIM, idx_list_hector)):
        d_f = d_f_list[i_c]
        score_MBD_GSIM, area_MBD_GSIM, area_maxP_MBD_GSIM = get_score(idx_MBD_GSIM, P_E, d_f, gamma, res)
        score_FSE_GSIM, area_FSE_GSIM, area_maxP_FSE_GSIM = get_score(idx_FSE_GSIM, P_E, d_f, gamma, res)
        score_hector,   area_hector,   area_maxP_hector   = get_score(idx_hector,   P_E, d_f, gamma, res)
        print('score : ' + str(score_MBD_GSIM) + ' , ' + str(score_FSE_GSIM) + ' , ' + str(score_hector) + ' , ' +
              str(score_MBD_GSIM / score_FSE_GSIM) + ' , ' + str(score_MBD_GSIM / score_hector))
        # print('area  : ' + str(area_MBD_GSIM) + '      ,       ' + str(area_FSE_GSIM) + '      ,       ' + str(
        #     area_hector))
        # print('areaP : ' + str(area_maxP_MBD_GSIM) + '      ,       ' + str(
        #     area_maxP_FSE_GSIM) + '      ,       ' + str(area_maxP_hector))


        sum_score_list = [sum_score_list[0] + score_MBD_GSIM, sum_score_list[1] + score_FSE_GSIM,
                          sum_score_list[2] + score_hector]
        min_score_list = [min(min_score_list[0], score_MBD_GSIM), min(min_score_list[1], score_FSE_GSIM),
                          min(min_score_list[2], score_hector)]
        max_score_list = [max(max_score_list[0], score_MBD_GSIM), max(max_score_list[1], score_FSE_GSIM),
                          max(max_score_list[2], score_hector)]
    mean_score_list = [sum_case / N_s for sum_case in sum_score_list]
    print('----------------------------------------------------------------------------------------')
    print('mean  : ' + str(mean_score_list[0]) + ' , ' + str(mean_score_list[1]) + ' , ' + str(
        mean_score_list[2]) + ' , ' + str(mean_score_list[0] / mean_score_list[1]) + ' , ' + str(
        mean_score_list[0] / mean_score_list[2]))
    print('min   : ' + str(min_score_list[0]) + ' , ' + str(min_score_list[1]) + ' , ' + str(
        min_score_list[2]) + ' , ' + str(min_score_list[0] / min_score_list[1]) + ' , ' + str(
        min_score_list[0] / min_score_list[2]))
    print('max   : ' + str(max_score_list[0]) + ' , ' + str(max_score_list[1]) + ' , ' + str(
        max_score_list[2]) + ' , ' + str(max_score_list[0] / max_score_list[1]) + ' , ' + str(
        max_score_list[0] / max_score_list[2]))

# score 1
# params
gamma = 0.99
res = 0.05
N_s = 5

print('\n========================================================================================')
idx_MBD_GSIM_list = list(range(15,20))
idx_FSE_GSIM_list = list(range(20,25))
idx_hector_list = list(range(25,30))
d_f_list = [100, 100, 100, 100, 100]
P_E = [0.025, 0.025, 0.025, 0.025, 0.9]
table_of_score(idx_MBD_GSIM_list, idx_FSE_GSIM_list, idx_hector_list, d_f_list, P_E, gamma, res, N_s)

print('\n========================================================================================')
idx_MBD_GSIM_list = list(range(30,35))
idx_FSE_GSIM_list = list(range(35,40))
idx_hector_list = list(range(20,25))
d_f_list = [40, 40, 40, 40, 40]
P_E = [0.9, 0.025, 0.025, 0.025, 0.025]
table_of_score(idx_MBD_GSIM_list, idx_FSE_GSIM_list, idx_hector_list, d_f_list, P_E, gamma, res, N_s)

print('\n========================================================================================')
idx_MBD_GSIM_list = list(range(0,5))
idx_FSE_GSIM_list = list(range(5,10))
idx_hector_list = list(range(10,15))
d_f_list = [60, 60, 60, 60, 60]
P_E = [0.025, 0.025, 0.025, 0.025, 0.9]
table_of_score(idx_MBD_GSIM_list, idx_FSE_GSIM_list, idx_hector_list, d_f_list, P_E, gamma, res, N_s)

print('\n========================================================================================')
idx_MBD_GSIM_list = list(range(40,45))
idx_FSE_GSIM_list = list(range(45,50))
idx_hector_list = list(range(10,15))
d_f_list = [40, 40, 40, 40, 40]
P_E = [0.9, 0.025, 0.025, 0.025, 0.025]
table_of_score(idx_MBD_GSIM_list, idx_FSE_GSIM_list, idx_hector_list, d_f_list, P_E, gamma, res, N_s)


# score 2
case_to_run = 4
sigma = 0

# case 1
if case_to_run == 1:
    xlim = (0, 100)
    P_E = [0.025, 0.025, 0.025, 0.025, 0.9]
    MBD_datalist = list(range(15,20)) # MBD
    FSE_list = list(range(20,25))    # FSE
    hector_list = list(range(25,30)) # hector

# case 2
if case_to_run == 2:
    xlim = (0, 40)
    P_E = [0.9, 0.025, 0.025, 0.025, 0.025]
    MBD_datalist = list(range(30,35))  # MBD
    FSE_list = list(range(35,40))      # FSE
    hector_list = list(range(20,25))   # hector

# case 3
if case_to_run == 3:
    xlim = (0,60)
    P_E = [0.025, 0.025, 0.025, 0.025, 0.9]
    MBD_datalist = list(range(0,5)) # MBD
    FSE_list = list(range(5,10)) # FSE
    hector_list = list(range(10,15)) # hector

# case 4
if case_to_run == 4:
    xlim = (0, 40)
    P_E = [0.9, 0.025, 0.025, 0.025, 0.025]
    MBD_datalist = list(range(40,45)) # MBD
    FSE_list = list(range(45,50)) # FSE
    hector_list = list(range(10,15)) # hector

plot_mc_hist_dist_vs_targetarea(MBD_datalist, FSE_list, hector_list, P_E, xlim, sigma=sigma, case=case_to_run)