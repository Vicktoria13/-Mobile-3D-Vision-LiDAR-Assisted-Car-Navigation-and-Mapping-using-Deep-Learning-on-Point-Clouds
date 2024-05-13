# %%
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import glob
import pickle as pkl
from matplotlib.colors import LinearSegmentedColormap

matplotlib.rc('font', **{'family': 'serif', 'serif': ['cmr10']})

# %%
paths = ['/home/sdi-2023-01/Téléchargements/res_MLS_simple_10/logs/']

fig1 = plt.figure()
fig1.suptitle('Inlier ratios on raw correspondences', fontsize=14)
ax1 = fig1.add_subplot(111)
ax1.grid(color='grey', linestyle='--',)
ax1.set_xlabel('Inlier threshold [m]', fontsize=12)
ax1.set_ylabel('Inlier ratio [%]', fontsize=12)
ax1.set_xlim(0, 1)
ax1.set_ylim(0, 40)

# fig2 = plt.figure()
# fig2.suptitle('Inlier ratios after RANSAC', fontsize=14)
# ax2 = fig2.add_subplot(111)
# ax2.grid(color='grey', linestyle='--')
# ax2.set_xlabel('Inlier threshold [m]',fontsize=12)
# ax2.set_ylabel('Inlier ratio [%]',fontsize=12)
# ax2.set_xlim(0, 1)
# ax2.set_ylim(0, 100)

# fig3 = plt.figure()
# fig3.suptitle('Inlier ratios after ICP refinement', fontsize=14)
# ax3 = fig3.add_subplot(111)
# ax3.grid(color='grey', linestyle='--')
# ax3.set_xlabel('Inlier threshold [m]', fontsize=12)
# ax3.set_ylabel('Inlier ratio [%]', fontsize=12)
# ax3.set_xlim(0, 1)
# ax3.set_ylim(0, 100)


for k in range(len(paths)):
    print('Processing case {}'.format(paths[k]))
    raw_files = glob.glob(paths[k] + 'raw/*.pkl')
    raw_files.sort()

    rsc_files = glob.glob(paths[k] + 'filt/*.pkl')
    rsc_files.sort()

    icp_files = glob.glob(paths[k] + 'icp/*.pkl')
    icp_files.sort()

    print("len raw files  ", len(raw_files))

    print("len rsc_files  ", len(rsc_files))

    print("len icp_files  ", len(icp_files))

    stats_raw = []
    stats_rsc = []
    stats_icp = []


    for i in range(len(raw_files)):
        with open(raw_files[i], 'rb') as f:
            stats_raw.append(pkl.load(f))

        with open(rsc_files[i], 'rb') as f:
            stats_rsc.append(pkl.load(f))
        with open(icp_files[i], 'rb') as f:
            stats_icp.append(pkl.load(f))

    raw_ir = np.empty((len(raw_files), len(stats_raw[0]['Inlier thresholds'])))
    rsc_ir = np.empty((len(raw_files), len(stats_rsc[0]['Inlier thresholds'])))
    icp_ir = np.empty((len(raw_files), len(stats_icp[0]['Inlier thresholds'])))

    for i in range(len(raw_files)):
        raw_ir[i] = stats_raw[i]['Inlier ratios']
        rsc_ir[i] = stats_rsc[i]['Inlier ratios']
        icp_ir[i] = stats_icp[i]['Inlier ratios']

    mean_raw_ir = 100*np.mean(raw_ir, axis=0)
    std_raw_ir = 100*np.std(raw_ir, axis=0)

    mean_rsc_ir = 100*np.mean(rsc_ir, axis=0)
    std_rsc_ir = 100*np.std(rsc_ir, axis=0)

    mean_icp_ir = 100*np.mean(icp_ir, axis=0)
    std_icp_ir = 100*np.std(icp_ir, axis=0)

    mean_raw_error = np.mean(stats_raw[0]['Distances'])
    mean_rsc_error = np.mean(stats_rsc[0]['Distances'])
    mean_icp_error = np.mean(stats_icp[0]['Distances'])

    ax1.plot(stats_raw[0]['Inlier thresholds'], mean_raw_ir, label=f'Case {k+1} (mean error = {mean_raw_error:.3f})')
    ax1.fill_between(stats_raw[0]['Inlier thresholds'], mean_raw_ir - std_raw_ir, mean_raw_ir + std_raw_ir, alpha=0.1)
    ax1.legend()
    plt.savefig('/home/topo/Desktop/plots/fig1.svg')
    # ax2.plot(stats_rsc[0]['Inlier thresholds'], mean_rsc_ir, label=f'Case {k+1} (mean error = {mean_rsc_error:.3f})')
    # ax2.fill_between(stats_rsc[0]['Inlier thresholds'], mean_rsc_ir - std_rsc_ir, mean_rsc_ir + std_rsc_ir, alpha=0.3)
    # ax2.legend()
    # plt.savefig('/home/topo/Desktop/plots/fig2.svg')
    # ax3.plot(stats_icp[0]['Inlier thresholds'], mean_icp_ir, label=f'Case {k+1}  (mean error = {mean_icp_error:.3f})')
    # ax3.fill_between(stats_icp[0]['Inlier thresholds'], mean_icp_ir - std_icp_ir, mean_icp_ir + std_icp_ir, alpha=0.3)
    # ax3.legend()
    # plt.savefig('/home/topo/Desktop/plots/fig3.svg')

# %%
corr_num = np.empty((len(paths), len(icp_files)))
for k in range(len(paths)):
    icp_files = glob.glob(paths[k] + 'icp/*.pkl')
    icp_files.sort()
    for i in range(len(icp_files)):
        with open(icp_files[i], 'rb') as f:
            stats_raw = pkl.load(f)
        corr_num[k, i] = stats_raw['Number corr a'] + stats_raw['Number corr b']

# %%
#plot box plot of number of correspondences relative to number of correspondences of the first case
fig4 = plt.figure()
fig4.suptitle('Number of correspondences', fontsize=14)
ax4 = fig4.add_subplot(111)
ax4.grid(color='grey', linestyle='--')
ax4.set_xlabel('Case', fontsize=12)
ax4.set_ylabel('Number of correspondences', fontsize=12)
ax4.boxplot(corr_num.T)
ax4.set_xticklabels(['Baseline', 'ALS', 'BLK 1', 'BLK 2', 'BLK 3', 'SfM'])
plt.savefig('/home/topo/Desktop/plots/corr_num.svg')
# %%
rsc_files = glob.glob(paths[k] + 'filt/*.pkl')
rsc_files.sort()

icp_files = glob.glob(paths[k] + 'icp/*.pkl')
icp_files.sort()

# boxplot of correspondence errors, ransac vs icp
corr_err_rsc = np.empty((len(rsc_files)))
corr_err_icp = np.empty((len(icp_files)))

rsc_files = glob.glob(paths[-1] + 'filt/*.pkl')
rsc_files.sort()
icp_files = glob.glob(paths[-1] + 'icp/*.pkl')
icp_files.sort()
for i in range(len(rsc_files)):
    with open(rsc_files[i], 'rb') as f:
        stats_rsc = pkl.load(f)
    with open(icp_files[i], 'rb') as f:
        stats_icp = pkl.load(f)
    corr_err_rsc[i] = np.mean(stats_rsc['Distances'])
    corr_err_icp[i] = np.mean(stats_icp['Distances'])

fig5 = plt.figure()
fig5.suptitle('Mean correspondence errors', fontsize=14)
ax5 = fig5.add_subplot(111)
ax5.grid(color='grey', linestyle='--')
ax5.set_xlabel('Case', fontsize=12)
ax5.set_ylabel('Mean correspondence error [m]', fontsize=12)
ax5.boxplot([corr_err_rsc, corr_err_icp])
ax5.set_xticklabels(['RANSAC', 'ICP'])
plt.savefig('/home/topo/Desktop/plots/corr_err_rsc.svg')
# %%

path_baseline = '/media/topo/Data/Data/MLS/Dataset_B/B3_CalField3/02_P2P_grandslam/logs/filt/'
path_sfm = '/media/topo/Data/Data/MLS/Dataset_B/B3_CalField3/05_coarse2fine_retrain_eval/logs/test/icp/'

glob_baseline = glob.glob(path_baseline + '*.pkl')
glob_sfm = glob.glob(path_sfm + '*.pkl')


stats_baseline = {}
stats_sfm = {}

stats_baseline['Distances'] = np.empty((len(glob_baseline)))
stats_sfm['Distances'] = np.empty((len(glob_sfm)))
stats_baseline['Number corr tot'] = np.empty((len(glob_baseline)))
stats_sfm['Number corr tot'] = np.empty((len(glob_sfm)))
stats_baseline['id_1'] = np.empty((len(glob_baseline)))
stats_sfm['id_1'] = np.empty((len(glob_sfm)))
stats_baseline['id_2'] = np.empty((len(glob_baseline)))
stats_sfm['id_2'] = np.empty((len(glob_sfm))    )

for i in range(len(glob_baseline)):
    with open(glob_baseline[i], 'rb') as f:
        stats = pkl.load(f)
    stats_baseline['Distances'][i] = np.mean(stats['Distances'])
    stats_baseline['Number corr tot'][i] = stats['Number corr a'] + stats['Number corr b']
    id_tile = stats["Tile number"]
    stats_baseline['id_1'][i] = id_tile[0]
    stats_baseline['id_2'][i] = id_tile[1]

for i in range(len(glob_sfm)):
    with open(glob_sfm[i], 'rb') as f:
        stats = pkl.load(f)
    stats_sfm['Distances'][i] = np.mean(stats['Distances'])
    stats_sfm['Number corr tot'][i] = stats['Number corr a'] + stats['Number corr b']
    id_tile = stats["Tile number"]
    stats_sfm['id_1'][i] = id_tile[0]
    stats_sfm['id_2'][i] = id_tile[1]

# %%
red = '#B51F1F'
blue = '#007480'
grey = '#CAC7C7'

colors = [blue, grey, red]


cmap = LinearSegmentedColormap.from_list('gauss', colors, N=50)
#plot color bar for the colormap
fig = plt.figure()
ax = fig.add_subplot(111)
cbar = matplotlib.colorbar.ColorbarBase(ax, cmap=cmap, orientation='horizontal')
cbar.set_label('Mean correspondence error [m]')
plt.savefig('/home/topo/Desktop/plots/colorbar.svg')

# %%
# matshow of mean correspondence errors for baseline and sfm cases with id ranging from 0 to 10

error_matrice_baseline = np.zeros((19, 19))
error_matrice_baseline[:] = np.nan

error_matrice_sfm = np.zeros((19, 19))
error_matrice_sfm[:] = np.nan


for i in range(len(stats_baseline['Distances'])):
    error_matrice_baseline[int(stats_baseline['id_1'][i]), int(stats_baseline['id_2'][i])] = stats_baseline['Distances'][i]

for i in range(len(stats_sfm['Distances'])):
    error_matrice_sfm[int(stats_sfm['id_1'][i]), int(stats_sfm['id_2'][i])] = stats_sfm['Distances'][i]


fig = plt.figure()
ax = fig.add_subplot(111)
cax = ax.matshow(error_matrice_baseline, cmap=cmap, vmin=0, vmax=0.15)
fig.colorbar(cax)
plt.xlabel('Tile id 1')
plt.ylabel('Tile id 2')
plt.gca().xaxis.tick_bottom()
plt.title('Mean correspondence error, baseline L2L')
plt.savefig('/home/topo/Desktop/plots/error_matrice_baseline.svg')

fig = plt.figure()
ax = fig.add_subplot(111)
cax = ax.matshow(error_matrice_sfm, cmap=cmap, vmin=0, vmax=0.15)
fig.colorbar(cax)
plt.xlabel('Tile id 1')
plt.ylabel('Tile id 2')
plt.gca().xaxis.tick_bottom()
plt.title('Mean correspondence error, updated L2L')
plt.savefig('/home/topo/Desktop/plots/error_matrice_sfm.svg')

#do same for number of correspondences
corr_matrice_baseline = np.zeros((19, 19))
corr_matrice_baseline[:] = np.nan

corr_matrice_sfm = np.zeros((19, 19))
corr_matrice_sfm[:] = np.nan

for i in range(len(stats_baseline['Number corr tot'])):
    corr_matrice_baseline[int(stats_baseline['id_1'][i]), int(stats_baseline['id_2'][i])] = stats_baseline['Number corr tot'][i]

for i in range(len(stats_sfm['Number corr tot'])):
    corr_matrice_sfm[int(stats_sfm['id_1'][i]), int(stats_sfm['id_2'][i])] = stats_sfm['Number corr tot'][i]

#set colorbar to white if below 25 correspondences

cmap_invert = LinearSegmentedColormap.from_list('gauss', colors[::-1], N=50)
fig = plt.figure()
ax = fig.add_subplot(111)
cax = ax.matshow(corr_matrice_baseline, cmap=cmap_invert, vmin=25, vmax=500)
fig.colorbar(cax)
#set axis labels and title
plt.xlabel('Tile id 1')
plt.ylabel('Tile id 2')
#xtick at bottom of matrix instead of top
plt.gca().xaxis.tick_bottom()
plt.title('Number of correspondences, baseline L2L')
plt.savefig('/home/topo/Desktop/plots/corr_matrice_baseline.svg')

fig = plt.figure()
ax = fig.add_subplot(111)
cax = ax.matshow(corr_matrice_sfm, cmap=cmap_invert, vmin=25, vmax=500)
fig.colorbar(cax)
plt.xlabel('Tile id 1')
plt.ylabel('Tile id 2')
plt.gca().xaxis.tick_bottom()
plt.title('Number of correspondences, updated L2L')
plt.savefig('/home/topo/Desktop/plots/corr_matrice_sfm.svg')


# %%