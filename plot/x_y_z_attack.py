import argparse
import csv
import matplotlib.pyplot as plt
# plot the change in x axis along with time

def plot_x(attFile, benFile, idx):
    timestamp1, x1, y1, z1 = [], [], [], []
    timestamp2, x2, y2, z2 = [], [], [], []
    with open(benFile) as f1:
        data = csv.reader(f1, delimiter=',')
        for row in data:
            timestamp1.append(float(row[0]))
            x1.append(float(row[1]))
            y1.append(float(row[2]))
            z1.append(float(row[3]))

    with open(attFile) as f2:
        data2 = csv.reader(f2, delimiter=',')
        for row2 in data2:
            timestamp2.append(float(row2[0]))
            x2.append(float(row2[1]))
            y2.append(float(row2[2]))
            z2.append(float(row2[3]))
    
    # print("length of time: "+str(len(timestamp1)))
    # print("length of x1: "+str(len(x1)))
    # print(timestamp1[0:200])
    # print(x1[0:200])

    fig,axs = plt.subplots(3,1, figsize=(5, 10))
    axs[0].plot(timestamp1, x1, label = 'Original')
    axs[0].plot(timestamp2, x2, label = 'Attacked')
    # axs[0].set_title('X-axis change')
    axs[0].set(xlabel='time(s)', ylabel='x(m)')
    axs[0].legend()
    axs[1].plot(timestamp1, y1, label = 'Original')
    axs[1].plot(timestamp2, y2, label = 'Attacked')
    # axs[1].set_title('Y-axis change')
    axs[1].set(xlabel='time(s)', ylabel='y(m)')
    axs[1].legend()
    axs[2].plot(timestamp1, z1, label = 'Original')
    axs[2].plot(timestamp2, z2, label = 'Attacked')
    axs[2].set(xlabel='time(s)', ylabel='z(m)')
    # axs[2].set_title('Z-axis change')
    axs[2].legend()
    
    plt.savefig('./plot/drone'+str(idx)+'.png')
    plt.show()

# read the attack file and benign file from the command line
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', dest='att_file')
    parser.add_argument('-b', dest='ben_file')
    args = parser.parse_args()

    plot_x(args.att_file, args.ben_file, 1)
    
