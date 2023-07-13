import math
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
# plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg' # Linux only

num_sheep = 300
# file_name = 'pos_300_2'

files = [f for f in os.listdir('.') if os.path.isfile(f) and f.endswith(".csv")]
files = [f.split('.')[0] for f in files]
print(files)

def main(file_name):
    data = pd.read_csv (r'' + file_name + '.csv')   
    df = pd.DataFrame(data)

    # Only use some rows
    print("steps ", df.shape[0])
    df = df[-5000:]

    # Clear out nondata columns
    df.drop('time', axis=1, inplace=True)
    df.drop(df.filter(regex='robotId').columns, axis=1, inplace=True)

    # Take every 3 cols (xyz) and combine into tuples (https://stackoverflow.com/questions/34052001/grouping-a-dataframe-and-applying-tuple)
    positionalDf = df.groupby(np.arange(len(df.columns)) // 3, axis=1).apply(lambda x: pd.Series([tuple(i) for i in x.values]))
    shape = positionalDf.shape
    num_sheep = shape[1] - shape[1] % 50
    print("Running: ", file_name, " size: ", shape[0], " bots: ", shape[1], " sheep: ", num_sheep, end =" ")

    fig = plt.figure()  
    axis = plt.axes(xlim =(-6, 15),  
                    ylim =(-6, 8))
    # ax.set(xlim=(-3, 3), ylim=(-1, 1))
    plt.style.use("ggplot")

    positions_start = positionalDf[0:0+1].to_numpy().tolist()[0]
    sheep_pos_start = positions_start[:num_sheep]
    dog_pos_start = positions_start[num_sheep:]
    sheep_scat = axis.scatter(*( list(zip(*sheep_pos_start))[:-1] ), color='red')
    dog_scat = axis.scatter(*( list(zip(*sheep_pos_start))[:-1] ), color='green')

    def animate(t):
        positions = positionalDf[t:t+1].to_numpy().tolist()[0]
        positions = [x[0:2] for x in positions]
        sheep_pos_start = positions[:num_sheep]
        dog_pos_start = positions[num_sheep:]
        sheep_scat.set_offsets(sheep_pos_start)
        dog_scat.set_offsets(dog_pos_start)
        fig.suptitle(t, fontsize=12)

    # plt.draw()
    # plt.show()
    positions_over_time = animation.FuncAnimation(fig, animate, interval=5, frames=(shape[0] - 1))
    positions_over_time.save(file_name + '.mp4')
    print("frames: ", (shape[0] - 1))

    # Linux only 
    # writer = animation.FFMpegWriter(fps=30, codec='gif') 
    # positions_over_time.save('positions.mp4', writer = writer) fps = 30

for f in files:
    main(f)