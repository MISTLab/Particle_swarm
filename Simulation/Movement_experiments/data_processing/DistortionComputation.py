import numpy as np
import pandas as pd
import math

def getCoords(items):
  for key, value in items:
    if key.startswith('x'):
      x = value
    if key.startswith('y'):
      y = value
    if key.startswith('z'):
      z = value
  return x,y,z

def norm(x, y, z):
  return math.sqrt(x**2 + y**2 + z**2)

# get from dict: https://stackoverflow.com/questions/10795973/python-dictionary-search-values-for-keys-using-regular-expression#10796073
def dist(row1, row2):
  x,y,z = getCoords(row1.iteritems())
  x2,y2,z2 = getCoords(row2.iteritems())
  return (x-x2), (y-y2), (z-z2)

def getKwgsCoords(**kwgs):
  return getCoords(kwgs.items())

def getSeparations(df):
  robot_separations = {}
  i = 0
  j = 0
  for group_name_i, df_group_i in df.groupby(np.arange(len(df.columns)) // 3, axis=1):
    print("iteration i: ", i)
    robot_separations[i] = {}
    j = 0
    for group_name_j, df_group_j in df.groupby(np.arange(len(df.columns)) // 3, axis=1):
      if i != j:
        print("iteration j: ", j)
        robot_separations[i][j] = {}
        for row_index_i, row_i in df_group_i.iterrows():
          for row_index_j, row_j in df_group_j.iterrows():
            print("rows: ", row_i, row_j)
            x,y,z = dist(row_i, row_j)
            robot_separations[i][j]['x'] = x
            robot_separations[i][j]['y'] = y
            robot_separations[i][j]['z'] = z
      j = j+1
      # print(row_index, '\n', " here is one row ", row, " end of row.")
    i = i+1
    return robot_separations


def main():
  data = pd.read_csv (r'positions.csv')   
  df = pd.DataFrame(data)
  df.drop('time', axis=1, inplace=True)
  df.drop(df.filter(regex='robotId').columns, axis=1, inplace=True)

  stable_points = df.iloc[50:51] # Review Change to range at which it is stable rigid
  stable_points.apply(lambda x: x.mean(), axis=1)
  final_points = df.iloc[70:71] # Review Change to range at which we want to compare to
  final_points.apply(lambda x: x.mean(), axis=1)
  stable_points.index = range(1)
  final_points.index = range(1)
  print(stable_points)

  # Extract difference in position between each robot, at stable and final positions
  print(stable_points.groupby(np.arange(len(stable_points.columns)) // 3, axis=1).groups)
  initial_separations = getSeparations(stable_points)
  final_separations = getSeparations(final_points)

  # Get change in these separations,
  #  by getting difference b/w all separations and
  #  then combining, through an average across all subtraction results
  # TODO




# a different way to group by robotId (https://stackoverflow.com/questions/48299874/how-to-groupby-column-headers-using-a-regex)
# # df.groupby(df.columns.str.extract('(^robotId:\d*)'), axis=1).groups






  # # Naive way of getting difference of absolute positions
  # diff = stable_points.sub(final_points)
  # print(diff)

  # use lambda to get norm on each robots position. Use https://stackoverflow.com/questions/1993727/expanding-tuples-into-arguments
  # diff_series = diff.groupby(np.arange(len(diff.columns)) // 3, axis=1).apply(lambda df:norm(*getKwgsCoords(**df))) # .transform(lambda x: dist(x, 'f', 'a', 'b', 'c'))
  # print(diff_series)

  # diff = diff_series.to_frame()
  # print(diff)

  # diff_series = diff_series.mean()
  # print(diff_series)


main()



# Not recommended
# for index, row in df.iterrows():
#     print(row['c1'], row['c2'])

# Rolling average every n items and then take the averaged val
# https://stackoverflow.com/questions/36810595/calculate-average-of-every-x-rows-in-a-table-and-create-new-table
# avg = df.rolling(5).mean() 
# avg = avg.iloc[::5, :]
# print(avg)
