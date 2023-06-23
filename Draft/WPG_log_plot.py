#!/usr/bin/python3

import matplotlib.pyplot as mpl
import csv

with open("../Log/WPG_log.csv") as file:
  csv_log = csv.reader(file)
  
  foot_point_t = []
  foot_point_x = []
  foot_point_y = []
  
  cog_point_x = []
  cog_point_y = []
  cog_veloc_x = []
  cog_veloc_y = []
  
  tag = 0
  for row in csv_log:
    if(tag < 6):
      foot_point_t.append(row[0]*1000)
      foot_point_x.append(row[1]*1000)
      foot_point_y.append(row[2]*1000)
    elif(tag > 6):
      cog_point_x.append(row[0]*1000)
      cog_point_y.append(row[1]*1000)
      cog_veloc_x.append(row[2]*1000)
      cog_veloc_y.append(row[3]*1000)
      print(row[1])
      
    tag += 1
    
  control_step = []
  for step in range(0, 4010, 10):
    control_step.append(step)
    
  mpl.plot(cog_point_y)
  # mpl.xticks([0, 1, 2, 3, 4000])
  # mpl.yticks([-0.06, -0.04, -0.02, 0.00, 0.02, 0.04, 0.06])
  mpl.show()