import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib

def add_line (path) :
	for i in range (0,len(path)):
		plt.plot(path[i][0],path[i][1],'.',color='red',markersize=10)
	
	for i in range(0,len(path)-1):
		plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],color='b')
	
	plt.axis('scaled')
	# plt.show()
	
def add_complicated_line (path,lineStyle,lineColor,lineLabel) :
	for i in range (0,len(path)):
		plt.plot(path[i][0],path[i][1],'.',color='red',markersize=10)
	
	for i in range(0,len(path)-1):        
		if(i == 0):
			# plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],color='b')
			plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],lineStyle,color=lineColor,label=lineLabel)
		else:
			plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],lineStyle,color=lineColor)
	
	plt.axis('scaled')
	
def highlight_points (points, pointColor):
	for point in points :
		plt.plot(point[0], point[1], '.', color = pointColor, markersize = 10)
		
def draw_circle (x, y, r, circleColor):
	xs = []
	ys = []
	angles = np.arange(0, 2.2*np.pi, 0.5)
	
	for angle in angles :
		xs.append(r*np.cos(angle) + x)
		ys.append(r*np.sin(angle) + y)
	
	plt.plot(xs, ys, '-', color = circleColor)

# specify num of points for each path
def add_more_points (path,num_of_points):
	new_path = []
	
	for i in range (0,len(path)-1):
		segment_x = (path[i+1][0] - path[i][0])/num_of_points
		segment_y = (path[i+1][1] - path[i][1])/num_of_points
		for j in range (0,num_of_points):
			new_point = [(path[i][0] + j*segment_x),(path[i][1] + j*segment_y)]
			new_path.append(new_point)
	
	new_path.append(path[-1])
	
	return new_path

# specify the distance between adjacent points
def add_more_points2 (path,segment_length):
	new_path = []
	for i in range (0,len(path)-1):
		distance = np.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
		num_of_points = int(round(distance/segment_length))
		if num_of_points == 0:
			new_path.append(path[i])
		else:
			segment_x = (path[i+1][0] - path[i][0]) / num_of_points
			segment_y = (path[i+1][1] - path[i][1]) / num_of_points
			for j in range (0,num_of_points):
				new_point = [(path[i][0] + j*segment_x),(path[i][1] + j*segment_y)]
				new_path.append(new_point)
	new_path.append(path[-1])
	
	return new_path 

# reference value 0.1, 0.3, 0.00001
def smoothing (path,weight_data,weight_smooth,tolerance):
	
	smoothed_path = path.copy()
	change = tolerance
	
	while change >= tolerance :
		change = 0.0

		for i in range (1,len(path)-1):
			
			for j in range (0,len(path[i])):
				aux = smoothed_path[i][j]

				smoothed_path[i][j] += weight_data * (path[i][j] - smoothed_path[i][j]) + weight_smooth * (smoothed_path[i-1][j] + smoothed_path[i+1][j] - (2.0 * smoothed_path[i][j]))
				change += np.abs(aux - smoothed_path[i][j])
				
	return smoothed_path

def sgn (num):
	if num >= 0:
		return 1
	else:
		return -1

def findMinAngle (absTargetAngle, currentHeading) :
	minAngle = absTargetAngle - currentHeading
	if minAngle > 180 or minAngle < -180 :
		minAngle = -1 * sgn(minAngle) * (360 - abs(minAngle))
	return minAngle

def autoSmooth (path, maxAngle) :
	currentMax = 0
	param = 0.01
	new_path = path
	firstLoop = True
	counter = 0
	while (currentMax >= maxAngle or firstLoop == True) : # and counter <= 15 :
		param += 0.01
		firstLoop = False
		# counter += 1
		# print('this is the {} iteration'.format(counter))
		new_path = smoothing (path, 0.1, param, 0.1)
		currentMax = 0

		for i in range (1, len(new_path)-2) :
			angle1 = math.atan2(new_path[i][1] - new_path[i-1][1], new_path[i][0] - new_path[i-1][0]) *180/np.pi
			if angle1 < 0 : angle1 += 360
			angle2 = math.atan2(new_path[i+1][1] - new_path[i][1], new_path[i+1][0] - new_path[i][0]) *180/np.pi
			if angle2 < 0 : angle2 += 360
			if abs(findMinAngle(angle2, angle1)) > currentMax :
				currentMax = abs(findMinAngle(angle2, angle1))
	return new_path

# also smooth path and add more points
def path_visualizer (orig_path, fig_size, field_size, segment_length, maxAngle):
	
	path = add_more_points2(orig_path,segment_length)
	path = autoSmooth (path, maxAngle)
	
	field = plt.figure()
	xscale,yscale = fig_size
	path_ax = field.add_axes([0,0,xscale,yscale])
	add_complicated_line(orig_path,'--','grey','original')
	add_complicated_line(path,'--','orange','smoothed')
	
	xMin, yMin, xMax, yMax = field_size
	
	# plot field
	path_ax.plot([xMin,xMax],[yMin,yMin],color='black')
	path_ax.plot([xMin,xMin],[yMin,yMax],color='black')
	path_ax.plot([xMax,xMax],[yMin,yMax],color='black')
	path_ax.plot([xMax,xMin],[yMax,yMax],color='black')
	
	# set grid
	xTicks = np.arange(xMin, xMax+1, 2)
	yTicks = np.arange(yMin, yMax+1, 2)
	
	path_ax.set_xticks(xTicks)
	path_ax.set_yticks(yTicks)
	path_ax.grid(True)
	
	# path_ax.set_xlim(xMin-0.25,xMax+0.25)
	# path_ax.set_ylim(yMin-0.25,yMax+0.25)
	
	# plot start and end
	path_ax.plot(path[0][0],path[0][1],'.',color='blue',markersize=15,label='start')
	path_ax.plot(path[-1][0],path[-1][1],'.',color='green',markersize=15,label='end')
	path_ax.legend()
	return path

# covert to c++ 2d array format
def convert (path, pathName):
	length = 0
	print('double {}[{}][2] = '.format(pathName, len(path)))
	print('{')
	for i in range (0, len(path)):
		length += 1
		print('  {',path[i][0],',', path[i][1],'}, ')
	print('};')
	print('\n')
	print('length = ',length)


if __name__ == "__main__":
	path1 = [[1, 9.5], [3, 11], [6, 9], [9.35, 6.95]]
	path2 = []
	traj_x = np.arange(0, 100, 5)
	traj_y = [math.sin(x / 10.0) * x / 2.0 for x in traj_x]
	# for i in range(0, len(traj_y)):
	# 	path2.append([traj_x[i],traj_y[i]])
	# path_visualizer (original_path, (figure_size), (figure_x_and_y_limits), length_of_each_line_segment, max_angle_change_between_segments)
	path2 = path_visualizer(path1, (1, 1), (0, 0, 12, 12), 0.5, 40)
	# p = add_more_points2(path2,0.5)
	# p = autoSmooth(p,90)
	# gen_x = []
	# gen_y = []
	# for i in range (0,len(p)):
	# 	gen_x.append(p[i][0])
	# 	gen_y.append(p[i][1])
	# plt.plot(traj_x, traj_y,"*",color= "black", linewidth=1, label="original course")
	# plt.plot(gen_x, gen_y,"--",color= "grey", linewidth=1, label="generate course")
	plt.show()