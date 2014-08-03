colors = [['red', 'green', 'green',   'red', 'red'],
          ['red',   'red', 'green',   'red', 'red'],
          ['red',   'red', 'green', 'green', 'red'],
          ['red',   'red',   'red',   'red', 'red']]

measurements = ['green', 'green', 'green' ,'green', 'green']

p = [[1./20, 1./20, 1./20, 1./20, 1./20],
     [1./20, 1./20, 1./20, 1./20, 1./20],
     [1./20, 1./20, 1./20, 1./20, 1./20],
     [1./20, 1./20, 1./20, 1./20, 1./20]]

motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

sensor_right = 0.7

p_move = 0.8

def show(p):
    for i in range(len(p)):
        print p[i]

#Do not delete this comment!
#Do not use any import statements.
#Adding or changing any code above may
#cause the assignment to be graded incorrectly.

#Enter your code here:
def sense(p, Z):
    q=[[] for row in range(len(p))]
    s=[]
    for row in range(len(p)):
        for col in range(len(p[row])):
            hit = (Z == colors[row][col])
            q[row].append(p[row][col] * (hit * sensor_right + (1-hit) * (1-sensor_right)))
        s.append(sum(q[row]))
    for row in range(len(q)):
        for col in range(len(q[row])):
            q[row][col] = q[row][col] / sum(s)
    return q

def move(p, U):
    q=[[] for row in range(len(p))]
    for row in range(len(p)):
    	for col in range(len(p[row])):
        	s = p_move * p[(row-U[0]) % len(p)][(col-U[1]) % len(p[row])]
        	s = s + (1-p_move) * p[row][col]
        	q[row].append(s)
    return q

for k in range(len(measurements)):
    p = move(p, motions[k])
    p = sense(p, measurements[k])

#Your probability array must be printed 
#with the following code.

show(p)
