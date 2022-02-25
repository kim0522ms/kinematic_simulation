import time
import numpy as np
import math
import matplotlib.pyplot as plt
from math import atan2, degrees, sin, cos, tan, pi
import math
import numpy as np

class variables:

    def __init__(self) -> None:
        self.v = 0.15
        self.Wmax = 0.0652
        self.alpha = self.Wmax / 0.1
        self.N = 150

        # plots
        self.list_trajecty_x = [0]
        self.list_trajecty_y = [0]
        self.list_w = []
        self.list_dw = []
        self.list_w_integral = []
        self.list_dt = []

        # medium variables
        self.ta = self.Wmax/self.alpha
        self.tb = math.pi/(2*self.Wmax) - self.ta
        if self.tb < 0:
            self.tb = 0
        self.te = 2*self.ta+self.tb
        self.dt = 0

        self.pass_ta = False
        self.pass_ta_tb = False
        pass

    def set_ang_vel_infinite(self):
        self.ta = 0
        self.tb = math.pi/(2*self.Wmax) - self.ta
        if self.tb < 0:
            self.tb = 0
        self.te = 2*self.ta+self.tb
        self.dt = 0

def psi(t, var):
    if t < var.ta:
        val = 0.0
        val += 1 / 2 * var.alpha * t * t
        return val
    elif t < var.ta+var.tb:
        val = 0.0
        val += 1 / 2 * var.alpha * var.ta * var.ta
        val += (t - var.ta) * var.Wmax
        return val
    elif t <= var.te:
        val = 0.0
        val += 1 / 2 * var.alpha * var.ta * var.ta
        val += var.tb * var.Wmax
        val += 1 / 2 * var.ta * var.Wmax - (var.Wmax - var.alpha * (t - var.ta - var.tb)) * (var.te - t) / 2
        return val
    else:
        return None

def Vorient(t, var):
    _psi = psi(t, var)
    
    return np.array([math.cos(_psi), math.sin(_psi)])

def getPsi(t, var):
    return psi(t, var)

p_psi = 0

def xy(t, var):
    pos = np.array([0.0, 0.0])
    pos_2 = 0.0

    for i in var.list_dt:
        # v등속일 때, 매 시간마다 v속도로 이동.
        # v랑 차에 헤딩 vector를 곱하면 차가 순간적으로 움직인 distance가 나오고
        # 그걸 적분하면 된다.
        pos += var.v * Vorient(i) * i
    return pos

def distance(v1, v2):
    return np.sqrt(np.sum((v1 - v2) ** 2)) 


v1 = variables();

v2 = variables();
v2.set_ang_vel_infinite();

fig = plt.figure(figsize=(24, 8))
plt.subplot(133)
plt.title("Integral of ω")
plt.grid(linestyle='--')
plt.ylim(-0.05, 100)
plt.xlim(0, v1.te)

plt.show(block=False)
plt.subplot(132)
plt.title("Differentiated psi (ω)")
plt.axhline(0, linestyle='--', color='gray')
plt.axhline(v1.Wmax, linestyle='--', color='gray')
plt.text(v1.N, v1.Wmax, 'ω max' ,rotation=0, weight='bold')
plt.grid(linestyle='--')
plt.draw()
plt.ylim(-0.05, 0.3)
plt.xlim(0, v1.te)

plt.subplot(131)
plt.title("Vehicle trajectory")
plt.grid(linestyle='--')
plt.xlim(-5,5)
plt.ylim(-5,5)
mybox={'facecolor':'y','edgecolor':'r','boxstyle':'round','alpha':0.5}
plt.draw()

p_car1_x, p_car1_y = 0, 0
p_car1_w = 0

p_car2_x, p_car2_y = 0, 0
p_car2_w = 0;

theta = 0

plt1_car1_ann = None
plt1_car2_ann = None
plt1_tbx1 = None
plt1_tbx2 = None
plt3_car1_txt_w_integral = None
plt3_car2_txt_w_integral = None
plt3_line_start_to_end = None

car1_prop = dict(arrowstyle="-|>,head_width=0.6,head_length=0.8",
            shrinkA=0,shrinkB=0,color='blue')

car2_prop = dict(arrowstyle="-|>,head_width=0.6,head_length=0.8",
            shrinkA=0,shrinkB=0,color='blueviolet')


car1_point1 = None
car1_point2 = None
t_last = t_start = time.time()
while (v1.te >= time.time() - t_start):
#for i in range(0, N):
    t_current = time.time()
    t_elapsed = t_current - t_start
    dt = t_current - t_last
    #print(dt)
    
    if dt > 0:
        if plt1_tbx1 is not None:
            plt1_car1_ann.remove()
            if plt1_car2_ann is not None:
                try:
                    plt1_car2_ann.remove()
                except:
                    pass
            plt1_tbx1.remove()
            if plt1_tbx2 is not None:
                plt1_tbx2.remove()
            plt3_car1_txt_w_integral.remove()
            if plt3_car2_txt_w_integral is not None:
                try:
                    plt3_car2_txt_w_integral.remove()
                except:
                    pass
            

        if t_elapsed > v1.ta and v1.pass_ta == False:
            plt.subplot(132)
            plt.axvline(t_elapsed, linestyle='--', color='gray')
            plt.scatter(t_elapsed, 0, color = 'b')
            plt.text(t_elapsed, 0, '( ta )' ,rotation=0, weight='bold')
            v1.pass_ta = True
            plt.subplot(131)

        if t_elapsed > v1.ta + v1.tb and v1.pass_ta_tb == False:
            plt.subplot(132)
            plt.axvline(t_elapsed, linestyle='--', color='gray')
            plt.scatter(t_elapsed, 0, color = 'b')
            plt.text(t_elapsed, 0,'( ta + tb )',rotation=0, weight='bold')
            v1.pass_ta_tb = True
            plt.subplot(131)

        ################################################################################################################################
        ########## Draw plot 1
        ################################################################################################################################
        plt.subplot(131)
        if car1_point1 is not None:
            # plt.scatter(point1[0], point1[1], color='green')
            # plt.scatter(point2[0], point2[1], color='yellowgreen')
            # plt.plot([point1[0],point2[0]], [point1[1], point2[1]], color='green')

            c = math.dist(car1_point1, car1_point2)
            #deg = math.sin(((w - p_w) / dt) / 2)
            # if theta != 0:
            #     r = (c / 2) / math.degrees(math.sin(theta))
            #     print(r)
                
            #     plt1_tbx2 = plt.text(0,0,'R ' + str(r), bbox=mybox)

        plt1_tbx1 = plt.text(0,-10,'Elapsed ' + str(round(t_elapsed * 1000, 2)) + 'ms', bbox=mybox)
        #pos = xy(t_elapsed)
        ### Car1
        car1_x = p_car1_x + (v1.v * math.cos(psi(t_elapsed, v1)) * dt)
        car1_y = p_car1_y + (v1.v * math.sin(psi(t_elapsed, v1)) * dt)
        
        car1_point1 = [p_car1_x, p_car1_y]
        car1_point2 = [car1_x, car1_y]
        p_car1_x, p_car1_y = car1_x, car1_y
        plt1_car1_ann = plt.annotate('', xy=(car1_x, car1_y), xytext=(p_car1_x, p_car1_y), arrowprops=car1_prop)

        v1.list_trajecty_x.append(car1_x)
        v1.list_trajecty_y.append(car1_y)
        plt.plot(v1.list_trajecty_x, v1.list_trajecty_y, color='b')
        
        ### Car2
        if psi(t_elapsed, v2) is not None:
            car2_x = p_car2_x + (v2.v * math.cos(psi(t_elapsed, v2)) * dt)
            car2_y = p_car2_y + (v2.v * math.sin(psi(t_elapsed, v2)) * dt)

            car2_point1 = [p_car2_x, p_car2_y]
            car2_point2 = [car2_x, car2_y]
            plt1_car2_ann = plt.annotate('', xy=(car2_x, car2_y), xytext=(p_car2_x, p_car2_y), arrowprops=car2_prop)

            p_car2_x, p_car2_y = car2_x, car2_y
            v2.list_trajecty_x.append(car2_x)
            v2.list_trajecty_y.append(car2_y)
            plt.plot(v2.list_trajecty_x, v2.list_trajecty_y, color='blueviolet')

        
        ################################################################################################################################
        ########## Draw plot 2
        ################################################################################################################################
        plt.subplot(132)
        ### Car 1
        car1_w = getPsi(t_elapsed, v1) #velocity * tan(delta) / wheelbase
        v1.list_dw.append((car1_w - p_car1_w) / dt) # dw / dt
        v1.list_w.append(car1_w - p_car1_w)
        v1.list_dt.append(t_current - t_start)
        theta = math.radians(((car1_w - p_car1_w)) / 2)
        p_car1_w = car1_w
        
        plt.plot(v1.list_dt, v1.list_dw, color='sandybrown')

        ### Car 2
        if psi(t_elapsed, v2) is not None:
            car2_w = getPsi(t_elapsed, v2)

            spd = v2.v * 1000
            yaw = (car2_w - p_car2_w) / dt
            resultR = (yaw * 1407 / spd) / 2

            print(math.atan(resultR) * (180 / math.pi) % 360)

            v2.list_dw.append((car2_w - p_car2_w) / dt) # dw / dt
            v2.list_w.append(car2_w - p_car2_w)
            v2.list_dt.append(t_current - t_start)
            theta = math.radians(((car2_w - p_car2_w)) / 2)
            p_car2_w = car2_w
            
            plt.plot(v2.list_dt, v2.list_dw, color='green')

        ################################################################################################################################
        ########## Draw plot 3
        ################################################################################################################################
        plt.subplot(133)

        ### Car 1
        car1_w_integral = 180 / math.pi * sum(v1.list_w)
        v1.list_w_integral.append(car1_w_integral)
        textstr = "%.4f°" % car1_w_integral 
        plt3_car1_txt_w_integral = plt.text(t_elapsed, car1_w_integral + 2, textstr ,rotation=00, weight='bold')
        plt.plot(v1.list_dt, v1.list_w_integral, color='red')

        ### Car 2
        if psi(t_elapsed, v2) is not None:
            car2_w_integral = 180 / math.pi * sum(v2.list_w)
            v2.list_w_integral.append(car2_w_integral)
            textstr = "%.4f°" % car2_w_integral 
            plt3_car2_txt_w_integral = plt.text(t_elapsed, car2_w_integral + 2, textstr ,rotation=00, weight='bold')
            plt.plot(v2.list_dt, v2.list_w_integral, color='sienna')

        #print(w_integral)
    t_last = t_current
    plt.pause(0.0001)

print("Time : {0}".format(t_elapsed))

print(car2_x - car1_x, car2_y - car1_y)

print(180 / math.pi * sum(v1.list_w))

plt.subplot(132)
plt.axvline(v1.N, linestyle='--', color='gray')
plt.scatter(v1.N, 0, color = 'b')
plt.text(v1.N, 0,'( 2 * ta + tb )',rotation=0, weight='bold')
pass_ta_tb = True
plt.subplot(131)

print('test')
