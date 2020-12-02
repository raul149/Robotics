#! /usr/bin/env python3
import math
from numpy import cos, sin, dot,linalg, pi, cross, subtract, transpose

"""
    # {Raul Aznar Alvarez}
    # {raulaa@kth.se}
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    l1 = 0.07
    l2 = 0.3
    l3 = 0.35
    
    x = x - l1
    c2 = ((x*x + y*y - l2*l2 - l3*l3)/(2*l2*l3))
    s2 = math.sqrt(1 - c2*c2)
    o2 = math.atan2(s2,c2)

    c1 = (((l2 + l3*c2) * x +(l3*s2*y))/(x*x + y*y))
    s1 = (((l2 + l3*c2) * y -(l3*s2*x))/(x*x + y*y))
    o1 = math.atan2(s1,c1)

    q = (o1,o2,z) 

    return q

def kuka_IK(point, R, joint_positions):
    
        x = point[0]
        y = point[1]
        z = point[2]
        q = joint_positions #it must contain 7 elements

        L = 0.4
        M = 0.39
        a = 0.078
        b = 0.311
        ERR = True

        while ERR:
            t01 = [[1,0,0,0],[0,1,0,0],[0,0,1,b],[0,0,0,1]]
            t12 = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
            t23 = [[1,0,0,0],[0,1,0,0],[0,0,1,L],[0,0,0,1]]
            t34 = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
            t45 = [[1,0,0,0],[0,1,0,0],[0,0,1,M],[0,0,0,1]]
            t56 = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
            t67 = [[1,0,0,0],[0,1,0,0],[0,0,1,a],[0,0,0,1]]
            
            r01 = [[cos(q[0]), -sin(q[0]),0, 0],[sin(q[0]), cos(q[0]), 0,0],[0,0,1,0],[0,0,0,1]]
            r12 = [[cos(q[1]), -sin(q[1]),0, 0],[sin(q[1]), cos(q[1]), 0,0],[0,0,1,0],[0,0,0,1]]
            r23 = [[cos(q[2]), -sin(q[2]),0, 0],[sin(q[2]), cos(q[2]), 0,0],[0,0,1,0],[0,0,0,1]]
            r34 = [[cos(q[3]), -sin(q[3]),0, 0],[sin(q[3]), cos(q[3]), 0,0],[0,0,1,0],[0,0,0,1]]
            r45 = [[cos(q[4]), -sin(q[4]),0, 0],[sin(q[4]), cos(q[4]), 0,0],[0,0,1,0],[0,0,0,1]]
            r56 = [[cos(q[5]), -sin(q[5]),0, 0],[sin(q[5]), cos(q[5]), 0,0],[0,0,1,0],[0,0,0,1]]
            r67 = [[cos(q[6]), -sin(q[6]),0, 0],[sin(q[6]), cos(q[6]), 0,0],[0,0,1,0],[0,0,0,1]]

            
            rotalpha01 = [[1,0,0,0],[0,cos(pi/2), -sin(pi/2), 0], [0,sin(pi/2), cos(pi/2), 0],[0,0,0,1]]
            rotalpha12 = [[1,0,0,0],[0,cos(-pi/2), -sin(-pi/2), 0], [0,sin(-pi/2), cos(-pi/2), 0],[0,0,0,1]]
            rotalpha23 = [[1,0,0,0],[0,cos(-pi/2), -sin(-pi/2), 0], [0,sin(-pi/2), cos(-pi/2), 0],[0,0,0,1]]
            rotalpha34 = [[1,0,0,0],[0,cos(pi/2), -sin(pi/2), 0], [0,sin(pi/2), cos(pi/2), 0],[0,0,0,1]]
            rotalpha45 = [[1,0,0,0],[0,cos(pi/2), -sin(pi/2), 0], [0,sin(pi/2), cos(pi/2), 0],[0,0,0,1]]
            rotalpha56 = [[1,0,0,0],[0,cos(-pi/2), -sin(-pi/2), 0], [0,sin(-pi/2), cos(-pi/2), 0],[0,0,0,1]]
            rotalpha67 = [[1,0,0,0],[0,cos(0), -sin(0), 0], [0,sin(0), cos(0), 0],[0,0,0,1]]

            mt01 = dot(t01, r01)
            mt12 = dot(t12, r12)
            mt23 = dot(t23, r23)
            mt34 = dot(t34, r34)
            mt45 = dot(t45, r45)
            mt56 = dot(t56, r56)
            mt67 = dot(t67, r67)

            rmt01 = dot(mt01, rotalpha01)
            rmt12 = dot(mt12, rotalpha12)
            rmt23 = dot(mt23, rotalpha23)
            rmt34 = dot(mt34, rotalpha34)
            rmt45 = dot(mt45, rotalpha45)
            rmt56 = dot(mt56, rotalpha56)
            rmt67 = dot(mt67, rotalpha67)

            #rotation
            mult = [0,0,1,0]
            e1 = dot(rmt01,mult)
            e2 = dot(dot(rmt01,rmt12),mult)
            e3 = dot(dot(dot(rmt01,rmt12),rmt23),mult)
            e4 = dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),mult)
            e5 = dot(dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),rmt45),mult)
            e6 = dot(dot(dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),rmt45),rmt56),mult)
            e7 = dot(dot(dot(dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),rmt45),rmt56),rmt67),mult)

            nn = [[0,0,1],e1[:3],e2[:3],e3[:3],e4[:3],e5[:3],e6[:3],e7[:3]]

            #translation
            mult2 = [0,0,0,1]
            v1 = dot(rmt01,mult2)
            v2 = dot(dot(rmt01,rmt12),mult2)
            v3 = dot(dot(dot(rmt01,rmt12),rmt23),mult2)
            v4 = dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),mult2)
            v5 = dot(dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),rmt45),mult2)
            v6 = dot(dot(dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),rmt45),rmt56),mult2)
            v7 = dot(dot(dot(dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),rmt45),rmt56),rmt67),mult2)
            
            
            pp = [0,0,0]
            v0 = pp
            nm = [v0[:3],v1[:3],v2[:3],v3[:3],v4[:3],v5[:3],v6[:3],v7[:3]]


            jac = [[0,0,0,0,0,0,0], [0,0,0,0,0,0,0], [0,0,0,0,0,0,0], [0,0,0,0,0,0,0], [0,0,0,0,0,0,0], [0,0,0,0,0,0,0]]

            #3-1
            for i in range (0,7):
                ll = cross(nn[i], subtract(nm[7], nm[i]))
                for j in range (0,3):
                    jac[j][i] = ll[j]
                    jac[j+3][i] = nn[i][j]

            #inverse jacobi
            inv_jac = linalg.pinv(jac)

            #error
            ee = dot(dot(dot(dot(dot(dot(rmt01,rmt12),rmt23),rmt34),rmt45),rmt56),rmt67)
        
            ee0 = [ee[0][0],ee[1][0],ee[2][0]]
            ee1 = [ee[0][1],ee[1][1],ee[2][1]]
            ee2 = [ee[0][2],ee[1][2],ee[2][2]]
            R0 = [R[0][0],R[1][0],R[2][0]]
            R1 = [R[0][1],R[1][1],R[2][1]]
            R2 = [R[0][2],R[1][2],R[2][2]]

            s = (cross(ee0,R0) + cross(ee1,R1)+ cross(ee2,R2)/2)

            e11 = subtract(v7[:3], point) 
            e22 = [e11[0],e11[1],e11[2],s[0],s[1],s[2]] 

            d1 = dot(inv_jac, e22)
            q = subtract(q,d1)
            exn = linalg.norm(e22)
       
            if exn<0.25:
                ERR = False

        
        return q
