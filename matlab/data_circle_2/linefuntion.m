function [a0,a1] = linefuntion(q0,q1,ti,tf)

a1 = (q1-q0)/(tf-ti);
a0 = q0-a1*ti;