import scipy.optimize as op
import numpy 
#x = np.array([0,0.85,1])
# y = np.array([0,0.95,1])
# result = op.curve_fit(lambda t, a,b: a+b*np.log(t), x, y)
# print result[0]

x = numpy.array([0,0.08,0.85,1])
y = numpy.array([0,0.04,0.7,1])
print op.curve_fit(lambda t,a,b: a*numpy.exp(b*t),  x,  y)[0]

