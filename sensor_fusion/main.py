import matlab.engine
import numpy as np

#Matlab in numpy
#http://mathesaurus.sourceforge.net/matlab-numpy.html

eng = matlab.engine.start_matlab()
tf = eng.isprime(37)
print(tf)

eng.script(nargout=0)
