import matlab.engine
eng = matlab.engine.start_matlab()
tf = eng.isprime(37)
print(tf)

width = 500.0
height = 500.0

#map = eng.occupancyMap(width, height)
map = eng.occupancyMap(10.0, 10.0, 20.0)

pose = [5, 5, 0]
ranges = 3.0*eng.ones(100, 1)
angles = eng.linspace(-eng.pi/2, eng.pi/2, 100)
maxrange = 20

scan = eng.lidarScan(ranges, angles)
eng.insertRay(map, pose, scan, maxrange);

eng.show(map)

