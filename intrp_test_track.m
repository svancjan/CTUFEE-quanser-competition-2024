close all

%whole route
X = [-0.79 1.4  1.87 2.05 2.16 2.20 2.20 2.20 2.13 2.03 1.82 1.46 1.11 -1.03 -1.53 -1.83 -1.95 -1.96 -1.96];
Y = [ -1.02 -1.02 -0.89 -0.68 -0.47 -0.26  3.00  3.7 4.0 4.2   4.35  4.4   4.42  4.42  4.3   3.97  3.47  1.8   0.8 ];
theta = [0 0 pi/6 pi/3 pi/2 pi/2 pi/2 pi/2 deg2rad(100) deg2rad(110) pi pi pi pi pi deg2rad(260) -pi/2 -pi/2 -pi/2];


%semafor1
X = [   -0.79  1.4   1.87  2.05  2.16  2.20  2.20 ];
Y = [   -1.02 -1.02 -0.89 -0.68 -0.47 -0.26  1.67 ];
theta = [0     0     pi/6  pi/3  pi/2  pi/2  pi/2 ];

%STOP1
X = [   -0.79  1.4   1.87  2.05  2.16  2.20  2.20 ];
Y = [   -1.02 -1.02 -0.89 -0.68 -0.47 -0.26  3.17 ];
theta = [0     0     pi/6  pi/3  pi/2  pi/2  pi/2 ];

%STOP2
X = [   -0.79  1.4   1.87  2.05  2.16  2.20  2.20  2.20 2.13 2.03 1.82  1.46 1.11 -1.17 ];
Y = [   -1.02 -1.02 -0.89 -0.68 -0.47 -0.26  3.00  3.7  4.0  4.2  4.35  4.4   4.42  4.42 ];
theta = [0     0    pi/6 pi/3 pi/2 pi/2 pi/2 pi/2 deg2rad(100) deg2rad(110) pi pi pi pi];

%semafor2
X = [   -0.79  1.4   1.87  2.05  2.16  2.20  2.20  2.20 2.13 2.03 1.82  1.46 1.11 -1.03 -1.53 -1.83 -1.95 -1.96 -1.96 -1.96];
Y = [   -1.02 -1.02 -0.89 -0.68 -0.47 -0.26  3.00  3.7  4.0  4.2  4.35  4.4   4.42  4.42  4.3   3.97  3.47  1.8   0.67  0.5];
theta = [0     0    pi/6 pi/3 pi/2 pi/2 pi/2 pi/2 deg2rad(100) deg2rad(110) pi pi pi pi pi deg2rad(260) -pi/2 -pi/2 -pi/2 -pi/2];


size(theta)

dubinsSpace = stateSpaceDubins([-10 10; -10 10; -pi pi])

dubinsSpace.MinTurningRadius = 0.2;

pathobj = navPath(dubinsSpace)
waypoints = [X' Y' theta']
append(pathobj,waypoints)
interpolate(pathobj,100)

figure
grid on
axis equal
hold on
plot(pathobj.States(:,1),pathobj.States(:,2),".b")
plot(waypoints(:,1),waypoints(:,2),"*r","MarkerSize",10)
len = pathLength(pathobj);
disp("Path length = " + num2str(len))

