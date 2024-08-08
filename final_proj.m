clc
clear all

% frames A,B,A,C,A
global len_shoulder len_hip len_elbow len_arm len_thigh len_leg len_body p_actual

neck = [ 1923  1933  1923  1930 1923;
        1952  1965  1952  1970 1952;
       (2340-893)  (2340-900)  (2340-893)  (2340-896) (2340-893)];

l_shoulder = [  2131  2048  2131  2081 2131;
                1970  1897  1970  2101 1970;
               (2340-1024)  (2340-951)  (2340-1024)  (2340-964) (2340-1024) ];
r_shoulder = [ 1733  1804  1733  1755 1733;
        1970  2200  1970  1901 1970;
       (2340-1024)  (2340-962)  (2340-1024)  (2340-970) (2340-1024)];

l_elbow =[ 2145  2131  2145  2074 2145;
        1954  1682  1954  2273 1954;
       (2340-1310)  (2340-969)  (2340-1310)  (2340-1049) (2340-1310)];
r_elbow =[ 1704  1804  1704  1755 1704;
        1954  2312  1954  1675 1954;
       (2340-1310)  (2340-1053)  (2340-1310)  (2340-963) (2340-1310) ];

l_wrist =[ 2141  2192  2141  2074 2141;
        1930  1490  1930  2404 1930;
       (2340-1563)  (2340-966)  (2340-1563)  (2340-1183) (2340-1563)];
r_wrist =[ 1708  1804  1708  1745 1708;
        1930  2384  1930  1475 1930;
       (2340-1563)  (2340-1180)  (2340-1563)  (2340-958) (2340-1563)];

l_hip = [ 2047  2053  2047  2150 2047;
        1948  2063  1948  2236 1948;
       (2340-1545)  (2340-1395)  (2340-1545)  (2340-1341) (2340-1545)];
r_hip = [ 1792  1780  1792  1895 1792;
        1948  2231  1948  2090 1948;
       (2340-1545)  (2340-1336)  (2340-1545)  (2340-1392) (2340-1545)];

l_knee =[ 2018  1968  2018  2160 2018;
        1943  2138  1943  2422 1943;
       (2340-1953)  (2340-1740)  (2340-1953)  (2340-1614) (2340-1953) ];
r_knee =[ 1813  1773  1813  1920 1813;
        1943  2418  1943  2145 1943;
       (2340-1953)  (2340-1618)  (2340-1953)  (2340-1732) (2340-1953)];

l_ankle =[ 1996  2002  1996  2236 1996;
        1967  2168  1967  2422 1967;
       (2340-2263)  (2340-2042)  (2340-2263)  (2340-1297) (2340-2263)];
r_ankle = [ 1830  1804  1830  1920 1830;
        1963  2417  1963  2171 1963;
       (2340-2241)  (2340-1308)  (2340-2241)  (2340-2037) (2340-2241)];

ang = zeros(23,4);
for i=1:5
    len_shoulder = norm( r_shoulder(:,i)-l_shoulder(:,i) )/2;
    len_hip = norm( r_hip(:,i)-l_hip(:,i) )/2;
    len_elbow = norm( r_shoulder(:,i)-r_elbow(:,i) );
    len_arm = norm( r_elbow(:,i)-r_wrist(:,i) );
    len_thigh = norm( r_hip(:,i)-r_knee(:,i) );
    len_leg = norm( r_knee(:,i)-r_ankle(:,i) );
    len_body = norm( neck(2:3,i)-l_hip(2:3,i) );

    p_actual = [neck(:,i) r_shoulder(:,i) r_elbow(:,i) r_wrist(:,i) l_shoulder(:,i) l_elbow(:,i) l_wrist(:,i) r_hip(:,i) r_knee(:,i) r_ankle(:,i) l_hip(:,i) l_knee(:,i) l_ankle(:,i)];
    x0 = [0 0 0, 0, 0 0 0, 0, 0 0 0, 0, 0 0 0, 0, 0 0 0, 0, neck(:,i)']';
    ang(:,i) = lsqnonlin(@human2, x0);
    
end

% animation b/w frames
N1 = 100;
for i = 2:1:5
    for j=1:N1
        human1(ang(:,i-1)*(N1-j)/N1 + ang(:,i)*j/N1);
        xlim([0 3000]); ylim([0 3000]); zlim([0 3000]);
        view(40,20);
        pause(0.05);   
        clf;
    end
end

%%
function out1 = human2(ang)
global len_shoulder len_hip len_elbow len_arm len_thigh len_leg len_body p_actual

neck = ang(21:23);

r_shoulder = [0; -len_shoulder; 0];
r_elbow = [len_elbow; 0; 0];
r_wrist = [len_arm; 0; 0];

l_shoulder = [0; len_shoulder; 0];
l_elbow = [len_elbow; 0; 0];
l_wrist = [len_arm; 0; 0];

r_hip = [0; -len_hip; -len_body];
r_knee = [len_thigh; 0; 0];
r_ankle = [len_leg; 0; 0];

l_hip = [0; len_hip; -len_body];
l_knee = [len_thigh; 0; 0];
l_ankle = [len_leg; 0; 0];


R_th1 = rotZ(ang(1))*rotY(ang(2))*rotX(ang(3));

r_shoulder_i = R_th1*r_shoulder + neck;
l_shoulder_i = R_th1*l_shoulder + neck;
r_hip_i = R_th1*r_hip + neck;
l_hip_i = R_th1*l_hip + neck;

R_th3 = rotZ(ang(5))*rotY(ang(6))*rotX(ang(7));
R_th4 = rotY(ang(8));
r_elbow_i = R_th1*R_th3*r_elbow +r_shoulder_i;
r_wrist_i = R_th1*R_th3*R_th4*r_wrist + r_elbow_i;
R_th5 = rotZ(ang(9))*rotY(ang(10))*rotX(ang(11));
R_th6 = rotY(ang(12));
l_elbow_i = R_th1*R_th5*l_elbow +l_shoulder_i;
l_wrist_i = R_th1*R_th5*R_th6*l_wrist + l_elbow_i;

R_th7 = rotZ(ang(13))*rotY(ang(14))*rotX(ang(15));
R_th8 = rotY(ang(16));
r_knee_i = R_th1*R_th7*r_knee + r_hip_i;
r_ankle_i = R_th1*R_th7*R_th8*r_ankle + r_knee_i;
R_th9 = rotZ(ang(17))*rotY(ang(18))*rotX(ang(19));
R_th10 = rotY(ang(20));
l_knee_i = R_th1*R_th9*l_knee + l_hip_i;
l_ankle_i = R_th1*R_th9*R_th10*l_ankle + l_knee_i;

z0 = [neck r_shoulder_i r_elbow_i r_wrist_i l_shoulder_i l_elbow_i l_wrist_i r_hip_i r_knee_i r_ankle_i l_hip_i l_knee_i l_ankle_i];
out1 = sum((z0-p_actual).^2,'all');

end

%%
function human1(ang)
global len_shoulder len_hip len_elbow len_arm len_thigh len_leg len_body p_actual

neck = ang(21:23);

r_shoulder = [0; -len_shoulder; 0];     
r_elbow = [len_elbow; 0; 0];         
r_wrist = [len_arm; 0; 0];          

l_shoulder = [0; len_shoulder; 0];
l_elbow = [len_elbow; 0; 0];
l_wrist = [len_arm; 0; 0];

r_hip = [0; -len_hip; -len_body];
r_knee = [len_thigh; 0; 0];
r_ankle = [len_leg; 0; 0];

l_hip = [0; len_hip; -len_body];
l_knee = [len_thigh; 0; 0];
l_ankle = [len_leg; 0; 0];

R_th1 = rotZ(ang(1))*rotY(ang(2))*rotX(ang(3));

r_shoulder_i = R_th1*r_shoulder + neck;
l_shoulder_i = R_th1*l_shoulder + neck;
r_hip_i = R_th1*r_hip + neck;
l_hip_i = R_th1*l_hip + neck;

R_th3 = rotZ(ang(5))*rotY(ang(6))*rotX(ang(7));
R_th4 = rotY(ang(8));
r_elbow_i = R_th1*R_th3*r_elbow +r_shoulder_i;
r_wrist_i = R_th1*R_th3*R_th4*r_wrist + r_elbow_i;
R_th5 = rotZ(ang(9))*rotY(ang(10))*rotX(ang(11));
R_th6 = rotY(ang(12));
l_elbow_i = R_th1*R_th5*l_elbow +l_shoulder_i;
l_wrist_i = R_th1*R_th5*R_th6*l_wrist + l_elbow_i;

R_th7 = rotZ(ang(13))*rotY(ang(14))*rotX(ang(15));
R_th8 = rotY(ang(16));
r_knee_i = R_th1*R_th7*r_knee + r_hip_i;
r_ankle_i = R_th1*R_th7*R_th8*r_ankle + r_knee_i;
R_th9 = rotZ(ang(17))*rotY(ang(18))*rotX(ang(19));
R_th10 = rotY(ang(20));
l_knee_i = R_th1*R_th9*l_knee + l_hip_i;
l_ankle_i = R_th1*R_th9*R_th10*l_ankle + l_knee_i;

%plotting
lin_wid = 3;
%body
vert1 = [r_shoulder_i l_shoulder_i r_hip_i l_hip_i r_shoulder_i];
x = vert1(1,:);
y = vert1(2,:);
z = vert1(3,:);
patch(x,y,z,'b'); axis equal; hold on;

%right arm1
[x,y,z] = plt_point(r_shoulder_i,r_elbow_i);
plot3(x,y,z,'r','LineWidth',lin_wid);
%right arm2
[x,y,z] = plt_point(r_wrist_i,r_elbow_i);
plot3(x,y,z,'g','LineWidth',lin_wid);
%left arm1
[x,y,z] = plt_point(l_shoulder_i,l_elbow_i);
plot3(x,y,z,'r','LineWidth',lin_wid);
%left arm2
[x,y,z] = plt_point(l_wrist_i,l_elbow_i);
plot3(x,y,z,'g','LineWidth',lin_wid);
%right thigh
[x,y,z] = plt_point(r_hip_i,r_knee_i);
plot3(x,y,z,'r','LineWidth',lin_wid);
%right leg
[x,y,z] = plt_point(r_ankle_i,r_knee_i);
plot3(x,y,z,'g','LineWidth',lin_wid);
%left thigh
[x,y,z] = plt_point(l_hip_i,l_knee_i);
plot3(x,y,z,'r','LineWidth',lin_wid);
%left leg
[x,y,z] = plt_point(l_ankle_i,l_knee_i);
plot3(x,y,z,'g','LineWidth',lin_wid);

end

%%
function [x,y,z] = plt_point(pa,pb)
vert1 = [pa,pb];
x = vert1(1,:);
y = vert1(2,:);
z = vert1(3,:);
end

%%
function R1 = rotZ(theta)
R1 = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end

%%
function R1 = rotY(theta)
R1 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end

%%
function R1 = rotX(theta)
R1 = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end
