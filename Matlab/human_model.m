function [human] = human_model(th1,th2,th3,th4,th5,th6,th7)

global human
global q0
global qmin
global qmax
global xdes

%% DH parameters

human = robotics.RigidBodyTree;
right_arm = robotics.RigidBodyTree;
left_arm = robotics.RigidBodyTree;
trunk = robotics.RigidBodyTree;
head = robotics.RigidBodyTree;
upper_body_right = robotics.RigidBodyTree;
upper_body_left = robotics.RigidBodyTree;

T_trunk_flexion = [0 0 -1 0;0 -1 0 0;1 0 0 0; 0,0,0,1];
T_trunk_lateral_flexion = [0 1 0 0;0 0 1 0;1 0 0 0;0 0 0 1];
T_trunk_rotation = [1 0 0 0;0 0 1 0.58;0 1 0 0;0 0 0 1];

T_clavicle_abduction = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
T_clavicle_elevation = [1 0 0 0;0 0 1 0;0 1 0 0;0 0 0 1];

T_shoulder_flexion = [0 0 1 0.2;1 0 0 0;0 -1 0 0;0 0 0 1];
T_shoulder_abduction = [0 1 0 0;0 0 1 0;1 0 0 0;0 0 0 1];
T_shoulder_rotation = [0 1 0 0;0 0 1 -0.31;1 0 0 0;0 0 0 1];

T_elbow = [-1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1];

T_wrist_pronation = [1 0 0 0;0 0 -1 -0.28;0 1 0 0;0 0 0 1];
T_wrist_flexion = [0 1 0 0;0 0 1 0;1 0 0 0;0 0 0 1];
T_wrist_abduction = [0 1 0 0;0 0 1 0;1 0 0 0;0 0 0 1];

T_wrist_tip = [0 1 0 0;0 0 1 9.3*0.01;1 0 0 0;0 0 0 1];

T_hip_flexion = [0 0 1 -0.16;0 1 0 0;1 0 0 0;0 0 0 1];
T_hip_flexion_left = [0 0 -1 0.16;0 1 0 0;1 0 0 0;0 0 0 1];
T_hip_abduction = [0 1 0 0;0 0 1 0;1 0 0 0;0 0 0 1];
T_hip_rotation = [0 1 0 0;0 0 1 -0.43;1 0 0 0;0 0 0 1];

T_knee = [1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1];

T_foot = [1 0 0 0;0 0 -1 -0.40;0 1 0 0;0 0 0 1];

T_head_flexion = [0 0 1 0;0 1 0 0;1 0 0 0;0 0 0 1];
T_head_abduction = [0 1 0 0;0 0 1 0;1 0 0 0;0 0 0 1];
T_head_rotation = [0 1 0 0;0 0 1 0.05;1 0 0 0;0 0 0 1];


%% Trunk
trunk_flexion = robotics.RigidBody('trunk_flexion');
trunk_flexion_joint = robotics.Joint('trunk_flexion_joint','revolute');
setFixedTransform(trunk_flexion_joint,T_trunk_flexion);
trunk_flexion.Joint = trunk_flexion_joint;
addVisual(trunk_flexion, "Mesh", "link_mesh.stl");
addBody(human, trunk_flexion,'base');

trunk_lateral_flexion = robotics.RigidBody('trunk_lateral_flexion');
trunk_lateral_flexion_joint = robotics.Joint('trunk_lateral_flexion_joint','revolute');
setFixedTransform(trunk_lateral_flexion_joint,T_trunk_lateral_flexion);
trunk_lateral_flexion.Joint = trunk_lateral_flexion_joint;
addBody(human, trunk_lateral_flexion,'trunk_flexion');

trunk_rotation = robotics.RigidBody('trunk_rotation');
trunk_rotation_joint = robotics.Joint('trunk_rotation_joint','revolute');
setFixedTransform(trunk_rotation_joint,T_trunk_rotation);
trunk_rotation.Joint = trunk_rotation_joint;
addBody(human, trunk_rotation,'trunk_lateral_flexion');

%% Clavicle (right arm)

clavicle_abduction_right = robotics.RigidBody('clavicle_abduction_right');
clavicle_abduction_right_joint = robotics.Joint('clavicle_abduction_right_joint','revolute');
setFixedTransform(clavicle_abduction_right_joint,T_clavicle_abduction);
clavicle_abduction_right.Joint = clavicle_abduction_right_joint;
addBody(human, clavicle_abduction_right, 'trunk_rotation');

clavicle_elevation_right = robotics.RigidBody('clavicle_elevation_right');
clavicle_elevation_right_joint = robotics.Joint('clavicle_elevation_right_joint','revolute');
setFixedTransform(clavicle_elevation_right_joint,T_clavicle_elevation);
clavicle_elevation_right.Joint = clavicle_elevation_right_joint;
addBody(human, clavicle_elevation_right, 'clavicle_abduction_right');

%% Shoulder (right arm)

shoulder_flexion_right = robotics.RigidBody('shoulder_flexion_right');
shoulder_flexion_right_joint = robotics.Joint('shoulder_flexion_right_joint','revolute');
setFixedTransform(shoulder_flexion_right_joint,T_shoulder_flexion);
shoulder_flexion_right.Joint = shoulder_flexion_right_joint;
addBody(human, shoulder_flexion_right, 'clavicle_elevation_right');

shoulder_abduction_right = robotics.RigidBody('shoulder_abduction_right');
shoulder_abduction_right_joint = robotics.Joint('shoulder_abduction_right_joint','revolute');
setFixedTransform(shoulder_abduction_right_joint,T_shoulder_abduction);
shoulder_abduction_right.Joint = shoulder_abduction_right_joint;
addBody(human, shoulder_abduction_right, 'shoulder_flexion_right');

shoulder_rotation_right = robotics.RigidBody('shoulder_rotation_right');
shoulder_rotation_right_joint = robotics.Joint('shoulder_rotation_right_joint','revolute');
setFixedTransform(shoulder_rotation_right_joint,T_shoulder_rotation);
shoulder_rotation_right.Joint = shoulder_rotation_right_joint;
addBody(human, shoulder_rotation_right, 'shoulder_abduction_right');

%% Elbow (right arm)

elbow_flexion_right = robotics.RigidBody('elbow_flexion_right');
elbow_flexion_right_joint = robotics.Joint('elbow_flexion_right_joint','revolute');
setFixedTransform(elbow_flexion_right_joint,T_elbow);
elbow_flexion_right.Joint = elbow_flexion_right_joint;
addBody(human, elbow_flexion_right, 'shoulder_rotation_right');

%% Wrist (right arm)

wrist_pronation_right = robotics.RigidBody('wrist_pronation_right');
wrist_pronation_right_joint = robotics.Joint('wrist_pronation_right_joint','revolute');
setFixedTransform(wrist_pronation_right_joint,T_wrist_pronation);
wrist_pronation_right.Joint = wrist_pronation_right_joint;
addBody(human, wrist_pronation_right, 'elbow_flexion_right');

syms th1 th2 th3 th4 th5 th6 th7

geometricJacobian(human, , 'wrist_pronation_right')

return;
%% Left arm

clavicle_abduction_left = copy(clavicle_abduction_right);
clavicle_abduction_left.Name = 'clavicle_abduction_left';
clavicle_abduction_left.Joint.Name = 'clavicle_abduction_left_joint';
setFixedTransform(clavicle_abduction_left.Joint,[[-1 0 0;0 1 0;0 0 1] [0;0;0]; [0,0,0,1]]);
addBody(human, clavicle_abduction_left, 'trunk_rotation');

clavicle_elevation_left = copy(clavicle_elevation_right);
clavicle_elevation_left.Name = 'clavicle_elevation_left';
clavicle_elevation_left.Joint.Name = 'clavicle_elevation_left_joint';
addBody(human, clavicle_elevation_left, 'clavicle_abduction_left');

shoulder_flexion_left = copy(shoulder_flexion_right);
shoulder_flexion_left.Name = 'shoulder_flexion_left';
shoulder_flexion_left.Joint.Name = 'shoulder_flexion_left_joint';
addBody(human, shoulder_flexion_left, 'clavicle_elevation_left');

shoulder_abduction_left = copy(shoulder_abduction_right);
shoulder_abduction_left.Name = 'shoulder_abduction_left';
shoulder_abduction_left.Joint.Name = 'shoulder_abduction_left_joint';
addBody(human, shoulder_abduction_left, 'shoulder_flexion_left');

shoulder_rotation_left = copy(shoulder_rotation_right);
shoulder_rotation_left.Name = 'shoulder_rotation_left';
shoulder_rotation_left.Joint.Name = 'shoulder_rotation_left_joint';
addBody(human, shoulder_rotation_left, 'shoulder_abduction_left');

elbow_flexion_left = copy(elbow_flexion_right);
elbow_flexion_left.Name = 'elbow_flexion_left';
elbow_flexion_left.Joint.Name = 'elbow_flexion_left_joint';
addBody(human, elbow_flexion_left, 'shoulder_rotation_left');

wrist_pronation_left = copy(wrist_pronation_right);
wrist_pronation_left.Name = 'wrist_pronation_left';
wrist_pronation_left.Joint.Name = 'wrist_pronation_left_joint';
addBody(human, wrist_pronation_left, 'elbow_flexion_left');

%% Head

head_flexion = robotics.RigidBody('head_flexion');
head_flexion_joint = robotics.Joint('head_flexion_joint','fixed');
setFixedTransform(head_flexion_joint,T_head_flexion);
head_flexion.Joint = head_flexion_joint;
addBody(human, head_flexion,'trunk_rotation');

head_abduction = robotics.RigidBody('head_abduction');
head_abduction_joint = robotics.Joint('head_abduction_joint','fixed');
setFixedTransform(head_abduction_joint,T_head_abduction);
head_abduction.Joint = head_abduction_joint;
addBody(human, head_abduction,'head_flexion');

head_rotation = robotics.RigidBody('head_rotation');
head_rotation_joint = robotics.Joint('head_rotation_joint','fixed');
setFixedTransform(head_rotation_joint,T_head_rotation);
head_rotation.Joint = head_rotation_joint;
addBody(human, head_rotation,'head_abduction');

%% Upperbody right

%addSubtree(upper_body_right, 'trunk_rotation', right_arm)

%% Hip (right leg)

hip_flexion_right = robotics.RigidBody('hip_flexion_right');
hip_flexion_right_joint = robotics.Joint('hip_flexion_right_joint','fixed');
setFixedTransform(hip_flexion_right_joint, T_hip_flexion);
hip_flexion_right.Joint = hip_flexion_right_joint;
addBody(human, hip_flexion_right, 'base');

hip_abduction_right = robotics.RigidBody('hip_abduction_right');
hip_abduction_right_joint = robotics.Joint('hip_abduction_right_joint','fixed');
setFixedTransform(hip_abduction_right_joint,T_hip_abduction);
hip_abduction_right.Joint = hip_abduction_right_joint;
addBody(human, hip_abduction_right, 'hip_flexion_right');

hip_rotation_right = robotics.RigidBody('hip_rotation_right');
hip_rotation_right_joint = robotics.Joint('hip_rotation_right_joint','fixed');
setFixedTransform(hip_rotation_right_joint,T_hip_rotation);
hip_rotation_right.Joint = hip_rotation_right_joint;
addBody(human, hip_rotation_right, 'hip_abduction_right');

%% Knee (right leg)

knee_flexion_right = robotics.RigidBody('knee_flexion_right');
knee_flexion_right_joint = robotics.Joint('knee_flexion_right_joint','fixed');
setFixedTransform(knee_flexion_right_joint,T_knee);
knee_flexion_right.Joint = knee_flexion_right_joint;
addBody(human, knee_flexion_right, 'hip_rotation_right');

%% Foot (right leg)

foot_right = robotics.RigidBody('foot_right');
foot_right_joint = robotics.Joint('foot_right_joint','fixed');
setFixedTransform(foot_right_joint,T_foot);
foot_right.Joint = foot_right_joint;
addBody(human, foot_right, 'knee_flexion_right');

%% Left leg

hip_flexion_left = copy(hip_flexion_right);
hip_flexion_left.Name = 'hip_flexion_left';
hip_flexion_left.Joint.Name = 'hip_flexion_left_joint';
setFixedTransform(hip_flexion_left.Joint,T_hip_flexion_left);
addBody(human, hip_flexion_left, 'base');

hip_abduction_left = copy(hip_abduction_right);
hip_abduction_left.Name = 'hip_abduction_left';
hip_abduction_left.Joint.Name = 'hip_abduction_left_joint';
addBody(human, hip_abduction_left, 'hip_flexion_left');

hip_rotation_left = copy(hip_rotation_right);
hip_rotation_left.Name = 'hip_rotation_left';
hip_rotation_left.Joint.Name = 'hip_rotation_left_joint';
addBody(human, hip_rotation_left, 'hip_abduction_left');

knee_flexion_left = copy(knee_flexion_right);
knee_flexion_left.Name = 'knee_flexion_left';
knee_flexion_left.Joint.Name = 'knee_flexion_left_joint';
addBody(human, knee_flexion_left, 'hip_rotation_left');

foot_left = copy(foot_right);
foot_left.Name = 'foot_left';
foot_left.Joint.Name = 'foot_left_joint';
addBody(human, foot_left, 'knee_flexion_left');

%% Limits

n = length(q0);

% human.Bodies{1,1}.Joint.PositionLimits = 0*[0 90]*3.1415/180;
% human.Bodies{1,2}.Joint.PositionLimits = [-30 30]*3.1415/180;
% human.Bodies{1,3}.Joint.PositionLimits = 0*[-90 90]*3.1415/180;
% human.Bodies{1,4}.Joint.PositionLimits = 0*[-20 20]*3.1415/180;
% human.Bodies{1,5}.Joint.PositionLimits = [-20 20]*3.1415/180;
% human.Bodies{1,6}.Joint.PositionLimits = 0*[-70 180]*3.1415/180;
% %human.Bodies{1,6}.Joint.PositionLimits = [20.2 20.2]*3.1415/180;
% human.Bodies{1,7}.Joint.PositionLimits = [0 130]*3.1415/180;
% %human.Bodies{1,8}.Joint.PositionLimits = [0 30]*3.1415/180;
% human.Bodies{1,8}.Joint.PositionLimits = [-90 -90]*3.1415/180;
% human.Bodies{1,9}.Joint.PositionLimits = [0 130]*3.1415/180;
% human.Bodies{1,10}.Joint.PositionLimits = 0*[-90 90]*3.1415/180;
% human.Bodies{1,11}.Joint.PositionLimits = 0*[-60 60]*3.1415/180;
% human.Bodies{1,12}.Joint.PositionLimits = 0*[-15 15]*3.1415/180;

q0 = homeConfiguration(human);

q0(1).JointPosition = th1;
q0(2).JointPosition = th2;
q0(3).JointPosition = th3;
q0(6).JointPosition = th4;
q0(7).JointPosition = th5;
q0(8).JointPosition = th6;
q0(9).JointPosition = th7;


reba = [7 7 7 5 4 4 3 3 3 2 2 2 5 4 4 3 3 3 2 2 2];
qmin = zeros(n,1);
qmax = zeros(n,1);

% j = 1;
% k = 1;
% while j<n
%     if strcmp(human.Bodies{1,k}.Joint.Type, 'fixed') == 0
%         qmin(j) = human.Bodies{1,k}.Joint.PositionLimits(1);
%         qmax(j) = human.Bodies{1,k}.Joint.PositionLimits(2);
%         j = j + 1;
%     end
%     k = k + 1;
% end

%syms th1 th2 th3 th4 th5 th6 th7


% th1 = q0(1).JointPosition;
% th2 = q0(2).JointPosition;
% th3 = 0;
% th4 = 0;
% th5 = 0;
% th6 = 0;
% th7 = 0;


T_tot = T_trunk_flexion*transfZ(th1)*T_trunk_lateral_flexion*transfZ(th2)*T_trunk_rotation*transfZ(th3)*...
    T_clavicle_abduction*T_clavicle_elevation*T_shoulder_flexion*transfZ(th4)*T_shoulder_abduction*transfZ(th5)*...
    T_shoulder_rotation*transfZ(th6)*T_elbow*transfZ(th7)*T_wrist_pronation;


T_tot

%T_value = subs(T_tot,{th1,th2,th3,th4,th5,th6,th7},{1,0,0,0,0,0,0})

%simplify(T_value)
figure;
%show(human,q0);
drawnow;