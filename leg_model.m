function [leg, bodies] = leg_model
	leg = robotics.RigidBodyTree;
	
	root = robotics.RigidBody('root');
	root_joint = robotics.Joint('root_joint', 'fixed');
	root.Joint = root_joint;
	
	L0 = robotics.RigidBody('L0');
	J0 = robotics.Joint('J0', 'revolute');
	setFixedTransform(J0, trvec2tform([0, -1, 0]));
	J0.JointAxis = [0, 0, 1];
% 	J0.PositionLimits = [-pi/3, pi/3];
	L0.Joint = J0;
	
	L1 = robotics.RigidBody('L1');
	J1 = robotics.Joint('J1', 'revolute');
	J1.JointAxis = [1, 0, 0];
% 	J1.PositionLimits = [-pi/3, pi/3];
	L1.Joint = J1;
	
	L2 = robotics.RigidBody('L2');
	J2 = robotics.Joint('J2', 'revolute');
	J2.JointAxis = [0, 1, 0];
% 	J2.PositionLimits = [-pi/3, pi/3];
	L2.Joint = J2;
	
	L3 = robotics.RigidBody('L3');
	J3 = robotics.Joint('J3', 'revolute');
	setFixedTransform(J3, trvec2tform([0, 0, -3]));
	J3.JointAxis = [0, 1, 0];
% 	J3.PositionLimits = [-pi, 0];
	L3.Joint = J3;
	
	L4 = robotics.RigidBody('L4');
	J4 = robotics.Joint('J4', 'revolute');
	setFixedTransform(J4, trvec2tform([0, 0, -3]));
	J4.JointAxis = [0, 1, 0];
% 	J4.PositionLimits = [-pi/3, pi/3];
	L4.Joint = J4;
	
	L5 = robotics.RigidBody('L5');
	J5 = robotics.Joint('J5', 'revolute');
	J5.JointAxis = [1, 0, 0];
% 	J5.PositionLimits = [-pi/3, pi/3];
	L5.Joint = J5;
	
	foot = robotics.RigidBody('foot');
	foot_end = robotics.Joint('foot_end', 'fixed');
	setFixedTransform(foot_end, trvec2tform([0, 0, -1]));
	foot.Joint = foot_end;
	
	addBody(leg, root, leg.BaseName);
	addBody(leg, L0, 'root');
	addBody(leg, L1, 'L0');
	addBody(leg, L2, 'L1');
	addBody(leg, L3, 'L2');
	addBody(leg, L4, 'L3');
	addBody(leg, L5, 'L4');
	addBody(leg, foot, 'L5');
	
	bodies = {'L0', 'L1', 'L2', 'L3', 'L4', 'L5'};
end
