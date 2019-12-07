function robot = createRigidTreeModel()


T01 = getTransformation(pi/2, 0.03, 0.163, 0);
T12 = getTransformation(0, 0.34, 0, pi/2);
T23 = getTransformation(0,0, 0.197, 0);
T34 = getTransformation(-pi/2, 0, 0.143, 0);
T4f = getTransformation(pi/2,0,0,0)*getTransformation(0, 0, 0.08, 0);

robot = rigidBodyTree;
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body1.Joint = jnt1;
body1.Mass = 0.1;
addBody(robot, body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2,T01);
body2.Joint = jnt2;
body2.Mass = 0.05;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
setFixedTransform(jnt3,T12);
body3.Mass  = 0.05;
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body3 to body2

offset = rigidBody('offset');
jnt_off = rigidBodyJoint('jnt_off','fixed');
setFixedTransform(jnt_off,getTransformation(pi/2, 0.02, 0, 0));
offset.Joint = jnt_off;
offset.Mass = 0;
addBody(robot,offset,'body3'); % Add body3 to body2

body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
setFixedTransform(jnt4,T23);
body4.Joint = jnt4;
body4.Mass  = 0.05;
addBody(robot,body4,'offset'); % Add body4 to body2

body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
setFixedTransform(jnt5,T34);
body5.Joint = jnt5;
body5.Mass  = 0.05;
addBody(robot, body5,'body4'); % Add body4 to body2

body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
setFixedTransform(jnt6,T4f);
body6.Mass  = 0.05;
body6.Joint = jnt6;
addBody(robot,body6,'body5'); % Add body4 to body2

robot.Gravity = [0 0 9.8];
robot.DataFormat = 'row';
end