function trajectory = planContinuousWorkspaceTrajectory(t, x, robot)    
    cartesianTrajectory = planContinuousTrajectory(t,x);
    trajectory = @(x__,t__)planJointPositions(x__,t__);
    ik = inverseKinematics;
    ik.RigidBodyTree = robot;
    ik.SolverAlgorithm = 'LevenbergMarquardt';
    ik.SolverParameters.SolutionTolerance = 1.0000e-04;
    ik.SolverParameters.ErrorChangeTolerance = 1.0000e-4;
    %lastPos = [0 0 0 0 0 0];
    
    function jointPositions = planJointPositions(x_,t_)
        q = cartesianTrajectory(t_);
        %x_ = x_.';
        transform = trvec2tform(q(1,1:3))*eul2tform(q(1,4:6));
        [sol, ~] = ik('body6', transform, [0.25 0.25 0.25 1 1 1], x_);
        %sol = getJacobian();
        temp = [q(2,4:6) q(2,1:3)].';        
        jointPositions(1,:) = sol;
        jointPositions(2,:) = inv(robot.geometricJacobian(sol,'body6'))*temp;
        jointPositions(3,:) = [0 0 0 0 0 0];
    end
end