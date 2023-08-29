function joint_position_series = forward_kinematics_modified(q, T0, joint_axes)

    T = eye(4);
    joint_position_series = zeros(3,length(q));

    for i = 1 : length(q)
        dim = find(joint_axes(:,i) ~= 0);
        if dim == 1
            R = rx(q(i));
        elseif dim == 2
            R = ry(q(i));
        else
            R = rz(q(i));
        end

        T = T * T0(:,:,i) * [R [0;0;0]; 0 0 0 1];
        joint_position_series(:,i) = T(1:3,4);
    end
end