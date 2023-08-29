function T = forward_kinematics_symbolic(q, T0, joint_axes)

    T = sym(eye(4));

    threshold = 1e-4;

    for i = 1:size(T0, 3)
        R = T0(1:3,1:3,i);
        p = T0(1:3,4,i);
        
        R(abs(R) <= threshold) = 0;
        R(abs(R - 1) <= threshold) = 1;
        R(abs(R + 1) <= threshold) = -1;
        p(abs(p) <= threshold) = 0;
        
        T0(1:3,1:3,i) = R;
        T0(1:3,4,i) = p;
    end

    for i =1:length(q)
        dim = find(joint_axes(:,i) ~=0);
        if dim == 1
            R = rx(q(i));
        elseif dim == 2
            R = ry(q(i));
        else
            R = rz(q(i));
        end

        T = T * T0(:,:,i) * [R [0;0;0]; 0 0 0 1];
    end
end