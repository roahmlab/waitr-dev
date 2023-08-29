% time variable
syms t 'real'

% duration variable
syms T 'real'

% desired initial conditions
syms q0 qd0 qdd0 'real'

% Bezier curve degree
degree = 5;

% Bezier coefficients
b = sym('b', [degree + 1, 1], 'real');

% Bernstein polynomials and their time-derivatives
[B, dB, ddB] = Bezier_kernel_deg5(t / T);

% get trajectories
q = B * b;
qd = dB * b / T;
qdd = ddB * b / T / T;

% compute the actual initial condition
q0_actual = simplify(subs(q, t, 0));
qd0_actual = simplify(subs(qd, t, 0));
qdd0_actual = simplify(subs(qdd, t, 0));

% align the initial conditions
% trying to completely replace b(1) and b(2) but failed
b1_res = solve(q0_actual == q0, b(1));

qd0_actual = subs(qd0_actual, b(1), b1_res);
b2_res = solve(qd0_actual == qd0, b(2));

qdd0_actual = subs(qdd0_actual, [b(1); b(2)], [b1_res; b2_res]);
b3_res = solve(qdd0_actual == qdd0, b(3));

b(1) = b1_res;
b(2) = b2_res;
b(3) = b3_res;



% parameter
syms k 'real'

% parameterize the end position
b(4) = q0 + k;
b(5) = q0 + k;
b(6) = q0 + k;

q = B * b;
qd = dB * b / T;
qdd = ddB * b / T / T;

% scaled variable, guaranteed to range in [0,1]
syms s 'real'

q = subs(q, t / T, s);
qd = subs(qd, t / T, s);
qdd = subs(qdd, t / T, s);

% TODO, write down how to generate create_jrs_online

disp(b);








