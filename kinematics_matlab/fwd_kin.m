L = [0, -0.612, -0.5723, 0, 0, 0]
d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]
a = [pi/2, 0, 0, pi/2, -pi/2, 0]

theta = sym('th', [1, 6])

% warning('off', 'all')
M_total = eye(4)
for i = 1:1:6
  disp(i)
  M = [
    [cos(theta(i)), -sin(theta(i)), 0, L(i)],
    [sin(theta(i)) * cos(a(i)), cos(theta(i)) * cos(a(i)), -sin(a(i)), -sin(a(i)) * d(i)],
    [sin(theta(i)) * sin(a(i)), cos(theta(i)) * sin(a(i)), cos(a(i)), cos(a(i)) * d(i)],
    [0, 0, 0, 1]
  ]
  disp(M)
  M_total = M_total * M
endfor

disp(theta)
disp(M_total)