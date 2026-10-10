function R = attitude_rotation(q, tilt_only = false)
  % Body-to-world, right-handed, [w x y z]. No toolbox required.
  q = q / norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
  R = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y);
       2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x);
       2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
  if tilt_only
    yaw = atan2(R(2,1), R(1,1));
    R = [cos(yaw),sin(yaw),0;-sin(yaw),cos(yaw),0;0,0,1] * R;
  endif
endfunction
