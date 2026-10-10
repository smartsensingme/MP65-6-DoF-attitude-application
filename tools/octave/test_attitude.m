addpath(fileparts(mfilename('fullpath')));
assert(attitude_rotation([1 0 0 0]),eye(3),1e-12);
q=[cos(pi/4),0,0,sin(pi/4)];
assert(attitude_rotation(q)*[1;0;0],[0;1;0],1e-12);
assert(attitude_rotation(q,true),eye(3),1e-12);
assert(attitude_rotation(-q),attitude_rotation(q),1e-12);
block=struct('channels',struct('name',{'qw','qx','qy','qz','ready'}), ...
             'values',[round(q'*32767)/32767;1]);
s=mp65_attitude_sample(block); assert(s.ready);
assert(norm(s.q-q)<4e-5);
block.values(5)=2; assert(isempty(mp65_attitude_sample(block)));
disp('MP65 attitude adapter and rotation tests passed');
