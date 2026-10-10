function sample = mp65_attitude_sample(block)
  % Application schema adapter. Generic receiver does not know quaternions.
  sample=[]; required={'qw','qx','qy','qz','ready'}; indices=[];
  for k=1:numel(required)
    index=find(strcmp({block.channels.name},required{k}));
    if numel(index)!=1, return; endif
    indices(k)=index;
  endfor
  v=block.values(indices,end); q=v(1:4)';
  if any(!isfinite(v)) || abs(norm(q)-1)>0.01 || !ismember(v(5),[0,1]), return; endif
  sample=struct('q',q/norm(q),'ready',logical(v(5)));
endfunction
