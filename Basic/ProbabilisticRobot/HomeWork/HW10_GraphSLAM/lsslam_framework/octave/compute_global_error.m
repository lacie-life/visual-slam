% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    
    err_tmp = t2v(v2t(edge.measurement)\(x1\x2));
    ei = err_tmp'*edge.information*err_tmp;
    Fx = Fx + ei;
   
    

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z = v2t([edge.measurement;0]);
    X = v2t(x);
    L = v2t([l;0]);
    err_tmp = t2v(Z\(X\L));
    err_tmp = err_tmp(1:2);
    ei = err_tmp'*edge.information*err_tmp;
    Fx = Fx + ei;
    

  end

end
