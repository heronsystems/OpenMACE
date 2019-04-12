function [tf,controls,effort] = minControlEffort(Ad,Bd,dt,x0,targetLocation,umax,R,bisection)
% minControlEffort computes the minimum control effort |u|^2_R to arrive at
% a target location 'targetLocation' subject to a control saturation. The
% terminal time is varying (or fixed, WILL BE determined after we see how CVX can help sovle the problem)
% 09-Dec-2018, Sheng Cheng

tf = 1; % initial guess

if bisection == 1
    % do this only when bisection option is on
    tf_max = 500;
    tf_prev = tf_max; % placeholder for previous tf attempted
    
    cvx_optval = +Inf;
    
    tf_infeasible_max = -1;
    tf_feasible_min = tf_max;
    
    bisectionLimit = round(log2(tf_max));
    
    j = 0;
    while (j<bisectionLimit)
        j = j+1;
	   R_extention = kron(eye(tf),R);
        
	   cvx_begin
    	   variable x(4,tf+1)
    	   variable u(2*tf,1)
    	   minimize (u'*R_extention*u*dt)
    	   subject to
        	   x(:,1) == x0;
        	   x(1:2,tf+1) == [targetLocation];
        	   abs(u) <= umax;
        	   for k = 1:tf
            	   x(:,k+1) == Ad*x(:,k)+Bd*u(2*k-1:2*k);
        	   end
	   cvx_end
    	
	   if cvx_optval == Inf
    	   tf_infeasible_max = max(tf_infeasible_max,tf);
    	   tf_prev = tf;
    	   tf = round((tf_infeasible_max+tf_feasible_min)/2);
        else
            tf_feasible_min = min(tf_feasible_min,tf);
    	   if abs(tf-tf_prev) > 1
        	   tf_prev = tf;
        	   tf = round((tf_infeasible_max + tf)/2);
    	   else
        	   break;
    	   end
        end
    end
    
    if j == bisectionLimit && tf~=tf_feasible_min
        tf = tf_feasible_min;
        R_extention = kron(eye(tf),R);
        
	   cvx_begin 
    	   variable x(4,tf+1)
    	   variable u(2*tf,1)
    	   minimize (u'*R_extention*u*dt)
    	   subject to
        	   x(:,1) == x0;
        	   x(1:2,tf+1) == [targetLocation];
        	   abs(u) <= umax;
        	   for k = 1:tf
            	   x(:,k+1) == Ad*x(:,k)+Bd*u(2*k-1:2*k);
                end
        cvx_end
    end
    
    if max(isnan(u))
        5;
    end
     
    controls = u;

else
    tf = bisection
    R_extention = kron(eye(tf),R);
    
	cvx_begin quiet
    	variable x(4,tf+1)
    	variable u(2*tf,1)
    	minimize (u'*R_extention*u*dt)
    	subject to
        	x(:,1) == x0;
        	x(1:2,tf+1) == [targetLocation];
        	abs(u) <= umax;
        	for k = 1:tf
            	x(:,k+1) == Ad*x(:,k)+Bd*u(2*k-1:2*k);
        	end
	cvx_end
    controls = 0;
end

effort = cvx_optval;