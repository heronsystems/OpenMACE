function log_likelihood_env = projectLikelihood( log_likelihood_targs , Mc )
    Nenv = size(Mc,1);    
    log_likelihood_env = zeros(Nenv,1);
    for i = 1:1:Nenv
        ind = find( Mc(i,:) == 1 );
        log_likelihood_env(i) = sum(log_likelihood_targs(ind));    
    end

end