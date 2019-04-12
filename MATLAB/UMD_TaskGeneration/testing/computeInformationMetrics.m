function computeInformationMetrics(m, p, q, r, nz)

H_C = cellStateEntropy(p,q,r)
H_G = gridSensorEntropy(p,q,r)
H_Z = targetSensorEntropy(p,q,r,m,nz)
I_GC = mutualInformationGridSensor(p, q, r)
if ( I_GC > min(H_C, H_G) )
   disp('*** Warning I_GC > min(H_C, H_G) ! *** ') 
end
I_ZC = mutualInformationTargetSensor(m, p, q, r, nz)
if ( I_ZC > min(H_Z, H_C) )
   disp('*** Warning I_GC > min(H_C, H_G) ! *** ') 
end
I_C_GZ = mutualInformation(m, p, q, r, nz)
if ( I_C_GZ > min([H_C]) )
   disp('*** Warning I_C_GZ > min([H_C]) ! *** ') 
end
end