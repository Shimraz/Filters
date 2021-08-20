Var = [81 16];
pos = [160 167];
vsum = 0;
for l = 1:length(Var)
    vsum = vsum + 1 / Var(l);
end
psum = 0;
avg = 0;
if length(Var) == length(pos)
    for m = 1:length(pos)
        psum = psum + (pos(m) / Var(m));
        avg = avg + pos(m);
    end
end
avg = avg / length(pos);
Eq_Var = 1 / vsum;
% disp(Eq_Var);
% disp(avg);
llimit = avg-Eq_Var;
ulimit = avg+Eq_Var;
fprintf('Lower Limits is %f \n',llimit);
fprintf('Upper Limits is %f \n', ulimit);
Estimated_pos = psum / vsum;
% disp(Estimated_pos);
Var(end+1) = 169; 
pos(end+1) = 150;
vsum = 0;
psum = 0;
for l = 1:length(Var)
    vsum = vsum + 1 / Var(l);
end
psum = 0;
if length(Var) == length(pos)
    for m = 1:length(pos)
        psum = psum + (pos(m) / Var(m));
    end
end
Eq_Var = 1 / vsum;
% disp(Eq_Var);
Est_pos = psum / vsum;
fprintf('Likely robot position %f \n',Est_pos)
% disp(Est_pos);