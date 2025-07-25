function A=calc_Area(volume)

A_dot=0.0049*0.0049; %single dot Area [m2];
T=[0,0,0];

for y=1:size(volume,2)
    for z=1:size(volume,3)
        w=find(volume(:,y,z)==1);
        
        if ~isempty(w)

            T=[T;[w(end),y,z]];
            S=size(T);
            n=S(1)-1;
            A=n*A_dot;

        end
    end
end

end
